//! ISO 15765-2 transport layer: frame codec, segmentation and reassembly.
//!
//! Pure logic without I/O or timers so it can be unit-tested; `channel.rs` drives it.
//! Covers classic CAN and CAN FD (escape single frames and 32-bit first-frame length),
//! normal and extended addressing, and optional padding.

use crate::j2534::round_up_fd_len;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FcStatus {
    ContinueToSend,
    Wait,
    Overflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Pci {
    Single { len: usize, off: usize },
    First { total: usize, off: usize },
    Consecutive { sn: u8, off: usize },
    FlowControl { status: FcStatus, bs: u8, stmin: u8 },
}

/// Parse the PCI of a CAN payload (address byte, if any, already stripped).
pub fn parse_pci(p: &[u8]) -> Result<Pci, &'static str> {
    let b0 = *p.first().ok_or("empty payload")?;
    match b0 >> 4 {
        0 => {
            let n = (b0 & 0xf) as usize;
            if n != 0 {
                if n > p.len() - 1 || n > 7 {
                    return Err("single frame length exceeds frame");
                }
                Ok(Pci::Single { len: n, off: 1 })
            } else {
                // CAN FD escape sequence: only valid in frames longer than 8 bytes.
                if p.len() <= 8 {
                    return Err("single frame length 0");
                }
                let n = *p.get(1).ok_or("short escape single frame")? as usize;
                if n == 0 || n > p.len() - 2 {
                    return Err("escape single frame length exceeds frame");
                }
                Ok(Pci::Single { len: n, off: 2 })
            }
        }
        1 => {
            if p.len() < 2 {
                return Err("short first frame");
            }
            let total = (((b0 & 0xf) as usize) << 8) | p[1] as usize;
            if total != 0 {
                Ok(Pci::First { total, off: 2 })
            } else {
                if p.len() < 6 {
                    return Err("short escape first frame");
                }
                let total = u32::from_be_bytes([p[2], p[3], p[4], p[5]]) as usize;
                Ok(Pci::First { total, off: 6 })
            }
        }
        2 => Ok(Pci::Consecutive {
            sn: b0 & 0xf,
            off: 1,
        }),
        3 => {
            if p.len() < 3 {
                return Err("short flow control frame");
            }
            let status = match b0 & 0xf {
                0 => FcStatus::ContinueToSend,
                1 => FcStatus::Wait,
                2 => FcStatus::Overflow,
                _ => return Err("invalid flow status"),
            };
            Ok(Pci::FlowControl {
                status,
                bs: p[1],
                stmin: p[2],
            })
        }
        _ => Err("invalid PCI type"),
    }
}

/// Separation time from the STmin byte of a flow control frame.
pub fn decode_stmin(raw: u8) -> Duration {
    match raw {
        0..=0x7f => Duration::from_millis(raw as u64),
        0xf1..=0xf9 => Duration::from_micros((raw - 0xf0) as u64 * 100),
        _ => Duration::from_millis(127), // reserved values: treat as the maximum
    }
}

/// How outgoing frames are shaped.
#[derive(Clone, Copy, Debug)]
pub struct TxParams {
    /// Maximum CAN payload length (8 for classic, up to 64 for FD).
    pub tx_dl: usize,
    /// `Some(value)` pads frames; always padded to a valid FD length on FD frames.
    pub pad: Option<u8>,
    pub fd: bool,
    pub ext_addr: Option<u8>,
}

impl TxParams {
    fn addr_len(&self) -> usize {
        self.ext_addr.is_some() as usize
    }

    fn finish(&self, mut frame: Vec<u8>) -> Vec<u8> {
        let target = if self.fd && frame.len() > 8 {
            // FD lengths between 9 and 63 are sparse; always round up.
            round_up_fd_len(frame.len()).unwrap_or(frame.len())
        } else if self.pad.is_some() {
            8.max(frame.len())
        } else {
            frame.len()
        };
        frame.resize(target, self.pad.unwrap_or(0x00));
        frame
    }

    fn with_addr(&self, header: &[u8]) -> Vec<u8> {
        let mut v = Vec::with_capacity(self.tx_dl);
        v.extend(self.ext_addr);
        v.extend_from_slice(header);
        v
    }
}

/// Largest message that fits one single frame.
pub fn single_frame_capacity(p: &TxParams) -> usize {
    if p.tx_dl <= 8 {
        p.tx_dl - 1 - p.addr_len()
    } else {
        p.tx_dl - 2 - p.addr_len()
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum FirstSegment {
    Single(Vec<u8>),
    /// First frame payload and the number of message bytes it carries.
    First(Vec<u8>, usize),
}

pub fn first_segment(data: &[u8], p: &TxParams) -> FirstSegment {
    let addr = p.addr_len();
    let classic_limit = 7 - addr;
    if data.len() <= classic_limit {
        let mut f = p.with_addr(&[data.len() as u8]);
        f.extend_from_slice(data);
        return FirstSegment::Single(p.finish(f));
    }
    if data.len() <= single_frame_capacity(p) && p.tx_dl > 8 {
        let mut f = p.with_addr(&[0x00, data.len() as u8]);
        f.extend_from_slice(data);
        return FirstSegment::Single(p.finish(f));
    }
    let header: Vec<u8> = if data.len() <= 0xfff {
        vec![0x10 | (data.len() >> 8) as u8, data.len() as u8]
    } else {
        let mut h = vec![0x10, 0x00];
        h.extend_from_slice(&(data.len() as u32).to_be_bytes());
        h
    };
    let take = p.tx_dl - addr - header.len();
    let mut f = p.with_addr(&header);
    f.extend_from_slice(&data[..take]);
    FirstSegment::First(f, take)
}

/// Build the next consecutive frame; returns the payload and bytes consumed.
pub fn consecutive_frame(data: &[u8], sn: u8, p: &TxParams) -> (Vec<u8>, usize) {
    let take = (p.tx_dl - p.addr_len() - 1).min(data.len());
    let mut f = p.with_addr(&[0x20 | (sn & 0xf)]);
    f.extend_from_slice(&data[..take]);
    (p.finish(f), take)
}

pub fn flow_control_frame(status: FcStatus, bs: u8, stmin: u8, p: &TxParams) -> Vec<u8> {
    let s = match status {
        FcStatus::ContinueToSend => 0x30,
        FcStatus::Wait => 0x31,
        FcStatus::Overflow => 0x32,
    };
    // Flow control is always sent as a short classic-sized frame.
    let q = TxParams {
        tx_dl: 8,
        fd: false,
        ..*p
    };
    q.finish(q.with_addr(&[s, bs, stmin]))
}

/// What the receiver wants done after a frame.
#[derive(Debug, PartialEq, Eq)]
pub enum RxEvent {
    Ignore,
    /// Complete message (user data only).
    Message(Vec<u8>),
    /// First frame accepted: announce start of message and send flow control.
    Started {
        total: usize,
    },
    SendFlowControl(FcStatus),
    /// Reception aborted; `reason` is for diagnostics.
    Abort(&'static str),
}

/// Segmented-message receiver for one flow-control filter.
pub struct Reassembler {
    buf: Vec<u8>,
    total: usize,
    next_sn: u8,
    since_fc: u32,
    active: bool,
    max_total: usize,
}

impl Reassembler {
    pub fn new(max_total: usize) -> Self {
        Self {
            buf: Vec::new(),
            total: 0,
            next_sn: 0,
            since_fc: 0,
            active: false,
            max_total,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn reset(&mut self) {
        self.active = false;
        self.buf.clear();
    }

    /// Feed one CAN payload (address byte stripped). `bs` is our configured block size.
    /// A `SendFlowControl(ContinueToSend)` after `Started` is implied by the caller.
    pub fn feed(&mut self, payload: &[u8], bs: u8) -> RxEvent {
        let pci = match parse_pci(payload) {
            Ok(p) => p,
            Err(e) => {
                self.reset();
                return RxEvent::Abort(e);
            }
        };
        match pci {
            Pci::Single { len, off } => {
                self.reset();
                RxEvent::Message(payload[off..off + len].to_vec())
            }
            Pci::First { total, off } => {
                self.reset();
                if total > self.max_total {
                    return RxEvent::SendFlowControl(FcStatus::Overflow);
                }
                let chunk = &payload[off..];
                if chunk.len() >= total {
                    // Malformed: a first frame must be shorter than the message.
                    return RxEvent::Abort("first frame carries whole message");
                }
                self.buf.extend_from_slice(chunk);
                self.total = total;
                self.next_sn = 1;
                self.since_fc = 0;
                self.active = true;
                RxEvent::Started { total }
            }
            Pci::Consecutive { sn, off } => {
                if !self.active {
                    return RxEvent::Ignore;
                }
                if sn != self.next_sn {
                    self.reset();
                    return RxEvent::Abort("wrong sequence number");
                }
                let remaining = self.total - self.buf.len();
                let chunk = &payload[off..];
                self.buf
                    .extend_from_slice(&chunk[..chunk.len().min(remaining)]);
                self.next_sn = (sn + 1) & 0xf;
                self.since_fc += 1;
                if self.buf.len() >= self.total {
                    self.active = false;
                    return RxEvent::Message(std::mem::take(&mut self.buf));
                }
                if bs != 0 && self.since_fc >= bs as u32 {
                    self.since_fc = 0;
                    return RxEvent::SendFlowControl(FcStatus::ContinueToSend);
                }
                RxEvent::Ignore
            }
            Pci::FlowControl { .. } => RxEvent::Ignore,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(tx_dl: usize, pad: Option<u8>, ext: Option<u8>) -> TxParams {
        TxParams {
            tx_dl,
            pad,
            fd: tx_dl > 8,
            ext_addr: ext,
        }
    }

    #[test]
    fn tester_present_single_frame_padded_like_the_bus_capture() {
        // example request: 02 3E 00 AA AA AA AA AA
        let seg = first_segment(&[0x3e, 0x00], &params(8, Some(0xaa), None));
        assert_eq!(
            seg,
            FirstSegment::Single(vec![0x02, 0x3e, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa])
        );
        // unpadded classic: exact length
        assert_eq!(
            first_segment(&[0x3e, 0x00], &params(8, None, None)),
            FirstSegment::Single(vec![0x02, 0x3e, 0x00])
        );
        // response parses back to the user data
        assert_eq!(
            parse_pci(&[0x02, 0x7e, 0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa]),
            Ok(Pci::Single { len: 2, off: 1 })
        );
    }

    #[test]
    fn extended_addressing_shrinks_capacity() {
        let p = params(8, None, Some(0x42));
        assert_eq!(single_frame_capacity(&p), 6);
        match first_segment(&[1, 2, 3, 4, 5, 6], &p) {
            FirstSegment::Single(f) => assert_eq!(f, vec![0x42, 6, 1, 2, 3, 4, 5, 6]),
            other => panic!("{other:?}"),
        }
        match first_segment(&[1, 2, 3, 4, 5, 6, 7], &p) {
            FirstSegment::First(f, n) => {
                assert_eq!(f, vec![0x42, 0x10, 7, 1, 2, 3, 4, 5]);
                assert_eq!(n, 5);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn fd_uses_escape_single_frame_and_rounds_to_valid_length() {
        let p = params(64, None, None);
        let data: Vec<u8> = (0..10).collect();
        match first_segment(&data, &p) {
            FirstSegment::Single(f) => {
                assert_eq!(&f[..2], &[0x00, 10]);
                assert_eq!(f.len(), 12); // 12 bytes is the next valid FD length
                assert_eq!(&f[2..12], &data[..]);
            }
            other => panic!("{other:?}"),
        }
        // short messages keep the classic PCI even on an FD channel (cf. capture)
        match first_segment(&[0x3e, 0], &p) {
            FirstSegment::Single(f) => assert_eq!(f, vec![2, 0x3e, 0]),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn escape_first_frame_for_messages_over_4095() {
        let p = params(64, None, None);
        let data = vec![0x55; 5000];
        match first_segment(&data, &p) {
            FirstSegment::First(f, n) => {
                assert_eq!(&f[..6], &[0x10, 0x00, 0, 0, 0x13, 0x88]);
                assert_eq!(f.len(), 64);
                assert_eq!(n, 58);
            }
            other => panic!("{other:?}"),
        }
    }

    fn roundtrip(len: usize, tx_dl: usize, ext: Option<u8>, bs: u8) {
        let data: Vec<u8> = (0..len).map(|i| (i * 7 + 3) as u8).collect();
        let p = params(tx_dl, Some(0xcc), ext);
        let mut rx = Reassembler::new(8192);
        let strip = |f: &[u8]| -> Vec<u8> { f[ext.is_some() as usize..].to_vec() };
        let mut got = None;
        match first_segment(&data, &p) {
            FirstSegment::Single(f) => {
                if let RxEvent::Message(m) = rx.feed(&strip(&f), bs) {
                    got = Some(m);
                }
            }
            FirstSegment::First(f, n) => {
                assert!(matches!(rx.feed(&strip(&f), bs), RxEvent::Started { .. }));
                let mut off = n;
                let mut sn = 1u8;
                while off < data.len() {
                    let (cf, took) = consecutive_frame(&data[off..], sn, &p);
                    off += took;
                    sn = (sn + 1) & 0xf;
                    match rx.feed(&strip(&cf), bs) {
                        RxEvent::Message(m) => got = Some(m),
                        RxEvent::Abort(e) => panic!("abort: {e}"),
                        _ => {}
                    }
                }
            }
        }
        assert_eq!(got.as_deref(), Some(&data[..]), "len={len} dl={tx_dl}");
    }

    #[test]
    fn segmentation_roundtrips_for_many_sizes_and_modes() {
        for &dl in &[8usize, 12, 16, 32, 64] {
            for &ext in &[None, Some(0xf1u8)] {
                for &len in &[
                    1usize, 5, 6, 7, 8, 9, 20, 61, 62, 63, 64, 100, 4095, 4096, 5000,
                ] {
                    for &bs in &[0u8, 3] {
                        roundtrip(len, dl, ext, bs);
                    }
                }
            }
        }
    }

    #[test]
    fn receiver_rejects_bad_sequence_and_overflow() {
        let mut rx = Reassembler::new(100);
        assert_eq!(
            rx.feed(&[0x10, 0x20, 1, 2, 3, 4, 5, 6], 0),
            RxEvent::Started { total: 32 }
        );
        assert_eq!(
            rx.feed(&[0x22, 1, 2, 3, 4, 5, 6, 7], 0),
            RxEvent::Abort("wrong sequence number")
        );
        assert!(!rx.is_active());
        assert_eq!(
            rx.feed(&[0x1f, 0xff, 0, 0, 0, 0, 0, 0], 0),
            RxEvent::SendFlowControl(FcStatus::Overflow)
        );
        // CF without a first frame is ignored
        assert_eq!(rx.feed(&[0x21, 0, 0, 0, 0, 0, 0, 0], 0), RxEvent::Ignore);
    }

    #[test]
    fn block_size_triggers_flow_control() {
        let mut rx = Reassembler::new(4095);
        assert!(matches!(
            rx.feed(&[0x10, 30, 1, 2, 3, 4, 5, 6], 2),
            RxEvent::Started { total: 30 }
        ));
        assert_eq!(rx.feed(&[0x21, 1, 2, 3, 4, 5, 6, 7], 2), RxEvent::Ignore);
        assert_eq!(
            rx.feed(&[0x22, 1, 2, 3, 4, 5, 6, 7], 2),
            RxEvent::SendFlowControl(FcStatus::ContinueToSend)
        );
    }

    #[test]
    fn stmin_and_flow_control_codec() {
        assert_eq!(decode_stmin(0), Duration::ZERO);
        assert_eq!(decode_stmin(20), Duration::from_millis(20));
        assert_eq!(decode_stmin(0xf3), Duration::from_micros(300));
        assert_eq!(decode_stmin(0x80), Duration::from_millis(127));
        let fc = flow_control_frame(FcStatus::ContinueToSend, 8, 2, &params(8, Some(0xaa), None));
        assert_eq!(fc, vec![0x30, 8, 2, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa]);
        assert_eq!(
            parse_pci(&fc),
            Ok(Pci::FlowControl {
                status: FcStatus::ContinueToSend,
                bs: 8,
                stmin: 2
            })
        );
        let fc = flow_control_frame(FcStatus::Overflow, 0, 0, &params(64, None, Some(0x12)));
        assert_eq!(fc, vec![0x12, 0x32, 0, 0]);
    }

    #[test]
    fn parse_rejects_malformed() {
        assert!(parse_pci(&[]).is_err());
        assert!(parse_pci(&[0x07, 1, 2]).is_err()); // SF length beyond frame
        assert!(parse_pci(&[0x00, 5, 1, 2, 3, 4, 5, 6]).is_err()); // escape in <=8 byte frame
        assert!(parse_pci(&[0x31, 0]).is_err());
        assert!(parse_pci(&[0x35, 0, 0]).is_err());
        assert!(parse_pci(&[0x45]).is_err());
    }
}
