//! End-to-end tests of bus + channel + ISO-TP against a simulated adapter with a fake ECU.
//! No hardware is touched: the ECU is a small ISO 15765-2 peer built from the same codec.

use gs_usb_j2534::bus::Bus;
use gs_usb_j2534::channel::{Channel, Config, RxMsg};
use gs_usb_j2534::gsusb::{encode_tx, CanAdapter, CanFrame, StartConfig, UsbError, ECHO_NONE};
use gs_usb_j2534::isotp::{self, FcStatus, FirstSegment, Pci, Reassembler, RxEvent, TxParams};
use gs_usb_j2534::j2534::*;
use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

const REQ: u32 = 0x18DA10F1;
const RESP: u32 = 0x18DAF110;

// ---- fake ECU ----------------------------------------------------------------------

struct TxState {
    data: Vec<u8>,
    offset: usize,
    sn: u8,
}

type Reply = Box<dyn Fn(&[u8]) -> Vec<u8> + Send>;

struct Ecu {
    fd: bool,
    tx_dl: usize,
    ext_addr: Option<u8>,
    pad: Option<u8>,
    bs: u8,
    stmin: u8,
    fc_status: FcStatus,
    silent_on_first_frame: bool,
    reply: Reply,
    rx: Reassembler,
    received: Vec<Vec<u8>>,
    received_frames: Vec<CanFrame>,
    tx: Option<TxState>,
    fc_sent: usize,
    cf_times: Vec<Instant>,
    fcs_from_tester: Vec<(FcStatus, u8, u8)>,
}

impl Ecu {
    fn new(fd: bool) -> Self {
        Ecu {
            fd,
            tx_dl: 8,
            ext_addr: None,
            pad: Some(0xAA),
            bs: 0,
            stmin: 0,
            fc_status: FcStatus::ContinueToSend,
            silent_on_first_frame: false,
            reply: Box::new(|m| {
                if m == [0x3E, 0x00] {
                    vec![0x7E, 0x00]
                } else {
                    Vec::new()
                }
            }),
            rx: Reassembler::new(8192),
            received: Vec::new(),
            received_frames: Vec::new(),
            tx: None,
            fc_sent: 0,
            cf_times: Vec::new(),
            fcs_from_tester: Vec::new(),
        }
    }

    fn params(&self) -> TxParams {
        TxParams {
            tx_dl: self.tx_dl,
            pad: self.pad,
            fd: self.fd && self.tx_dl > 8,
            ext_addr: self.ext_addr,
        }
    }

    fn frame(&self, data: Vec<u8>) -> CanFrame {
        CanFrame {
            id: RESP,
            ext: true,
            fd: self.fd,
            brs: false,
            esi: false,
            data,
        }
    }

    fn on_frame(&mut self, f: &CanFrame) -> Vec<CanFrame> {
        if f.id != REQ {
            return Vec::new();
        }
        self.received_frames.push(f.clone());
        let addr = self.ext_addr.is_some() as usize;
        let payload = &f.data[addr..];
        if let Ok(Pci::FlowControl { status, bs, stmin }) = isotp::parse_pci(payload) {
            self.fcs_from_tester.push((status, bs, stmin));
            return self.continue_tx(bs);
        }
        match self.rx.feed(payload, self.bs) {
            RxEvent::Message(m) => self.respond(m),
            RxEvent::Started { .. } => {
                if self.silent_on_first_frame {
                    return Vec::new();
                }
                self.fc_sent += 1;
                vec![self.fc(self.fc_status)]
            }
            RxEvent::SendFlowControl(s) => {
                self.fc_sent += 1;
                vec![self.fc(s)]
            }
            _ => Vec::new(),
        }
    }

    fn fc(&self, status: FcStatus) -> CanFrame {
        let data = isotp::flow_control_frame(status, self.bs, self.stmin, &self.params());
        self.frame(data)
    }

    fn respond(&mut self, m: Vec<u8>) -> Vec<CanFrame> {
        self.received.push(m.clone());
        let reply = (self.reply)(&m);
        if reply.is_empty() {
            return Vec::new();
        }
        match isotp::first_segment(&reply, &self.params()) {
            FirstSegment::Single(p) => vec![self.frame(p)],
            FirstSegment::First(p, off) => {
                self.tx = Some(TxState {
                    data: reply,
                    offset: off,
                    sn: 1,
                });
                vec![self.frame(p)]
            }
        }
    }

    fn continue_tx(&mut self, bs: u8) -> Vec<CanFrame> {
        let params = self.params();
        let mut out = Vec::new();
        let Some(mut st) = self.tx.take() else {
            return out;
        };
        let mut sent = 0u32;
        while st.offset < st.data.len() {
            let (cf, took) = isotp::consecutive_frame(&st.data[st.offset..], st.sn, &params);
            st.offset += took;
            st.sn = (st.sn + 1) & 0xf;
            out.push(self.frame(cf));
            sent += 1;
            if bs != 0 && sent >= bs as u32 {
                break;
            }
        }
        if st.offset < st.data.len() {
            self.tx = Some(st);
        }
        out
    }
}

// ---- simulated adapter ----------------------------------------------------------------

struct Mock {
    stream: Mutex<VecDeque<u8>>,
    cv: Condvar,
    ecu: Arc<Mutex<Ecu>>,
    sent: Mutex<Vec<(Instant, CanFrame)>>,
    /// When set, the echo of every first frame of a pair is delivered after the second one.
    swap_echoes: std::sync::atomic::AtomicBool,
    held: Mutex<Option<(u32, CanFrame)>>,
    /// While set, echoes are parked: the frames stay "in flight" and the hardware slots fill up.
    hold: std::sync::atomic::AtomicBool,
    parked: Mutex<Vec<(u32, CanFrame)>>,
}

impl Mock {
    fn push(&self, echo: u32, f: &CanFrame) {
        let bytes = encode_tx(echo, f).unwrap();
        self.stream.lock().unwrap().extend(bytes);
        self.cv.notify_all();
    }
}

impl Mock {
    /// Deliver the parked echoes and stop parking.
    fn release(&self) {
        self.hold.store(false, std::sync::atomic::Ordering::Relaxed);
        let parked: Vec<_> = self.parked.lock().unwrap().drain(..).collect();
        for (echo, f) in parked {
            self.push(echo, &f);
        }
    }
}

impl CanAdapter for Mock {
    fn start(&self, _: &StartConfig) -> Result<(), UsbError> {
        Ok(())
    }
    fn stop(&self) -> Result<(), UsbError> {
        Ok(())
    }
    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, UsbError> {
        let mut s = self.stream.lock().unwrap();
        if s.is_empty() {
            s = self.cv.wait_timeout(s, timeout).unwrap().0;
        }
        let n = buf.len().min(s.len());
        for (i, b) in s.drain(..n).enumerate() {
            buf[i] = b;
        }
        Ok(n)
    }
    fn write_frame(&self, echo_id: u32, frame: &CanFrame) -> Result<(), UsbError> {
        self.sent
            .lock()
            .unwrap()
            .push((Instant::now(), frame.clone()));
        if self.hold.load(std::sync::atomic::Ordering::Relaxed) {
            self.parked.lock().unwrap().push((echo_id, frame.clone()));
            return Ok(());
        }
        let replies = self.ecu.lock().unwrap().on_frame(frame);
        if self.swap_echoes.load(std::sync::atomic::Ordering::Relaxed) {
            let mut held = self.held.lock().unwrap();
            match held.take() {
                Some((id0, f0)) => {
                    self.push(echo_id, frame);
                    self.push(id0, &f0);
                }
                None => *held = Some((echo_id, frame.clone())),
            }
        } else {
            self.push(echo_id, frame); // the adapter echoes every transmitted frame
        }
        for r in replies {
            if r.id == RESP {
                self.ecu.lock().unwrap().cf_times.push(Instant::now());
            }
            self.push(ECHO_NONE, &r);
        }
        Ok(())
    }
    fn has_hw_timestamp(&self) -> bool {
        false
    }
}

struct Rig {
    mock: Arc<Mock>,
    ch: Arc<Channel>,
    ecu: Arc<Mutex<Ecu>>,
    bus: Arc<Bus>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.bus.remove_sink(self.ch.id);
        self.ch.shutdown();
        self.bus.shutdown();
    }
}

fn msg(protocol: u32, tx_flags: u32, data: &[u8]) -> Box<PassThruMsg> {
    let mut m: Box<PassThruMsg> = unsafe { Box::new(std::mem::zeroed()) };
    m.protocol_id = protocol;
    m.tx_flags = tx_flags;
    m.data_size = data.len() as u32;
    m.data[..data.len()].copy_from_slice(data);
    m
}

fn rig(protocol: u32, ecu: Ecu, cfg: impl FnOnce(&mut Config)) -> Rig {
    let ecu = Arc::new(Mutex::new(ecu));
    let mock = Arc::new(Mock {
        stream: Mutex::new(VecDeque::new()),
        cv: Condvar::new(),
        ecu: Arc::clone(&ecu),
        sent: Mutex::new(Vec::new()),
        swap_echoes: std::sync::atomic::AtomicBool::new(false),
        held: Mutex::new(None),
        hold: std::sync::atomic::AtomicBool::new(false),
        parked: Mutex::new(Vec::new()),
    });
    let bus = Bus::start(
        mock.clone(),
        StartConfig {
            nominal: 500_000,
            data: is_fd_protocol(protocol).then_some(2_000_000),
            listen_only: false,
        },
    )
    .unwrap();
    let mut c = Config::default();
    cfg(&mut c);
    let ch = Channel::new(1, protocol, CAN_ID_BOTH, bus.clone(), c);
    bus.add_sink(1, ch.clone());
    Rig { mock, ch, ecu, bus }
}

fn flow_filter(r: &Rig, protocol: u32, flags: u32, ext_addr: Option<u8>) {
    let id_flags = flags & CAN_29BIT_ID;
    let addr_flag = if ext_addr.is_some() {
        ISO15765_ADDR_TYPE
    } else {
        0
    };
    let mut pattern = RESP.to_be_bytes().to_vec();
    let mut flow = REQ.to_be_bytes().to_vec();
    let mut mask = vec![0xFF; 4];
    if let Some(a) = ext_addr {
        pattern.push(a);
        flow.push(a);
        mask.push(0xFF);
    }
    r.ch.start_filter(
        FLOW_CONTROL_FILTER,
        &msg(protocol, id_flags, &mask),
        &msg(protocol, id_flags, &pattern),
        Some(&msg(protocol, flags | addr_flag, &flow)),
    )
    .unwrap();
}

fn req_msg(protocol: u32, flags: u32, ext_addr: Option<u8>, user: &[u8]) -> Box<PassThruMsg> {
    let mut d = REQ.to_be_bytes().to_vec();
    d.extend(ext_addr);
    d.extend_from_slice(user);
    msg(
        protocol,
        flags
            | if ext_addr.is_some() {
                ISO15765_ADDR_TYPE
            } else {
                0
            },
        &d,
    )
}

fn collect(ch: &Channel, want: usize, wait: Duration) -> Vec<RxMsg> {
    let deadline = Instant::now() + wait;
    let mut out = Vec::new();
    while out.len() < want && Instant::now() < deadline {
        out.extend(ch.read(want - out.len(), Duration::from_millis(50)));
    }
    out
}

const FD_PAD_EXT: u32 = CAN_29BIT_ID | ISO15765_FRAME_PAD | FD_CAN_FORMAT;

// ---- ISO 15765 tests -------------------------------------------------------------------

#[test]
fn tester_present_on_fd_iso_channel_matches_the_bus_capture() {
    let r = rig(FD_ISO15765_PS, Ecu::new(true), |c| c.pad_value = 0xAA);
    flow_filter(&r, FD_ISO15765_PS, CAN_29BIT_ID | ISO15765_FRAME_PAD, None);
    let (n, e) = r.ch.write(
        &[&*req_msg(FD_ISO15765_PS, FD_PAD_EXT, None, &[0x3E, 0x00])],
        Duration::from_secs(1),
    );
    assert!(e.is_none(), "{e:?}");
    assert_eq!(n, 1);
    // exactly the frame seen on the real bus: FD, extended id, 02 3E 00 AA AA AA AA AA
    let sent = r.mock.sent.lock().unwrap();
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0].1.id, REQ);
    assert!(sent[0].1.ext && sent[0].1.fd);
    assert_eq!(
        sent[0].1.data,
        vec![0x02, 0x3E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]
    );
    drop(sent);
    let got = collect(&r.ch, 1, Duration::from_secs(1));
    assert_eq!(got.len(), 1);
    assert_eq!(&got[0].data[..4], &RESP.to_be_bytes());
    assert_eq!(&got[0].data[4..], &[0x7E, 0x00]);
    assert_eq!(
        got[0].rx_status & (CAN_29BIT_ID | FD_CAN_FORMAT),
        CAN_29BIT_ID | FD_CAN_FORMAT
    );
}

#[test]
fn write_without_flow_control_filter_is_rejected() {
    let r = rig(ISO15765, Ecu::new(false), |_| {});
    let (n, e) = r.ch.write(
        &[&*req_msg(ISO15765, CAN_29BIT_ID, None, &[0x3E, 0x00])],
        Duration::from_millis(200),
    );
    assert_eq!(n, 0);
    assert_eq!(e.unwrap().0, ERR_NO_FLOW_CONTROL);
}

#[test]
fn multi_frame_response_is_reassembled_and_flow_control_is_sent() {
    let mut ecu = Ecu::new(false);
    let vin: Vec<u8> = b"TESTVIN1234567890".to_vec();
    let reply = [vec![0x62, 0xF1, 0x90], vin.clone()].concat();
    ecu.reply = Box::new(move |m| {
        if m == [0x22, 0xF1, 0x90] {
            reply.clone()
        } else {
            vec![]
        }
    });
    let r = rig(ISO15765, ecu, |c| {
        c.pad_value = 0xAA;
        c.bs = 0;
        c.stmin = 0;
    });
    flow_filter(&r, ISO15765, CAN_29BIT_ID | ISO15765_FRAME_PAD, None);
    let (_, e) = r.ch.write(
        &[&*req_msg(
            ISO15765,
            CAN_29BIT_ID | ISO15765_FRAME_PAD,
            None,
            &[0x22, 0xF1, 0x90],
        )],
        Duration::from_secs(1),
    );
    assert!(e.is_none(), "{e:?}");
    let got = collect(&r.ch, 2, Duration::from_secs(2));
    assert_eq!(
        got.len(),
        2,
        "expected start indication + message, got {got:?}"
    );
    assert_eq!(got[0].rx_status & START_OF_MESSAGE, START_OF_MESSAGE);
    assert_eq!(got[0].data, RESP.to_be_bytes());
    assert_eq!(
        &got[1].data[4..],
        &[vec![0x62, 0xF1, 0x90], vin].concat()[..]
    );
    // the driver answered the first frame with a padded CTS flow control frame
    let sent = r.mock.sent.lock().unwrap();
    let fc = sent
        .iter()
        .find(|(_, f)| f.data[0] == 0x30)
        .expect("flow control sent");
    assert_eq!(fc.1.id, REQ);
    assert_eq!(
        fc.1.data,
        vec![0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]
    );
}

#[test]
fn multi_frame_transmit_honours_block_size_and_stmin() {
    let mut ecu = Ecu::new(false);
    ecu.bs = 3;
    ecu.stmin = 2; // 2 ms
    let r = rig(ISO15765, ecu, |_| {});
    flow_filter(&r, ISO15765, CAN_29BIT_ID | ISO15765_FRAME_PAD, None);
    let payload: Vec<u8> = (0..100).map(|i| (i * 3 + 1) as u8).collect();
    let (n, e) = r.ch.write(
        &[&*req_msg(
            ISO15765,
            CAN_29BIT_ID | ISO15765_FRAME_PAD,
            None,
            &payload,
        )],
        Duration::from_secs(5),
    );
    assert!(e.is_none(), "{e:?}");
    assert_eq!(n, 1);
    let ecu = r.ecu.lock().unwrap();
    assert_eq!(ecu.received, vec![payload]);
    // FF + 14 CFs; the ECU needs a flow control after the FF and after each full block of 3
    assert_eq!(ecu.fc_sent, 5);
    drop(ecu);
    let sent = r.mock.sent.lock().unwrap();
    let cfs: Vec<Instant> = sent
        .iter()
        .filter(|(_, f)| f.data[0] >> 4 == 2)
        .map(|(t, _)| *t)
        .collect();
    assert_eq!(cfs.len(), 14);
    for w in cfs.windows(2) {
        let gap = w[1] - w[0];
        assert!(
            gap >= Duration::from_micros(1700),
            "STmin violated: {gap:?}"
        );
    }
    // sequence numbers wrap 1..F,0..
    let sns: Vec<u8> = sent
        .iter()
        .filter(|(_, f)| f.data[0] >> 4 == 2)
        .map(|(_, f)| f.data[0] & 0xF)
        .collect();
    assert_eq!(sns, vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]);
}

#[test]
fn fd_channel_sends_64_byte_frames_when_tx_data_length_is_raised() {
    let mut ecu = Ecu::new(true);
    ecu.tx_dl = 64;
    let r = rig(FD_ISO15765_PS, ecu, |c| c.tx_dl = 64);
    flow_filter(&r, FD_ISO15765_PS, CAN_29BIT_ID | ISO15765_FRAME_PAD, None);
    let payload: Vec<u8> = (0..300).map(|i| i as u8).collect();
    let (_, e) = r.ch.write(
        &[&*req_msg(FD_ISO15765_PS, FD_PAD_EXT, None, &payload)],
        Duration::from_secs(5),
    );
    assert!(e.is_none(), "{e:?}");
    assert_eq!(r.ecu.lock().unwrap().received, vec![payload]);
    let sent = r.mock.sent.lock().unwrap();
    assert_eq!(sent[0].1.data.len(), 64); // first frame fills the FD frame
    assert!(sent.iter().all(|(_, f)| f.fd));
}

#[test]
fn missing_flow_control_times_out_with_n_bs() {
    let mut ecu = Ecu::new(false);
    ecu.silent_on_first_frame = true;
    let r = rig(ISO15765, ecu, |c| c.n_bs = 120);
    flow_filter(&r, ISO15765, CAN_29BIT_ID, None);
    let t0 = Instant::now();
    let (n, e) = r.ch.write(
        &[&*req_msg(ISO15765, CAN_29BIT_ID, None, &[0u8; 40])],
        Duration::from_secs(3),
    );
    assert_eq!(n, 0);
    assert_eq!(e.unwrap().0, ERR_TIMEOUT);
    assert!(t0.elapsed() < Duration::from_secs(2));
}

#[test]
fn receiver_overflow_flow_control_aborts_transmission() {
    let mut ecu = Ecu::new(false);
    ecu.fc_status = FcStatus::Overflow;
    let r = rig(ISO15765, ecu, |_| {});
    flow_filter(&r, ISO15765, CAN_29BIT_ID, None);
    let (n, e) = r.ch.write(
        &[&*req_msg(ISO15765, CAN_29BIT_ID, None, &[0u8; 40])],
        Duration::from_secs(3),
    );
    assert_eq!(n, 0);
    assert_eq!(e.unwrap().0, ERR_BUFFER_FULL);
}

#[test]
fn extended_addressing_roundtrip() {
    let mut ecu = Ecu::new(false);
    ecu.ext_addr = Some(0xF1);
    ecu.reply = Box::new(|m| {
        if m == [0x3E, 0x00] {
            vec![0x7E, 0x00]
        } else {
            vec![]
        }
    });
    let r = rig(ISO15765, ecu, |_| {});
    flow_filter(&r, ISO15765, CAN_29BIT_ID, Some(0xF1));
    let (_, e) = r.ch.write(
        &[&*req_msg(ISO15765, CAN_29BIT_ID, Some(0xF1), &[0x3E, 0x00])],
        Duration::from_secs(1),
    );
    assert!(e.is_none(), "{e:?}");
    let got = collect(&r.ch, 1, Duration::from_secs(1));
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].rx_status & ISO15765_ADDR_TYPE, ISO15765_ADDR_TYPE);
    assert_eq!(&got[0].data[..5], &[0x18, 0xDA, 0xF1, 0x10, 0xF1]);
    assert_eq!(&got[0].data[5..], &[0x7E, 0x00]);
    assert_eq!(
        r.mock.sent.lock().unwrap()[0].1.data,
        vec![0xF1, 0x02, 0x3E, 0x00]
    );
}

#[test]
fn loopback_indication_is_generated_when_enabled() {
    let r = rig(ISO15765, Ecu::new(false), |c| c.loopback = true);
    flow_filter(&r, ISO15765, CAN_29BIT_ID, None);
    r.ch.write(
        &[&*req_msg(ISO15765, CAN_29BIT_ID, None, &[0x3E, 0x00])],
        Duration::from_secs(1),
    );
    let got = collect(&r.ch, 2, Duration::from_secs(1));
    assert_eq!(got.len(), 2);
    assert_eq!(got[0].rx_status & TX_MSG_TYPE, TX_MSG_TYPE);
    assert_eq!(got[0].data, REQ.to_be_bytes());
    assert_eq!(got[1].rx_status & TX_MSG_TYPE, 0);
}

// ---- raw CAN tests ----------------------------------------------------------------------

fn pass_all(r: &Rig, protocol: u32) {
    for f in [0, CAN_29BIT_ID] {
        r.ch.start_filter(
            PASS_FILTER,
            &msg(protocol, f, &[0; 4]),
            &msg(protocol, f, &[0; 4]),
            None,
        )
        .unwrap();
    }
}

#[test]
fn raw_channel_filters_loopback_and_fd_flags() {
    let r = rig(FD_CAN_PS, Ecu::new(true), |c| c.loopback = true);
    // without a pass filter nothing is received
    let req = {
        let mut d = REQ.to_be_bytes().to_vec();
        d.extend_from_slice(&[0x02, 0x3E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]);
        msg(FD_CAN_PS, CAN_29BIT_ID | FD_CAN_FORMAT, &d)
    };
    r.ch.write(&[&*req], Duration::from_secs(1));
    assert!(collect(&r.ch, 1, Duration::from_millis(200)).is_empty());

    pass_all(&r, FD_CAN_PS);
    r.ch.write(&[&*req], Duration::from_secs(1));
    let got = collect(&r.ch, 2, Duration::from_secs(1));
    assert_eq!(got.len(), 2, "{got:?}"); // loopback echo + ECU answer (raw, so no ISO-TP handling)
    assert_eq!(got[0].rx_status, TX_MSG_TYPE | CAN_29BIT_ID | FD_CAN_FORMAT);
    assert_eq!(&got[1].data[..4], &RESP.to_be_bytes());
    assert_eq!(
        &got[1].data[4..],
        &[0x02, 0x7E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]
    );

    // a block filter on the response id suppresses it
    r.ch.start_filter(
        BLOCK_FILTER,
        &msg(FD_CAN_PS, CAN_29BIT_ID, &[0xFF; 4]),
        &msg(FD_CAN_PS, CAN_29BIT_ID, &RESP.to_be_bytes()),
        None,
    )
    .unwrap();
    r.ch.write(&[&*req], Duration::from_secs(1));
    let got = collect(&r.ch, 2, Duration::from_millis(300));
    assert_eq!(got.len(), 1, "only the loopback echo remains: {got:?}");
}

#[test]
fn raw_channel_validates_messages() {
    let r = rig(CAN, Ecu::new(false), |_| {});
    let ok = msg(CAN, 0, &[0, 0, 7, 0xDF, 1, 2, 3]);
    assert!(r.ch.write(&[&*ok], Duration::from_secs(1)).1.is_none());
    let long = msg(CAN, 0, &[&[0, 0, 7, 0xDF][..], &[0u8; 9]].concat());
    assert_eq!(
        r.ch.write(&[&*long], Duration::ZERO).1.unwrap().0,
        ERR_INVALID_MSG
    );
    let wide = msg(CAN, 0, &[0, 0, 8, 0, 1]); // 0x800 without CAN_29BIT_ID
    assert_eq!(
        r.ch.write(&[&*wide], Duration::ZERO).1.unwrap().0,
        ERR_INVALID_MSG
    );
    let fd_on_classic = msg(CAN, FD_CAN_FORMAT, &[0, 0, 7, 0xDF, 1]);
    assert_eq!(
        r.ch.write(&[&*fd_on_classic], Duration::ZERO).1.unwrap().0,
        ERR_INVALID_MSG
    );
    let wrong_proto = msg(ISO15765, 0, &[0, 0, 7, 0xDF, 1]);
    assert_eq!(
        r.ch.write(&[&*wrong_proto], Duration::ZERO).1.unwrap().0,
        ERR_MSG_PROTOCOL_ID
    );
}

#[test]
fn periodic_messages_repeat_until_stopped() {
    let r = rig(CAN, Ecu::new(false), |_| {});
    let m = msg(CAN, 0, &[0, 0, 7, 0xDF, 0x01]);
    let id = r.ch.start_periodic(&m, 20).unwrap();
    std::thread::sleep(Duration::from_millis(250));
    r.ch.stop_periodic(id).unwrap();
    let n = r.mock.sent.lock().unwrap().len();
    assert!((8..=14).contains(&n), "expected ~12 transmissions, got {n}");
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(
        r.mock.sent.lock().unwrap().len(),
        n,
        "periodic message kept running after stop"
    );
    assert_eq!(
        r.ch.start_periodic(&m, 2).unwrap_err().0,
        ERR_INVALID_TIME_INTERVAL
    );
    assert_eq!(r.ch.stop_periodic(99).unwrap_err().0, ERR_INVALID_MSG_ID);
}

#[test]
fn bus_strips_tx_brs_unless_the_application_setting_honors_it() {
    let r = rig(FD_CAN_PS, Ecu::new(true), |_| {});
    pass_all(&r, FD_CAN_PS);
    let mut d = REQ.to_be_bytes().to_vec();
    d.extend_from_slice(&[0x03, 0x22, 0xF1, 0x90, 0x55, 0x55, 0x55, 0x55]);
    let with_brs = msg(FD_CAN_PS, CAN_29BIT_ID | FD_CAN_FORMAT | FD_CAN_BRS, &d);

    assert!(r
        .ch
        .write(&[&*with_brs], Duration::from_secs(1))
        .1
        .is_none());
    wait_sent(&r, 1);
    {
        let sent = r.mock.sent.lock().unwrap();
        assert!(
            sent[0].1.fd && !sent[0].1.brs,
            "BRS must be stripped by default"
        );
    }
    r.bus.set_honor_tx_brs(true);
    assert!(r
        .ch
        .write(&[&*with_brs], Duration::from_secs(1))
        .1
        .is_none());
    wait_sent(&r, 2);
    let sent = r.mock.sent.lock().unwrap();
    assert!(
        sent[1].1.fd && sent[1].1.brs,
        "BRS must be sent when honoring is enabled"
    );
}

#[test]
fn bus_statistics_count_echoes_and_flag_out_of_order_transmission() {
    let r = rig(FD_CAN_PS, Ecu::new(true), |_| {});
    let mut d = REQ.to_be_bytes().to_vec();
    d.extend_from_slice(&[1, 2, 3, 4]);
    let m = msg(FD_CAN_PS, CAN_29BIT_ID | FD_CAN_FORMAT, &d);

    // In-order adapter: every frame is echoed, nothing is flagged.
    for _ in 0..6 {
        assert!(r.ch.write(&[&*m], Duration::from_secs(1)).1.is_none());
    }
    std::thread::sleep(Duration::from_millis(100));
    let s = r.bus.stats();
    assert_eq!((s.tx, s.echoes, s.reordered), (6, 6, 0), "{s:?}");

    // An adapter that completes the second frame of a pair first must be noticed.
    r.mock
        .swap_echoes
        .store(true, std::sync::atomic::Ordering::Relaxed);
    assert!(r.ch.write(&[&*m, &*m], Duration::from_secs(1)).1.is_none());
    std::thread::sleep(Duration::from_millis(100));
    let s = r.bus.stats();
    assert_eq!((s.tx, s.echoes, s.reordered), (8, 8, 1), "{s:?}");
}

#[test]
fn error_frames_are_decoded_for_the_trace() {
    use gs_usb_j2534::bus::describe_error;
    // the frame seen during the failed flash block: protocol error, bit1, tec 8, rec 29
    let text = describe_error(0x8, &[0, 0, 0x10, 0, 0, 0, 8, 0x1D]);
    assert!(text.contains("protocol") && text.contains("bit1"), "{text}");
    assert!(text.contains("tec=8") && text.contains("rec=29"), "{text}");
    let text = describe_error(0x4, &[0, 0x30, 0, 0, 0, 0, 0, 0x7F]);
    assert!(
        text.contains("rx-passive") && text.contains("tx-passive"),
        "{text}"
    );
    assert!(describe_error(0x40, &[0; 8]).contains("BUS-OFF"));
}

/// Raw writes are queued and sent by a pump thread; wait until `n` frames reached the adapter.
fn wait_sent(r: &Rig, n: usize) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while r.mock.sent.lock().unwrap().len() < n && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn raw_burst_with_timeout_zero_is_queued_in_order_when_all_hardware_slots_are_busy() {
    // A flash tool writes a whole ISO-TP block (65 consecutive frames) back to back with timeout 0 and
    // does not retry a refused frame; the adapter only has ten TX slots.
    let r = rig(FD_CAN_PS, Ecu::new(true), |_| {});
    pass_all(&r, FD_CAN_PS);
    r.mock
        .hold
        .store(true, std::sync::atomic::Ordering::Relaxed);
    for i in 0..120u8 {
        let m = msg(FD_CAN_PS, FD_CAN_FORMAT, &[0, 0, 0x01, 0x23, i]);
        let (n, e) = r.ch.write(&[&*m], Duration::ZERO);
        assert!(
            (n, e.is_none()) == (1, true),
            "frame {i} was refused: {e:?}"
        );
    }
    // The simulated bus never delivers a frame, so the first transmission waits out the 250 ms
    // start-up synchronisation window before the ten slots fill.
    std::thread::sleep(Duration::from_millis(450));
    assert_eq!(
        r.mock.sent.lock().unwrap().len(),
        10,
        "only the ten slots may be in flight"
    );

    r.mock.release();
    wait_sent(&r, 120);
    let sent = r.mock.sent.lock().unwrap();
    assert_eq!(sent.len(), 120);
    let order: Vec<u8> = sent.iter().map(|(_, f)| f.data[0]).collect();
    assert_eq!(
        order,
        (0..120u8).collect::<Vec<_>>(),
        "frames must leave in submission order"
    );
}

#[test]
fn clear_tx_buffer_drops_frames_still_waiting_for_a_slot() {
    let r = rig(FD_CAN_PS, Ecu::new(true), |_| {});
    r.mock
        .hold
        .store(true, std::sync::atomic::Ordering::Relaxed);
    for i in 0..30u8 {
        let m = msg(FD_CAN_PS, FD_CAN_FORMAT, &[0, 0, 0x01, 0x23, i]);
        assert!(r.ch.write(&[&*m], Duration::ZERO).1.is_none());
    }
    std::thread::sleep(Duration::from_millis(100));
    r.ch.clear_tx();
    r.mock.release();
    std::thread::sleep(Duration::from_millis(200));
    assert!(
        r.mock.sent.lock().unwrap().len() <= 11,
        "queued frames must not be sent after CLEAR_TX_BUFFER"
    );
}
