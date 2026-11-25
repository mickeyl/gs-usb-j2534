//! A started CAN controller: reader thread, TX slot/echo accounting and fan-out of
//! received frames to every connected J2534 channel.
//!
//! Locking rules: sinks are cloned out of their lock before being called, and sink
//! callbacks only ever take `slots` through [`Bus::transmit`] with a zero timeout, so
//! the reader thread can never block on (or deadlock with) a transmitting thread.

use crate::gsusb::{CanAdapter, CanFrame, RxPacket, StartConfig, StreamDecoder, UsbError};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// Outstanding USB transmissions (echo ids 0..SLOTS). Matches the candleLight firmware
/// pool that the j2534-bridge backend also assumes.
const SLOTS: usize = 10;
/// Frames accepted by `PassThruWriteMsgs` while all hardware slots are busy wait here. A real
/// PassThru device has a deep transmit queue, and applications such as flash tools write a whole
/// ISO-TP block back to back with timeout 0 and do not retry a refused frame.
const TX_QUEUE_LIMIT: usize = 4096;
/// A frame without a TX echo for this long counts as failed (no ACK / bus problem).
const TX_ECHO_TIMEOUT: Duration = Duration::from_millis(1000);

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// Completion handle for one transmitted frame.
pub struct TxTicket {
    state: Mutex<Option<Option<u32>>>, // None pending, Some(None) failed, Some(Some(ts)) sent
    cv: Condvar,
}

impl TxTicket {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(None),
            cv: Condvar::new(),
        })
    }

    fn complete(&self, result: Option<u32>) {
        *lock(&self.state) = Some(result);
        self.cv.notify_all();
    }

    /// `None` on timeout, otherwise `Some(timestamp)` on success or `Some(None)` on failure.
    pub fn wait(&self, timeout: Duration) -> Option<Option<u32>> {
        let deadline = Instant::now() + timeout;
        let mut state = lock(&self.state);
        loop {
            if let Some(result) = *state {
                return Some(result);
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            state = self
                .cv
                .wait_timeout(state, deadline - now)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
    }
}

/// Message-level loopback indication generated in the reader thread when the frame that
/// completes an ISO-TP message leaves the controller. Producing it there (not in the
/// transmitting thread) keeps request indications causally before the ECU response.
#[derive(Clone, Debug)]
pub struct Indication {
    pub rx_status: u32,
    pub data: Vec<u8>,
}

/// Bookkeeping for a frame that is in flight.
pub struct TxMeta {
    /// Channel that submitted the frame (0 = internal, e.g. ISO-TP flow control).
    pub owner: u32,
    pub frame: CanFrame,
    pub ticket: Option<Arc<TxTicket>>,
    pub indication: Option<Indication>,
    /// Submission order, used to verify that the adapter transmits frames in that order.
    pub seq: u64,
}

pub trait Sink: Send + Sync {
    /// Received frame (not an echo). `ts` is the adapter timestamp in microseconds.
    fn on_frame(&self, frame: &CanFrame, ts: u32, overflow: bool);
    /// A frame submitted through [`Bus::transmit`] left the controller (`ts`) or failed.
    fn on_tx_done(&self, meta: &TxMeta, ts: Option<u32>);
    fn on_tick(&self, now: Instant);
}

#[derive(Debug)]
pub enum TxError {
    /// No free TX slot within the allowed time.
    Full,
    ListenOnly,
    Usb(String),
}

struct Queued {
    owner: u32,
    frame: CanFrame,
}

struct Slots {
    entries: [Option<(Instant, TxMeta)>; SLOTS],
}

/// Counters for diagnostics (also written to the trace when the bus shuts down).
#[derive(Default)]
struct Counters {
    tx: AtomicU64,
    echoes: AtomicU64,
    reordered: AtomicU64,
    error_frames: AtomicU64,
    bus_offs: AtomicU64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BusStats {
    pub tx: u64,
    pub echoes: u64,
    /// Echoes that arrived while an earlier submitted frame was still outstanding.
    pub reordered: u64,
    pub error_frames: u64,
    pub bus_offs: u64,
}

/// Human-readable decoding of a gs_usb (SocketCAN style) error frame.
pub fn describe_error(id: u32, d: &[u8]) -> String {
    let at = |i: usize| d.get(i).copied().unwrap_or(0);
    let mut parts: Vec<String> = Vec::new();
    for (bit, name) in [
        (0x1, "tx-timeout"),
        (0x2, "lost-arbitration"),
        (0x4, "controller"),
        (0x8, "protocol"),
        (0x10, "transceiver"),
        (0x20, "no-ack"),
        (0x40, "BUS-OFF"),
        (0x80, "bus-error"),
    ] {
        if id & bit != 0 {
            parts.push(name.to_string());
        }
    }
    let flags = |value: u8, names: &[(u8, &str)]| -> String {
        names
            .iter()
            .filter(|(b, _)| value & b != 0)
            .map(|(_, n)| *n)
            .collect::<Vec<_>>()
            .join("|")
    };
    if id & 0x4 != 0 {
        parts.push(format!(
            "[{}]",
            flags(
                at(1),
                &[
                    (0x1, "rx-overflow"),
                    (0x2, "tx-overflow"),
                    (0x4, "rx-warning"),
                    (0x8, "tx-warning"),
                    (0x10, "rx-passive"),
                    (0x20, "tx-passive"),
                    (0x40, "active")
                ]
            )
        ));
    }
    if id & 0x8 != 0 {
        parts.push(format!(
            "[{}]",
            flags(
                at(2),
                &[
                    (0x1, "bit"),
                    (0x2, "form"),
                    (0x4, "stuff"),
                    (0x8, "bit0"),
                    (0x10, "bit1"),
                    (0x20, "overload"),
                    (0x40, "active-error"),
                    (0x80, "while-transmitting")
                ]
            )
        ));
    }
    format!("{} tec={} rec={}", parts.join(" "), at(6), at(7))
}

pub struct Bus {
    usb: Arc<dyn CanAdapter>,
    cfg: Mutex<StartConfig>,
    slots: Mutex<Slots>,
    slot_cv: Condvar,
    sinks: Mutex<Vec<(u32, Arc<dyn Sink>)>>,
    generation: AtomicU64,
    threads: Mutex<Vec<JoinHandle<()>>>,
    broken: AtomicBool,
    last_error: Mutex<Option<String>>,
    recoveries: Mutex<Vec<Instant>>,
    /// Set by the reader on the first valid frame after a (re)start: the controller has
    /// integrated into the bus. Transmitting earlier tends to fail with bit errors.
    synced: AtomicBool,
    started_at: Mutex<Instant>,
    /// Send FD frames at the nominal rate even if the application asks for BRS (see
    /// `settings::honor_tx_brs`).
    strip_tx_brs: AtomicBool,
    /// Serialises USB writes against a controller restart after bus-off.
    io: Mutex<()>,
    seq: AtomicU64,
    counters: Counters,
    txq: Mutex<VecDeque<Queued>>,
    txq_cv: Condvar,
    txq_space: Condvar,
    epoch: Instant,
    /// Serialises start/stop/restart against each other.
    lifecycle: Mutex<()>,
}

impl Bus {
    pub fn start(usb: Arc<dyn CanAdapter>, cfg: StartConfig) -> Result<Arc<Self>, UsbError> {
        let bus = Arc::new(Self {
            usb,
            cfg: Mutex::new(cfg),
            slots: Mutex::new(Slots {
                entries: Default::default(),
            }),
            slot_cv: Condvar::new(),
            sinks: Mutex::new(Vec::new()),
            generation: AtomicU64::new(0),
            threads: Mutex::new(Vec::new()),
            broken: AtomicBool::new(false),
            last_error: Mutex::new(None),
            recoveries: Mutex::new(Vec::new()),
            synced: AtomicBool::new(false),
            started_at: Mutex::new(Instant::now()),
            strip_tx_brs: AtomicBool::new(!crate::settings::honor_tx_brs()),
            io: Mutex::new(()),
            seq: AtomicU64::new(0),
            counters: Counters::default(),
            txq: Mutex::new(VecDeque::new()),
            txq_cv: Condvar::new(),
            txq_space: Condvar::new(),
            epoch: Instant::now(),
            lifecycle: Mutex::new(()),
        });
        {
            let _g = lock(&bus.lifecycle);
            bus.usb.start(&cfg)?;
            bus.spawn_threads();
        }
        Ok(bus)
    }

    /// Choose whether BRS requested by the application is transmitted (default: stripped).
    pub fn set_honor_tx_brs(&self, honor: bool) {
        self.strip_tx_brs.store(!honor, Ordering::Relaxed);
    }

    pub fn stats(&self) -> BusStats {
        let c = &self.counters;
        BusStats {
            tx: c.tx.load(Ordering::Relaxed),
            echoes: c.echoes.load(Ordering::Relaxed),
            reordered: c.reordered.load(Ordering::Relaxed),
            error_frames: c.error_frames.load(Ordering::Relaxed),
            bus_offs: c.bus_offs.load(Ordering::Relaxed),
        }
    }

    pub fn config(&self) -> StartConfig {
        *lock(&self.cfg)
    }

    pub fn is_broken(&self) -> Option<String> {
        self.broken
            .load(Ordering::Relaxed)
            .then(|| lock(&self.last_error).clone().unwrap_or_default())
    }

    pub fn add_sink(&self, id: u32, sink: Arc<dyn Sink>) {
        lock(&self.sinks).push((id, sink));
    }

    pub fn remove_sink(&self, id: u32) {
        lock(&self.sinks).retain(|(i, _)| *i != id);
    }

    fn spawn_threads(self: &Arc<Self>) {
        let generation = self.generation.fetch_add(1, Ordering::SeqCst) + 1;
        self.broken.store(false, Ordering::Relaxed);
        self.mark_started();
        let reader = {
            let bus = Arc::clone(self);
            std::thread::Builder::new()
                .name("gsusb-rx".into())
                .spawn(move || bus.reader_loop(generation))
                .expect("spawn reader")
        };
        let ticker = {
            let bus = Arc::clone(self);
            std::thread::Builder::new()
                .name("gsusb-tick".into())
                .spawn(move || bus.ticker_loop(generation))
                .expect("spawn ticker")
        };
        let pump = {
            let bus = Arc::clone(self);
            std::thread::Builder::new()
                .name("gsusb-tx".into())
                .spawn(move || bus.pump_loop(generation))
                .expect("spawn tx pump")
        };
        let mut threads = lock(&self.threads);
        threads.push(reader);
        threads.push(ticker);
        threads.push(pump);
    }

    fn stop_threads(&self) {
        self.generation.fetch_add(1, Ordering::SeqCst);
        let handles: Vec<_> = std::mem::take(&mut *lock(&self.threads));
        for h in handles {
            let _ = h.join();
        }
        self.fail_all_slots();
        lock(&self.txq).clear();
        self.txq_space.notify_all();
    }

    /// Reconfigure bit rates: briefly resets the controller. In-flight frames are failed.
    pub fn restart(self: &Arc<Self>, cfg: StartConfig) -> Result<(), UsbError> {
        let _g = lock(&self.lifecycle);
        self.stop_threads();
        *lock(&self.cfg) = cfg;
        let result = self.usb.start(&cfg);
        // Re-arm the reader even on failure so the bus object stays consistent.
        self.spawn_threads();
        result
    }

    pub fn shutdown(&self) {
        let _g = lock(&self.lifecycle);
        let s = self.stats();
        crate::trace!(
            "bus stats: tx={} echoes={} reordered={} error_frames={} bus_off={}",
            s.tx,
            s.echoes,
            s.reordered,
            s.error_frames,
            s.bus_offs
        );
        self.stop_threads();
        let _ = self.usb.stop();
    }

    /// Queue one frame for transmission. Waits up to `timeout` for a free TX slot.
    pub fn transmit(
        &self,
        owner: u32,
        frame: CanFrame,
        ticket: Option<Arc<TxTicket>>,
        timeout: Duration,
    ) -> Result<(), TxError> {
        self.transmit_with(owner, frame, ticket, None, timeout)
    }

    pub fn transmit_with(
        &self,
        owner: u32,
        frame: CanFrame,
        ticket: Option<Arc<TxTicket>>,
        indication: Option<Indication>,
        timeout: Duration,
    ) -> Result<(), TxError> {
        if self.config().listen_only {
            return Err(TxError::ListenOnly);
        }
        let frame = if self.strip_tx_brs.load(Ordering::Relaxed) && frame.brs {
            CanFrame {
                brs: false,
                ..frame
            }
        } else {
            frame
        };
        let deadline = Instant::now() + timeout;
        if !timeout.is_zero() {
            self.wait_for_bus_sync(deadline);
        }
        let mut slots = lock(&self.slots);
        let index = loop {
            if let Some(i) = slots.entries.iter().position(Option::is_none) {
                break i;
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(TxError::Full);
            }
            slots = self
                .slot_cv
                .wait_timeout(slots, deadline - now)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        };
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        self.counters.tx.fetch_add(1, Ordering::Relaxed);
        crate::trace!(
            "TX seq={seq} slot={index} id={:X}{} fd={} brs={} len={} data={:02X?}",
            frame.id,
            if frame.ext { "x" } else { "" },
            frame.fd,
            frame.brs,
            frame.data.len(),
            frame.data
        );
        // Reserve the slot before writing so the echo can never race ahead of us.
        let write_frame = frame.clone();
        slots.entries[index] = Some((
            Instant::now(),
            TxMeta {
                owner,
                frame,
                ticket,
                indication,
                seq,
            },
        ));
        // Release the lock for the USB write: the reader needs it to process echoes.
        drop(slots);
        let written = {
            let _io = lock(&self.io);
            self.usb.write_frame(index as u32, &write_frame)
        };
        match written {
            Ok(()) => Ok(()),
            Err(e) => {
                let mut slots = lock(&self.slots);
                if let Some((_, meta)) = slots.entries[index].take() {
                    if let Some(t) = meta.ticket {
                        t.complete(None);
                    }
                }
                self.slot_cv.notify_one();
                self.note_error(&e.to_string());
                Err(TxError::Usb(e.to_string()))
            }
        }
    }

    fn mark_started(&self) {
        self.synced.store(false, Ordering::Relaxed);
        *lock(&self.started_at) = Instant::now();
    }

    /// Give a freshly started controller up to 250 ms to see its first valid frame.
    fn wait_for_bus_sync(&self, deadline: Instant) {
        const MAX_WAIT: Duration = Duration::from_millis(250);
        let limit = (*lock(&self.started_at) + MAX_WAIT).min(deadline);
        while !self.synced.load(Ordering::Relaxed) && Instant::now() < limit {
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn note_error(&self, message: &str) {
        crate::trace!("bus error: {message}");
        *lock(&self.last_error) = Some(message.to_string());
    }

    fn fail_all_slots(&self) {
        let metas: Vec<TxMeta> = {
            let mut slots = lock(&self.slots);
            let v = slots
                .entries
                .iter_mut()
                .filter_map(|s| s.take().map(|(_, m)| m))
                .collect();
            self.slot_cv.notify_all();
            v
        };
        for meta in metas {
            self.finish_tx(meta, None);
        }
    }

    fn finish_tx(&self, meta: TxMeta, ts: Option<u32>) {
        if let Some(t) = &meta.ticket {
            t.complete(ts);
        }
        let sinks: Vec<_> = lock(&self.sinks)
            .iter()
            .map(|(_, s)| Arc::clone(s))
            .collect();
        for s in sinks {
            s.on_tx_done(&meta, ts);
        }
    }

    fn host_timestamp(&self) -> u32 {
        self.epoch.elapsed().as_micros() as u32
    }

    fn reader_loop(self: Arc<Self>, generation: u64) {
        let mut decoder = StreamDecoder::new(self.usb.has_hw_timestamp());
        let mut buf = vec![0u8; 8192];
        while self.generation.load(Ordering::SeqCst) == generation {
            match self.usb.read(&mut buf, Duration::from_millis(20)) {
                Ok(0) => {}
                Ok(n) => decoder.push(&buf[..n]),
                Err(e) => {
                    self.note_error(&format!("USB read failed: {e}"));
                    self.broken.store(true, Ordering::Relaxed);
                    return;
                }
            }
            loop {
                match decoder.next_packet() {
                    Ok(Some(p)) => {
                        if self.dispatch(p) {
                            // Controller was restarted: drop bytes of the old session.
                            decoder = StreamDecoder::new(self.usb.has_hw_timestamp());
                            break;
                        }
                    }
                    Ok(None) => break,
                    Err(e) => {
                        self.note_error(&format!("RX decode: {e}"));
                        break;
                    }
                }
            }
        }
    }

    /// Returns true when the controller had to be restarted.
    fn dispatch(&self, p: RxPacket) -> bool {
        let ts = p.hw_timestamp.unwrap_or_else(|| self.host_timestamp());
        if p.error {
            self.counters.error_frames.fetch_add(1, Ordering::Relaxed);
            if crate::log::enabled() {
                let (inflight, oldest) = {
                    let slots = lock(&self.slots);
                    let oldest = slots
                        .entries
                        .iter()
                        .flatten()
                        .min_by_key(|(_, m)| m.seq)
                        .map(|(_, m)| {
                            format!(
                                "seq={} id={:X} {:02X?}",
                                m.seq,
                                m.frame.id,
                                &m.frame.data[..m.frame.data.len().min(4)]
                            )
                        });
                    (slots.entries.iter().flatten().count(), oldest)
                };
                crate::trace!(
                    "RX error frame: {} | in flight {}{}",
                    describe_error(p.frame.id, &p.frame.data),
                    inflight,
                    oldest.map(|o| format!(", oldest {o}")).unwrap_or_default()
                );
            }
            // SocketCAN-style error class CAN_ERR_BUSOFF: the controller stopped itself.
            if p.frame.id & 0x40 != 0 {
                return self.recover_bus_off();
            }
            return false;
        }
        self.synced.store(true, Ordering::Relaxed);
        if p.rtr {
            return false; // RTR frames have no J2534 representation.
        }
        if p.is_echo() {
            let (taken, oldest_open) = {
                let mut slots = lock(&self.slots);
                let t = slots
                    .entries
                    .get_mut(p.echo_id as usize)
                    .and_then(Option::take);
                let oldest_open = slots.entries.iter().flatten().map(|(_, m)| m.seq).min();
                self.slot_cv.notify_one();
                (t, oldest_open)
            };
            if let Some((sent_at, meta)) = taken {
                self.counters.echoes.fetch_add(1, Ordering::Relaxed);
                let reordered = oldest_open.is_some_and(|o| o < meta.seq);
                if reordered {
                    self.counters.reordered.fetch_add(1, Ordering::Relaxed);
                }
                crate::trace!(
                    "TX echo seq={} slot={} after {:?}{}",
                    meta.seq,
                    p.echo_id,
                    sent_at.elapsed(),
                    if reordered {
                        "  ** REORDERED: an earlier frame is still outstanding **"
                    } else {
                        ""
                    }
                );
                self.finish_tx(meta, Some(ts));
            }
            return false;
        }
        crate::trace!(
            "RX id={:X}{} fd={} brs={} len={} data={:02X?}",
            p.frame.id,
            if p.frame.ext { "x" } else { "" },
            p.frame.fd,
            p.frame.brs,
            p.frame.data.len(),
            p.frame.data
        );
        let sinks: Vec<_> = lock(&self.sinks)
            .iter()
            .map(|(_, s)| Arc::clone(s))
            .collect();
        for s in sinks {
            s.on_frame(&p.frame, ts, p.overflow);
        }
        false
    }

    /// Restart the controller after a bus-off. Runs on the reader thread, which is the only
    /// reader of the IN pipe, so draining it inside `start` is safe. Gives up (marks the bus
    /// broken) if bus-off keeps recurring, e.g. wrong bit rate or a shorted bus.
    fn recover_bus_off(&self) -> bool {
        const WINDOW: Duration = Duration::from_secs(5);
        const MAX_RECOVERIES: usize = 5;
        {
            let mut recoveries = lock(&self.recoveries);
            let now = Instant::now();
            recoveries.retain(|t| now.duration_since(*t) < WINDOW);
            if recoveries.len() >= MAX_RECOVERIES {
                self.note_error("repeated bus-off, giving up");
                self.broken.store(true, Ordering::Relaxed);
                return false;
            }
            recoveries.push(now);
        }
        self.counters.bus_offs.fetch_add(1, Ordering::Relaxed);
        crate::trace!("bus-off: restarting the controller");
        self.fail_all_slots();
        let cfg = self.config();
        let restarted = {
            let _io = lock(&self.io);
            self.usb.start(&cfg)
        };
        match restarted {
            Ok(()) => {
                self.mark_started();
                true
            }
            Err(e) => {
                self.note_error(&format!("restart after bus-off failed: {e}"));
                self.broken.store(true, Ordering::Relaxed);
                false
            }
        }
    }

    /// Queue a frame for in-order transmission without waiting for a free hardware slot. Waits up
    /// to `timeout` only when the (deep) software queue itself is full.
    pub fn enqueue(&self, owner: u32, frame: CanFrame, timeout: Duration) -> Result<(), TxError> {
        if self.config().listen_only {
            return Err(TxError::ListenOnly);
        }
        let deadline = Instant::now() + timeout;
        let mut q = lock(&self.txq);
        loop {
            if q.len() < TX_QUEUE_LIMIT {
                q.push_back(Queued { owner, frame });
                drop(q);
                self.txq_cv.notify_one();
                return Ok(());
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(TxError::Full);
            }
            q = self
                .txq_space
                .wait_timeout(q, deadline - now)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
    }

    /// Drop frames of `owner` that are still waiting for a hardware slot (`CLEAR_TX_BUFFER`).
    pub fn clear_tx_queue(&self, owner: u32) {
        lock(&self.txq).retain(|q| q.owner != owner);
        self.txq_space.notify_all();
    }

    /// Feeds the software queue into the hardware slots, strictly in order.
    fn pump_loop(self: Arc<Self>, generation: u64) {
        while self.generation.load(Ordering::SeqCst) == generation {
            let item = {
                let mut q = lock(&self.txq);
                loop {
                    if self.generation.load(Ordering::SeqCst) != generation {
                        return;
                    }
                    if let Some(item) = q.pop_front() {
                        self.txq_space.notify_all();
                        break item;
                    }
                    q = self
                        .txq_cv
                        .wait_timeout(q, Duration::from_millis(50))
                        .unwrap_or_else(|e| e.into_inner())
                        .0;
                }
            };
            loop {
                match self.transmit_with(
                    item.owner,
                    item.frame.clone(),
                    None,
                    None,
                    Duration::from_millis(20),
                ) {
                    Ok(()) => break,
                    Err(TxError::Full) => {
                        if self.generation.load(Ordering::SeqCst) != generation {
                            return;
                        }
                    }
                    Err(e) => {
                        crate::trace!("queued frame {:X} dropped: {e:?}", item.frame.id);
                        break;
                    }
                }
            }
        }
    }

    fn ticker_loop(self: Arc<Self>, generation: u64) {
        while self.generation.load(Ordering::SeqCst) == generation {
            std::thread::sleep(Duration::from_millis(2));
            let now = Instant::now();
            // Expire frames that never produced an echo.
            let expired: Vec<TxMeta> = {
                let mut slots = lock(&self.slots);
                let mut v = Vec::new();
                for slot in slots.entries.iter_mut() {
                    if slot
                        .as_ref()
                        .is_some_and(|(t, _)| now.duration_since(*t) > TX_ECHO_TIMEOUT)
                    {
                        v.extend(slot.take().map(|(_, m)| m));
                    }
                }
                if !v.is_empty() {
                    self.slot_cv.notify_all();
                }
                v
            };
            for meta in expired {
                crate::trace!(
                    "TX id={:X} had no echo within {:?}",
                    meta.frame.id,
                    TX_ECHO_TIMEOUT
                );
                self.finish_tx(meta, None);
            }
            let sinks: Vec<_> = lock(&self.sinks)
                .iter()
                .map(|(_, s)| Arc::clone(s))
                .collect();
            for s in sinks {
                s.on_tick(now);
            }
        }
    }
}
