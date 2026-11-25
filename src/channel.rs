//! A J2534 channel (CAN, CAN FD, ISO15765, ISO15765 on FD) on top of a [`Bus`].
//!
//! Raw channels pass frames through J2534 mask/pattern filters. ISO15765 channels own
//! flow-control filters; each runs an ISO-TP receiver on the bus reader thread (it must
//! never block) and a per-channel worker thread that executes queued transmissions.

use crate::bus::{Bus, Indication, Sink, TxError, TxMeta, TxTicket};
use crate::gsusb::{validate_frame, CanFrame};
use crate::isotp::{self, FcStatus, FirstSegment, Pci, Reassembler, RxEvent, TxParams};
use crate::j2534::*;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const RX_QUEUE_LIMIT: usize = 20_000;
const JOB_QUEUE_LIMIT: usize = 256;
const MAX_FILTERS: usize = 10;
const WORKER_POLL: Duration = Duration::from_millis(50);

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// J2534 error code plus text for `PassThruGetLastError`.
#[derive(Debug, Clone)]
pub struct ApiErr(pub i32, pub String);

pub fn err<T>(code: i32, msg: impl Into<String>) -> Result<T, ApiErr> {
    Err(ApiErr(code, msg.into()))
}

pub type ApiResult<T> = Result<T, ApiErr>;

#[derive(Clone, Debug)]
pub struct RxMsg {
    pub rx_status: u32,
    pub timestamp: u32,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub loopback: bool,
    pub bs: u8,
    pub stmin: u8,
    pub bs_tx: u32,
    pub stmin_tx: u32,
    pub wait_limit: u32,
    pub pad_value: u8,
    pub n_as: u32,
    pub n_ar: u32,
    pub n_bs: u32,
    pub n_cr: u32,
    pub tx_dl: usize,
    pub can_mixed_format: u32,
    pub j1962_pins: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            loopback: false,
            bs: 0,
            stmin: 0,
            bs_tx: 0xFFFF,
            stmin_tx: 0xFFFF,
            wait_limit: 0,
            pad_value: 0x00,
            // ISO 15765-2 N_As/N_Ar/N_Bs/N_Cr timeout of 1 s.
            n_as: 1000,
            n_ar: 1000,
            n_bs: 1000,
            n_cr: 1000,
            tx_dl: 8,
            can_mixed_format: 0,
            j1962_pins: 0,
        }
    }
}

struct FcMsg {
    status: FcStatus,
    bs: u8,
    stmin: u8,
}

struct IsoRx {
    reasm: Mutex<Reassembler>,
    deadline: Mutex<Option<Instant>>,
    fc: Mutex<VecDeque<FcMsg>>,
    fc_cv: Condvar,
}

struct Filter {
    id: u32,
    kind: u32,
    mask: Vec<u8>,
    pattern: Vec<u8>,
    /// TxFlags of the pattern message (CAN_29BIT_ID selects the identifier width).
    flags: u32,
    /// Flow-control message data (CAN id + optional address byte) and its TxFlags.
    flow: Vec<u8>,
    flow_flags: u32,
    iso: Option<IsoRx>,
}

impl Filter {
    fn addr_len(&self) -> usize {
        (self.flow_flags & ISO15765_ADDR_TYPE != 0) as usize
    }

    fn flow_id(&self) -> u32 {
        u32::from_be_bytes(self.flow[..4].try_into().unwrap())
    }
}

struct Periodic {
    id: u32,
    frame: CanFrame,
    interval: Duration,
    next: Instant,
}

struct State {
    cfg: Config,
    filters: Vec<Arc<Filter>>,
    next_filter: u32,
    periodic: Vec<Periodic>,
    next_periodic: u32,
}

struct RxQueue {
    q: VecDeque<RxMsg>,
    overflow: bool,
}

struct JobDone {
    state: Mutex<Option<Result<u32, ApiErr>>>,
    cv: Condvar,
}

impl JobDone {
    fn finish(&self, r: Result<u32, ApiErr>) {
        *lock(&self.state) = Some(r);
        self.cv.notify_all();
    }

    fn wait(&self, deadline: Instant) -> Option<Result<u32, ApiErr>> {
        let mut s = lock(&self.state);
        loop {
            if let Some(r) = s.clone() {
                return Some(r);
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            s = self
                .cv
                .wait_timeout(s, deadline - now)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
    }
}

struct Job {
    data: Vec<u8>,
    tx_flags: u32,
    done: Arc<JobDone>,
}

pub struct Channel {
    pub id: u32,
    pub protocol: u32,
    pub flags: u32,
    bus: Arc<Bus>,
    state: Mutex<State>,
    rx: Mutex<RxQueue>,
    rx_cv: Condvar,
    jobs: Mutex<VecDeque<Job>>,
    jobs_cv: Condvar,
    closed: AtomicBool,
    worker: Mutex<Option<JoinHandle<()>>>,
}

pub fn matches(mask: &[u8], pattern: &[u8], data: &[u8]) -> bool {
    data.len() >= mask.len()
        && mask
            .iter()
            .zip(pattern)
            .zip(data)
            .all(|((m, p), d)| d & m == p & m)
}

fn jdata(frame: &CanFrame) -> Vec<u8> {
    let mut v = frame.id.to_be_bytes().to_vec();
    v.extend_from_slice(&frame.data);
    v
}

fn frame_status(frame: &CanFrame) -> u32 {
    (if frame.ext { CAN_29BIT_ID } else { 0 })
        | (if frame.fd { FD_CAN_FORMAT } else { 0 })
        | (if frame.brs { FD_CAN_BRS } else { 0 })
        | (if frame.esi { FD_CAN_ESI } else { 0 })
}

fn sleep_until(t: Instant) {
    loop {
        let now = Instant::now();
        if now >= t {
            return;
        }
        let left = t - now;
        if left > Duration::from_micros(1500) {
            std::thread::sleep(left - Duration::from_micros(1000));
        } else {
            std::thread::yield_now();
        }
    }
}

impl Channel {
    pub fn new(id: u32, protocol: u32, flags: u32, bus: Arc<Bus>, cfg: Config) -> Arc<Self> {
        let ch = Arc::new(Self {
            id,
            protocol,
            flags,
            bus,
            state: Mutex::new(State {
                cfg,
                filters: Vec::new(),
                next_filter: 1,
                periodic: Vec::new(),
                next_periodic: 1,
            }),
            rx: Mutex::new(RxQueue {
                q: VecDeque::new(),
                overflow: false,
            }),
            rx_cv: Condvar::new(),
            jobs: Mutex::new(VecDeque::new()),
            jobs_cv: Condvar::new(),
            closed: AtomicBool::new(false),
            worker: Mutex::new(None),
        });
        if is_iso_protocol(protocol) {
            let worker = Arc::clone(&ch);
            *lock(&ch.worker) = Some(
                std::thread::Builder::new()
                    .name(format!("j2534-iso-tx-{id}"))
                    .spawn(move || worker.worker_loop())
                    .expect("spawn iso worker"),
            );
        }
        ch
    }

    /// Set once the adapter stopped answering (unplugged or USB failure); the channel is
    /// unusable afterwards and the application has to disconnect and reconnect.
    pub fn device_error(&self) -> Option<String> {
        self.bus.is_broken()
    }

    pub fn is_iso(&self) -> bool {
        is_iso_protocol(self.protocol)
    }

    fn is_fd(&self) -> bool {
        is_fd_protocol(self.protocol)
    }

    pub fn config(&self) -> Config {
        lock(&self.state).cfg.clone()
    }

    pub fn update_config<R>(&self, f: impl FnOnce(&mut Config) -> R) -> R {
        f(&mut lock(&self.state).cfg)
    }

    pub fn shutdown(&self) {
        self.closed.store(true, Ordering::SeqCst);
        self.jobs_cv.notify_all();
        for f in lock(&self.state).filters.iter() {
            if let Some(iso) = &f.iso {
                iso.fc_cv.notify_all();
            }
        }
        self.rx_cv.notify_all();
        if let Some(h) = lock(&self.worker).take() {
            let _ = h.join();
        }
        let pending: Vec<Job> = lock(&self.jobs).drain(..).collect();
        for job in pending {
            job.done.finish(err(ERR_FAILED, "channel disconnected"));
        }
    }

    // ---- receive queue -------------------------------------------------------------

    fn push_rx(&self, mut msg: RxMsg) {
        let mut rx = lock(&self.rx);
        if rx.q.len() >= RX_QUEUE_LIMIT {
            rx.overflow = true;
            return;
        }
        if std::mem::take(&mut rx.overflow) {
            msg.rx_status |= BUFFER_OVERFLOW_FLAG;
        }
        rx.q.push_back(msg);
        drop(rx);
        self.rx_cv.notify_all();
    }

    pub fn clear_rx(&self) {
        let mut rx = lock(&self.rx);
        rx.q.clear();
        rx.overflow = false;
    }

    /// Returns up to `max` messages, waiting up to `timeout` for the first/remaining ones.
    pub fn read(&self, max: usize, timeout: Duration) -> Vec<RxMsg> {
        let deadline = Instant::now() + timeout;
        let mut out = Vec::new();
        let mut rx = lock(&self.rx);
        loop {
            while out.len() < max {
                match rx.q.pop_front() {
                    Some(m) => out.push(m),
                    None => break,
                }
            }
            let now = Instant::now();
            if out.len() >= max || now >= deadline || self.closed.load(Ordering::SeqCst) {
                return out;
            }
            rx = self
                .rx_cv
                .wait_timeout(rx, deadline - now)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
    }

    // ---- filters ---------------------------------------------------------------------

    pub fn start_filter(
        &self,
        kind: u32,
        mask: &PassThruMsg,
        pattern: &PassThruMsg,
        flow: Option<&PassThruMsg>,
    ) -> ApiResult<u32> {
        for m in [Some(mask), Some(pattern), flow].into_iter().flatten() {
            if m.protocol_id != self.protocol {
                return err(
                    ERR_MSG_PROTOCOL_ID,
                    "filter message protocol differs from channel",
                );
            }
        }
        if mask.data_size == 0
            || mask.data_size != pattern.data_size
            || mask.data_size as usize > 4 + 64
        {
            return err(
                ERR_INVALID_MSG,
                "mask and pattern must have equal length of 1..68 bytes",
            );
        }
        let (flow_data, flow_flags) = match (self.is_iso(), kind) {
            (false, PASS_FILTER | BLOCK_FILTER) => (Vec::new(), 0),
            (true, FLOW_CONTROL_FILTER) => {
                let f = match flow {
                    Some(f) => f,
                    None => {
                        return err(
                            ERR_NULL_PARAMETER,
                            "flow control filter needs a flow control message",
                        )
                    }
                };
                let addr = (f.tx_flags & ISO15765_ADDR_TYPE != 0) as u32;
                if f.data_size < 4 + addr {
                    return err(ERR_INVALID_MSG, "flow control message shorter than CAN id");
                }
                (f.payload()[..(4 + addr as usize)].to_vec(), f.tx_flags)
            }
            _ => {
                return err(
                    ERR_NOT_SUPPORTED,
                    "ISO15765 channels take FLOW_CONTROL_FILTER; CAN channels take PASS/BLOCK",
                )
            }
        };
        let mut st = lock(&self.state);
        if st.filters.len() >= MAX_FILTERS {
            return err(ERR_EXCEEDED_LIMIT, "too many message filters");
        }
        if kind == FLOW_CONTROL_FILTER
            && st.filters.iter().any(|f| {
                f.kind == FLOW_CONTROL_FILTER
                    && f.mask == mask.payload()
                    && f.pattern == pattern.payload()
            })
        {
            return err(ERR_NOT_UNIQUE, "duplicate flow control filter");
        }
        let id = st.next_filter;
        st.next_filter += 1;
        let addr = (flow_flags & ISO15765_ADDR_TYPE != 0) as usize;
        let iso = (kind == FLOW_CONTROL_FILTER).then(|| IsoRx {
            reasm: Mutex::new(Reassembler::new(MSG_DATA_MAX - 4 - addr)),
            deadline: Mutex::new(None),
            fc: Mutex::new(VecDeque::new()),
            fc_cv: Condvar::new(),
        });
        st.filters.push(Arc::new(Filter {
            id,
            kind,
            mask: mask.payload().to_vec(),
            pattern: pattern.payload().to_vec(),
            flags: pattern.tx_flags,
            flow: flow_data,
            flow_flags,
            iso,
        }));
        crate::trace!("channel {} filter {} kind={} added", self.id, id, kind);
        Ok(id)
    }

    pub fn stop_filter(&self, id: u32) -> ApiResult<()> {
        let mut st = lock(&self.state);
        let before = st.filters.len();
        st.filters.retain(|f| f.id != id);
        if st.filters.len() == before {
            return err(ERR_INVALID_FILTER_ID, "unknown filter id");
        }
        Ok(())
    }

    pub fn clear_filters(&self) {
        lock(&self.state).filters.clear();
    }

    fn filters(&self) -> Vec<Arc<Filter>> {
        lock(&self.state).filters.clone()
    }

    // ---- periodic messages (raw channels) ---------------------------------------------

    pub fn start_periodic(&self, msg: &PassThruMsg, interval_ms: u32) -> ApiResult<u32> {
        if self.is_iso() {
            return err(
                ERR_NOT_SUPPORTED,
                "periodic messages are only supported on raw CAN channels",
            );
        }
        if !(5..=65_000).contains(&interval_ms) {
            return err(
                ERR_INVALID_TIME_INTERVAL,
                "periodic interval must be 5..65000 ms",
            );
        }
        let frame = self.frame_from_msg(msg)?;
        let mut st = lock(&self.state);
        if st.periodic.len() >= MAX_PERIODIC_MSGS {
            return err(
                ERR_EXCEEDED_LIMIT,
                "ten periodic messages are already active",
            );
        }
        let id = st.next_periodic;
        st.next_periodic += 1;
        st.periodic.push(Periodic {
            id,
            frame,
            interval: Duration::from_millis(interval_ms as u64),
            next: Instant::now(), // first transmission without delay
        });
        Ok(id)
    }

    pub fn stop_periodic(&self, id: u32) -> ApiResult<()> {
        let mut st = lock(&self.state);
        let before = st.periodic.len();
        st.periodic.retain(|p| p.id != id);
        if st.periodic.len() == before {
            return err(ERR_INVALID_MSG_ID, "unknown periodic message id");
        }
        Ok(())
    }

    pub fn clear_periodic(&self) {
        lock(&self.state).periodic.clear();
    }

    // ---- transmit --------------------------------------------------------------------

    fn frame_from_msg(&self, m: &PassThruMsg) -> ApiResult<CanFrame> {
        if m.protocol_id != self.protocol {
            return err(ERR_MSG_PROTOCOL_ID, "message protocol differs from channel");
        }
        if (m.data_size as usize) < 4 || m.data_size as usize > MSG_DATA_MAX {
            return err(ERR_INVALID_MSG, "CAN message needs a 4 byte identifier");
        }
        let d = m.payload();
        let id = u32::from_be_bytes(d[..4].try_into().unwrap());
        let ext = m.tx_flags & CAN_29BIT_ID != 0;
        let fd = m.tx_flags & FD_CAN_FORMAT != 0;
        let brs = m.tx_flags & FD_CAN_BRS != 0;
        if (fd || brs) && !self.is_fd() {
            return err(ERR_INVALID_MSG, "FD flags on a classic CAN channel");
        }
        if id > if ext { 0x1fff_ffff } else { 0x7ff } {
            return err(
                ERR_INVALID_MSG,
                "identifier exceeds the selected id width (CAN_29BIT_ID)",
            );
        }
        let frame = CanFrame {
            id,
            ext,
            fd,
            brs,
            esi: false,
            data: d[4..].to_vec(),
        };
        validate_frame(&frame, self.is_fd()).map_err(|e| ApiErr(ERR_INVALID_MSG, e))?;
        Ok(frame)
    }

    /// Transmit messages; returns how many were accepted plus an optional error.
    pub fn write(&self, msgs: &[&PassThruMsg], timeout: Duration) -> (usize, Option<ApiErr>) {
        if self.is_iso() {
            self.write_iso(msgs, timeout)
        } else {
            self.write_raw(msgs, timeout)
        }
    }

    fn write_raw(&self, msgs: &[&PassThruMsg], timeout: Duration) -> (usize, Option<ApiErr>) {
        let deadline = Instant::now() + timeout;
        for (i, m) in msgs.iter().enumerate() {
            let frame = match self.frame_from_msg(m) {
                Ok(f) => f,
                Err(e) => return (i, Some(e)),
            };
            let left = deadline.saturating_duration_since(Instant::now());
            // Queued, not sent directly: with Timeout 0 a burst must not fail just because the ten
            // hardware slots are busy for a moment (the bus is slower than the application).
            if let Err(e) = self.bus.enqueue(self.id, frame, left) {
                return (i, Some(tx_error(e, timeout.is_zero())));
            }
        }
        (msgs.len(), None)
    }

    fn write_iso(&self, msgs: &[&PassThruMsg], timeout: Duration) -> (usize, Option<ApiErr>) {
        let mut dones = Vec::new();
        for m in msgs {
            if let Err(e) = self.check_iso_msg(m) {
                return (dones.len(), Some(e));
            }
            let done = Arc::new(JobDone {
                state: Mutex::new(None),
                cv: Condvar::new(),
            });
            let mut jobs = lock(&self.jobs);
            if jobs.len() >= JOB_QUEUE_LIMIT {
                return (
                    dones.len(),
                    Some(ApiErr(ERR_BUFFER_FULL, "transmit queue full".into())),
                );
            }
            jobs.push_back(Job {
                data: m.payload().to_vec(),
                tx_flags: m.tx_flags,
                done: Arc::clone(&done),
            });
            drop(jobs);
            self.jobs_cv.notify_one();
            dones.push(done);
        }
        if timeout.is_zero() {
            return (dones.len(), None);
        }
        let deadline = Instant::now() + timeout;
        for (i, d) in dones.iter().enumerate() {
            match d.wait(deadline) {
                Some(Ok(_)) => {}
                Some(Err(e)) => return (i, Some(e)),
                None => {
                    return (
                        i,
                        Some(ApiErr(
                            ERR_TIMEOUT,
                            "ISO15765 transmission still in progress".into(),
                        )),
                    )
                }
            }
        }
        (dones.len(), None)
    }

    fn check_iso_msg(&self, m: &PassThruMsg) -> ApiResult<()> {
        if m.protocol_id != self.protocol {
            return err(ERR_MSG_PROTOCOL_ID, "message protocol differs from channel");
        }
        let hdr = 4 + (m.tx_flags & ISO15765_ADDR_TYPE != 0) as usize;
        let size = m.data_size as usize;
        if size <= hdr || size > MSG_DATA_MAX {
            return err(
                ERR_INVALID_MSG,
                "ISO15765 message needs identifier and at least one data byte",
            );
        }
        if (m.tx_flags & (FD_CAN_FORMAT | FD_CAN_BRS) != 0) && !self.is_fd() {
            return err(ERR_INVALID_MSG, "FD flags on a classic ISO15765 channel");
        }
        let ext = m.tx_flags & CAN_29BIT_ID != 0;
        let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
        if id > if ext { 0x1fff_ffff } else { 0x7ff } {
            return err(
                ERR_INVALID_MSG,
                "identifier exceeds the selected id width (CAN_29BIT_ID)",
            );
        }
        if self.find_flow_filter(&m.data[..hdr], m.tx_flags).is_none() {
            return err(
                ERR_NO_FLOW_CONTROL,
                "no flow control filter matches the transmit identifier",
            );
        }
        Ok(())
    }

    fn find_flow_filter(&self, id_and_addr: &[u8], tx_flags: u32) -> Option<Arc<Filter>> {
        self.filters().into_iter().find(|f| {
            f.kind == FLOW_CONTROL_FILTER
                && f.flow == id_and_addr
                && (f.flow_flags & CAN_29BIT_ID) == (tx_flags & CAN_29BIT_ID)
        })
    }

    pub fn clear_tx(&self) {
        self.bus.clear_tx_queue(self.id);
        let pending: Vec<Job> = lock(&self.jobs).drain(..).collect();
        for job in pending {
            job.done.finish(err(ERR_FAILED, "transmit queue cleared"));
        }
    }

    // ---- ISO-TP transmit worker --------------------------------------------------------

    fn worker_loop(self: Arc<Self>) {
        loop {
            let job = {
                let mut jobs = lock(&self.jobs);
                loop {
                    if self.closed.load(Ordering::SeqCst) {
                        return;
                    }
                    if let Some(j) = jobs.pop_front() {
                        break j;
                    }
                    jobs = self
                        .jobs_cv
                        .wait_timeout(jobs, WORKER_POLL)
                        .unwrap_or_else(|e| e.into_inner())
                        .0;
                }
            };
            let result = self.run_job(&job);
            if let Err(e) = &result {
                crate::trace!("channel {} ISO-TP transmit failed: {}", self.id, e.1);
            }
            job.done.finish(result);
        }
    }

    fn send_frame(
        &self,
        frame: CanFrame,
        ticket: Option<Arc<TxTicket>>,
        indication: Option<Indication>,
        limit: Duration,
    ) -> ApiResult<()> {
        let deadline = Instant::now() + limit;
        loop {
            // Retry while the TX slots are exhausted, but stay responsive to disconnects.
            match self.bus.transmit_with(
                self.id,
                frame.clone(),
                ticket.clone(),
                indication.clone(),
                Duration::from_millis(20),
            ) {
                Ok(()) => return Ok(()),
                Err(TxError::Full) => {
                    if self.closed.load(Ordering::SeqCst) {
                        return err(ERR_FAILED, "channel disconnected");
                    }
                    if Instant::now() >= deadline {
                        return err(ERR_TIMEOUT, "no free transmit slot");
                    }
                }
                Err(e) => return Err(tx_error(e, false)),
            }
        }
    }

    fn run_job(&self, job: &Job) -> Result<u32, ApiErr> {
        let cfg = self.config();
        let addr_type = job.tx_flags & ISO15765_ADDR_TYPE != 0;
        let hdr = 4 + addr_type as usize;
        let id = u32::from_be_bytes(job.data[..4].try_into().unwrap());
        let ext = job.tx_flags & CAN_29BIT_ID != 0;
        let ext_addr = addr_type.then(|| job.data[4]);
        let user = &job.data[hdr..];
        let filter = self
            .find_flow_filter(&job.data[..hdr], job.tx_flags)
            .ok_or(ApiErr(
                ERR_NO_FLOW_CONTROL,
                "flow control filter was removed".into(),
            ))?;
        let iso = filter.iso.as_ref().unwrap();
        let fd = self.is_fd() && job.tx_flags & FD_CAN_FORMAT != 0;
        let brs = fd && job.tx_flags & FD_CAN_BRS != 0;
        let params = TxParams {
            tx_dl: if fd { cfg.tx_dl.clamp(8, 64) } else { 8 },
            pad: (job.tx_flags & ISO15765_FRAME_PAD != 0).then_some(cfg.pad_value),
            fd,
            ext_addr,
        };
        let frame = |data: Vec<u8>| CanFrame {
            id,
            ext,
            fd,
            brs,
            esi: false,
            data,
        };
        let n_as = Duration::from_millis(cfg.n_as as u64);
        // Completion indication (identifier only), delivered when the last frame is echoed.
        let done_indication = || Indication {
            rx_status: TX_MSG_TYPE
                | (if ext { CAN_29BIT_ID } else { 0 })
                | (if addr_type { ISO15765_ADDR_TYPE } else { 0 }),
            data: job.data[..hdr].to_vec(),
        };
        let wait_tx = |ticket: &Arc<TxTicket>| -> ApiResult<u32> {
            match ticket.wait(n_as.max(Duration::from_millis(50))) {
                Some(Some(ts)) => Ok(ts),
                Some(None) => err(ERR_TIMEOUT, "frame was not acknowledged on the bus"),
                None => err(
                    ERR_TIMEOUT,
                    "no transmit confirmation from the adapter (N_As)",
                ),
            }
        };

        let last_ts = match isotp::first_segment(user, &params) {
            FirstSegment::Single(payload) => {
                let t = TxTicket::new();
                self.send_frame(
                    frame(payload),
                    Some(Arc::clone(&t)),
                    Some(done_indication()),
                    n_as,
                )?;
                wait_tx(&t)?
            }
            FirstSegment::First(payload, mut offset) => {
                lock(&iso.fc).clear();
                let t = TxTicket::new();
                self.send_frame(frame(payload), Some(Arc::clone(&t)), None, n_as)?;
                wait_tx(&t)?;
                let mut sn = 1u8;
                let mut waits = 0u32;
                let mut last_cf: Option<Instant> = None;
                let final_ts: u32 = 'blocks: loop {
                    let fc = self.wait_flow_control(iso, Duration::from_millis(cfg.n_bs as u64))?;
                    let (bs, stmin_raw) = match fc.status {
                        FcStatus::Overflow => {
                            return err(ERR_BUFFER_FULL, "receiver reported buffer overflow");
                        }
                        FcStatus::Wait => {
                            waits += 1;
                            if cfg.wait_limit != 0 && waits > cfg.wait_limit {
                                return err(ERR_TIMEOUT, "too many flow control WAIT frames");
                            }
                            continue 'blocks;
                        }
                        FcStatus::ContinueToSend => (
                            if cfg.bs_tx != 0xFFFF {
                                cfg.bs_tx as u8
                            } else {
                                fc.bs
                            },
                            if cfg.stmin_tx != 0xFFFF {
                                cfg.stmin_tx as u8
                            } else {
                                fc.stmin
                            },
                        ),
                    };
                    let stmin = isotp::decode_stmin(stmin_raw);
                    let mut in_block = 0u32;
                    loop {
                        if self.closed.load(Ordering::SeqCst) {
                            return err(ERR_FAILED, "channel disconnected");
                        }
                        let (cf, took) = isotp::consecutive_frame(&user[offset..], sn, &params);
                        let last = offset + took >= user.len();
                        if let Some(prev) = last_cf {
                            sleep_until(prev + stmin);
                        }
                        let ticket = last.then(TxTicket::new);
                        let ind = last.then(done_indication);
                        self.send_frame(frame(cf), ticket.clone(), ind, n_as)?;
                        last_cf = Some(Instant::now());
                        offset += took;
                        sn = (sn + 1) & 0xf;
                        in_block += 1;
                        if let Some(t) = ticket {
                            break 'blocks wait_tx(&t)?;
                        }
                        if bs != 0 && in_block >= bs as u32 {
                            continue 'blocks;
                        }
                    }
                };
                final_ts
            }
        };
        Ok(last_ts)
    }

    fn wait_flow_control(&self, iso: &IsoRx, timeout: Duration) -> ApiResult<FcMsg> {
        let deadline = Instant::now() + timeout;
        let mut q = lock(&iso.fc);
        loop {
            if let Some(m) = q.pop_front() {
                return Ok(m);
            }
            if self.closed.load(Ordering::SeqCst) {
                return err(ERR_FAILED, "channel disconnected");
            }
            let now = Instant::now();
            if now >= deadline {
                return err(
                    ERR_TIMEOUT,
                    "no flow control frame from the receiver (N_Bs)",
                );
            }
            q = iso
                .fc_cv
                .wait_timeout(q, (deadline - now).min(WORKER_POLL))
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
    }

    // ---- receive paths -----------------------------------------------------------------

    fn id_width_allowed(&self, frame: &CanFrame) -> bool {
        self.flags & CAN_ID_BOTH != 0 || frame.ext == (self.flags & CAN_29BIT_ID != 0)
    }

    /// J2534 filtering for raw channels: at least one pass filter must match, no block filter.
    fn raw_accepts(&self, frame: &CanFrame, data: &[u8]) -> bool {
        if (frame.fd && !self.is_fd()) || !self.id_width_allowed(frame) {
            return false;
        }
        let both = self.flags & CAN_ID_BOTH != 0;
        let applies = |f: &Filter| {
            matches(&f.mask, &f.pattern, data)
                && (!both || (f.flags & CAN_29BIT_ID != 0) == frame.ext)
        };
        let filters = self.filters();
        let blocked = filters.iter().any(|f| f.kind == BLOCK_FILTER && applies(f));
        let passed = filters.iter().any(|f| f.kind == PASS_FILTER && applies(f));
        passed && !blocked
    }

    fn raw_rx(&self, frame: &CanFrame, ts: u32) {
        let data = jdata(frame);
        if self.raw_accepts(frame, &data) {
            self.push_rx(RxMsg {
                rx_status: frame_status(frame),
                timestamp: ts,
                data,
            });
        }
    }

    fn send_fc(&self, f: &Filter, status: FcStatus, bs: u8, stmin: u8, like: &CanFrame) {
        let pad = (f.flow_flags & ISO15765_FRAME_PAD != 0).then_some(self.config().pad_value);
        let params = TxParams {
            tx_dl: 8,
            pad,
            fd: false,
            ext_addr: (f.addr_len() == 1).then(|| f.flow[4]),
        };
        let fc = CanFrame {
            id: f.flow_id(),
            ext: f.flow_flags & CAN_29BIT_ID != 0,
            // Same frame format as the peer used, but BRS only when the application's flow
            // control message asks for it: BRS transmissions can fail on marginal buses.
            fd: like.fd,
            brs: like.fd && f.flow_flags & FD_CAN_BRS != 0,
            esi: false,
            data: isotp::flow_control_frame(status, bs, stmin, &params),
        };
        // Never block the reader thread: a missing slot drops the flow control frame.
        if let Err(e) = self.bus.transmit(0, fc, None, Duration::ZERO) {
            crate::trace!("channel {} could not send flow control: {:?}", self.id, e);
        }
    }

    fn iso_rx(&self, frame: &CanFrame, ts: u32) {
        if frame.fd && !self.is_fd() {
            return;
        }
        let data = jdata(frame);
        for f in self.filters() {
            let Some(iso) = f.iso.as_ref() else { continue };
            if (f.flags & CAN_29BIT_ID != 0) != frame.ext || !matches(&f.mask, &f.pattern, &data) {
                continue;
            }
            let addr = f.addr_len();
            if frame.data.len() <= addr {
                return;
            }
            let payload = &frame.data[addr..];
            let cfg = self.config();
            if let Ok(Pci::FlowControl { status, bs, stmin }) = isotp::parse_pci(payload) {
                lock(&iso.fc).push_back(FcMsg { status, bs, stmin });
                iso.fc_cv.notify_all();
                return;
            }
            let id_bytes = frame.id.to_be_bytes();
            let mut header = id_bytes.to_vec();
            header.extend(frame.data.get(..addr).unwrap_or(&[]));
            let status = frame_status(frame) | if addr == 1 { ISO15765_ADDR_TYPE } else { 0 };
            let event = lock(&iso.reasm).feed(payload, cfg.bs);
            match event {
                RxEvent::Ignore => {
                    if lock(&iso.reasm).is_active() {
                        *lock(&iso.deadline) =
                            Some(Instant::now() + Duration::from_millis(cfg.n_cr as u64));
                    }
                }
                RxEvent::Message(user) => {
                    *lock(&iso.deadline) = None;
                    let mut d = header;
                    d.extend_from_slice(&user);
                    self.push_rx(RxMsg {
                        rx_status: status,
                        timestamp: ts,
                        data: d,
                    });
                }
                RxEvent::Started { total } => {
                    crate::trace!("channel {} ISO-TP first frame, {} bytes", self.id, total);
                    *lock(&iso.deadline) =
                        Some(Instant::now() + Duration::from_millis(cfg.n_cr as u64));
                    self.push_rx(RxMsg {
                        rx_status: START_OF_MESSAGE
                            | (status & !FD_CAN_FORMAT & !FD_CAN_BRS & !FD_CAN_ESI),
                        timestamp: ts,
                        data: header,
                    });
                    self.send_fc(&f, FcStatus::ContinueToSend, cfg.bs, cfg.stmin, frame);
                }
                RxEvent::SendFlowControl(st) => {
                    self.send_fc(&f, st, cfg.bs, cfg.stmin, frame);
                    *lock(&iso.deadline) = if st == FcStatus::ContinueToSend {
                        Some(Instant::now() + Duration::from_millis(cfg.n_cr as u64))
                    } else {
                        None
                    };
                }
                RxEvent::Abort(why) => {
                    crate::trace!("channel {} ISO-TP receive aborted: {why}", self.id);
                    *lock(&iso.deadline) = None;
                }
            }
            return;
        }
    }
}

fn tx_error(e: TxError, nonblocking: bool) -> ApiErr {
    match e {
        TxError::Full if nonblocking => ApiErr(ERR_BUFFER_FULL, "no free transmit slot".into()),
        TxError::Full => ApiErr(
            ERR_TIMEOUT,
            "no free transmit slot within the timeout".into(),
        ),
        TxError::ListenOnly => ApiErr(ERR_NOT_SUPPORTED, "adapter is in listen-only mode".into()),
        TxError::Usb(m) => ApiErr(ERR_DEVICE_NOT_CONNECTED, m),
    }
}

impl Sink for Channel {
    fn on_frame(&self, frame: &CanFrame, ts: u32, overflow: bool) {
        if overflow {
            lock(&self.rx).overflow = true;
        }
        if self.is_iso() {
            self.iso_rx(frame, ts);
        } else {
            self.raw_rx(frame, ts);
        }
    }

    fn on_tx_done(&self, meta: &TxMeta, ts: Option<u32>) {
        if meta.owner != self.id {
            return;
        }
        let Some(ts) = ts else {
            crate::trace!(
                "channel {} frame {:X} was not transmitted",
                self.id,
                meta.frame.id
            );
            return;
        };
        if !self.config().loopback {
            return;
        }
        if let Some(ind) = &meta.indication {
            // ISO15765: one indication per completed message, only for its last frame.
            self.push_rx(RxMsg {
                rx_status: ind.rx_status,
                timestamp: ts,
                data: ind.data.clone(),
            });
        } else if !self.is_iso() {
            // Raw CAN: echoes pass through the filters like received frames.
            let data = jdata(&meta.frame);
            if self.raw_accepts(&meta.frame, &data) {
                self.push_rx(RxMsg {
                    rx_status: TX_MSG_TYPE | frame_status(&meta.frame),
                    timestamp: ts,
                    data,
                });
            }
        }
    }

    fn on_tick(&self, now: Instant) {
        if self.closed.load(Ordering::SeqCst) {
            return;
        }
        if self.is_iso() {
            for f in self.filters() {
                if let Some(iso) = &f.iso {
                    let mut dl = lock(&iso.deadline);
                    if dl.is_some_and(|d| now >= d) {
                        *dl = None;
                        lock(&iso.reasm).reset();
                        crate::trace!("channel {} ISO-TP receive timed out (N_Cr)", self.id);
                    }
                }
            }
            return;
        }
        let due: Vec<CanFrame> = {
            let mut st = lock(&self.state);
            st.periodic
                .iter_mut()
                .filter(|p| now >= p.next)
                .map(|p| {
                    p.next = now + p.interval;
                    p.frame.clone()
                })
                .collect()
        };
        for frame in due {
            let _ = self.bus.transmit(self.id, frame, None, Duration::ZERO);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mask_pattern_matching() {
        // pass 0x18DAF110 exactly, ignore payload
        let mask = [0xff, 0xff, 0xff, 0xff];
        let pat = [0x18, 0xda, 0xf1, 0x10];
        assert!(matches(
            &mask,
            &pat,
            &[0x18, 0xda, 0xf1, 0x10, 0x02, 0x7e, 0x00]
        ));
        assert!(!matches(&mask, &pat, &[0x18, 0xda, 0xf1, 0x11, 0x02]));
        // pass-all
        assert!(matches(&[0, 0, 0, 0], &[0, 0, 0, 0], &[1, 2, 3, 4]));
        // frame shorter than the mask never matches
        assert!(!matches(&[0xff; 5], &[0; 5], &[0, 0, 0, 0]));
        // partial id mask (ECU address byte only)
        assert!(matches(
            &[0, 0, 0, 0xff],
            &[0, 0, 0, 0x10],
            &[0x18, 0xda, 0xf1, 0x10]
        ));
    }

    #[test]
    fn frame_status_bits() {
        let f = CanFrame {
            id: 1,
            ext: true,
            fd: true,
            brs: true,
            esi: true,
            data: vec![],
        };
        assert_eq!(
            frame_status(&f),
            CAN_29BIT_ID | FD_CAN_FORMAT | FD_CAN_BRS | FD_CAN_ESI
        );
        assert_eq!(frame_status(&CanFrame::classic(1, false, &[])), 0);
        assert_eq!(
            jdata(&CanFrame::classic(0x7e8, false, &[1, 2])),
            vec![0, 0, 7, 0xe8, 1, 2]
        );
    }
}
