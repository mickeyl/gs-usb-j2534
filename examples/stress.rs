//! Longer-running checks of the built DLL, driven through its exported J2534 API.
//!
//!   stress uds    --payload 22F190 [--req 18DA10F1 --resp 18DAF110 --ext --pad AA --dl 8]
//!                 one UDS request on an ISO15765-FD channel (multi-frame responses are reassembled
//!                 by the driver); the request is whatever --payload says, nothing else is sent
//!   stress soak   --minutes 10     passive capture on a raw FD channel plus cyclic TesterPresent
//!                 (2 Hz) on an ISO15765-FD channel sharing the same adapter
//!   stress cycle  --count 20       open / connect / capture / disconnect / close, repeatedly
//!   stress unplug                  capture until the adapter disappears, then wait for it to return
//!   stress burst  --count 20      flash-tool-like transfer: bursts of 65 FD frames of 64 bytes written back to
//!                 back with timeout 0 to an unused id; checks that no write is refused and that the
//!                 transmit confirmations arrive complete and in order
//!   stress brs    --frames 40 [--nobrs]
//!                 replays a flash-tool-like burst of read-only UDS requests (22 F1 xx) as raw FD frames
//!                 and counts how many leave the controller (TX echo) and how many the ECU answers
//!
//! Defaults: 500 kbit/s nominal, 2 Mbit/s data, 29-bit ids, request 18DA10F1 / response 18DAF110.

use gs_usb_j2534::j2534::*;
use libloading::Library;
use std::collections::{BTreeMap, HashMap};
use std::ffi::{c_char, c_void, CStr};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

type Open = unsafe extern "system" fn(*const c_void, *mut u32) -> i32;
type Close = unsafe extern "system" fn(u32) -> i32;
type Connect = unsafe extern "system" fn(u32, u32, u32, u32, *mut u32) -> i32;
type Disconnect = unsafe extern "system" fn(u32) -> i32;
type Read = unsafe extern "system" fn(u32, *mut PassThruMsg, *mut u32, u32) -> i32;
type Write = unsafe extern "system" fn(u32, *const PassThruMsg, *mut u32, u32) -> i32;
type StartFilter = unsafe extern "system" fn(
    u32,
    u32,
    *const PassThruMsg,
    *const PassThruMsg,
    *const PassThruMsg,
    *mut u32,
) -> i32;
type Ioctl = unsafe extern "system" fn(u32, u32, *mut c_void, *mut c_void) -> i32;
type LastError = unsafe extern "system" fn(*mut c_char) -> i32;

struct Api {
    _lib: Library,
    open: Open,
    close: Close,
    connect: Connect,
    disconnect: Disconnect,
    read: Read,
    write: Write,
    start_filter: StartFilter,
    ioctl: Ioctl,
    last_error: LastError,
}

fn msg(protocol: u32, tx_flags: u32, data: &[u8]) -> Box<PassThruMsg> {
    let mut m: Box<PassThruMsg> = unsafe { Box::new(std::mem::zeroed()) };
    m.protocol_id = protocol;
    m.tx_flags = tx_flags;
    m.data_size = data.len() as u32;
    m.extra_data_index = data.len() as u32;
    m.data[..data.len()].copy_from_slice(data);
    m
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

impl Api {
    fn load(path: &str) -> Result<Api, String> {
        let lib = unsafe { Library::new(path) }.map_err(|e| format!("load {path}: {e}"))?;
        macro_rules! sym {
            ($t:ty, $n:literal) => {
                *unsafe { lib.get::<$t>($n) }
                    .map_err(|e| format!("export {}: {e}", String::from_utf8_lossy($n)))?
            };
        }
        Ok(Api {
            open: sym!(Open, b"PassThruOpen"),
            close: sym!(Close, b"PassThruClose"),
            connect: sym!(Connect, b"PassThruConnect"),
            disconnect: sym!(Disconnect, b"PassThruDisconnect"),
            read: sym!(Read, b"PassThruReadMsgs"),
            write: sym!(Write, b"PassThruWriteMsgs"),
            start_filter: sym!(StartFilter, b"PassThruStartMsgFilter"),
            ioctl: sym!(Ioctl, b"PassThruIoctl"),
            last_error: sym!(LastError, b"PassThruGetLastError"),
            _lib: lib,
        })
    }

    fn text(&self) -> String {
        let mut buf = [0 as c_char; 128];
        unsafe { (self.last_error)(buf.as_mut_ptr()) };
        unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    }

    fn check(&self, what: &str, rc: i32) -> Result<(), String> {
        if rc == 0 {
            Ok(())
        } else {
            Err(format!("{what} failed: 0x{rc:X} ({})", self.text()))
        }
    }

    fn open_device(&self) -> Result<u32, (i32, String)> {
        let mut dev = 0u32;
        let rc = unsafe { (self.open)(std::ptr::null(), &mut dev) };
        if rc == 0 {
            Ok(dev)
        } else {
            Err((rc, self.text()))
        }
    }

    fn connect_channel(&self, dev: u32, protocol: u32, nominal: u32) -> Result<u32, String> {
        let mut ch = 0u32;
        self.check("PassThruConnect", unsafe {
            (self.connect)(dev, protocol, CAN_ID_BOTH, nominal, &mut ch)
        })?;
        Ok(ch)
    }

    fn set_config(&self, ch: u32, params: &[(u32, u32)]) -> Result<(), String> {
        let mut items: Vec<SConfig> = params
            .iter()
            .map(|&(parameter, value)| SConfig { parameter, value })
            .collect();
        let mut list = SConfigList {
            num_of_params: items.len() as u32,
            config_ptr: items.as_mut_ptr(),
        };
        self.check("SET_CONFIG", unsafe {
            (self.ioctl)(
                ch,
                SET_CONFIG,
                &mut list as *mut _ as *mut c_void,
                std::ptr::null_mut(),
            )
        })
    }

    fn pass_all(&self, ch: u32, protocol: u32) -> Result<(), String> {
        for flags in [0, CAN_29BIT_ID] {
            let (mask, pattern) = (msg(protocol, flags, &[0; 4]), msg(protocol, flags, &[0; 4]));
            let mut id = 0u32;
            self.check("PASS_FILTER", unsafe {
                (self.start_filter)(
                    ch,
                    PASS_FILTER,
                    &*mask,
                    &*pattern,
                    std::ptr::null(),
                    &mut id,
                )
            })?;
        }
        Ok(())
    }

    fn flow_filter(&self, ch: u32, protocol: u32, o: &Opts) -> Result<(), String> {
        let id_flags = if o.ext { CAN_29BIT_ID } else { 0 };
        let mask = msg(protocol, id_flags, &[0xFF; 4]);
        let pattern = msg(protocol, id_flags, &o.resp.to_be_bytes());
        let flow = msg(
            protocol,
            id_flags
                | if o.pad.is_some() {
                    ISO15765_FRAME_PAD
                } else {
                    0
                },
            &o.req.to_be_bytes(),
        );
        let mut id = 0u32;
        self.check("FLOW_CONTROL_FILTER", unsafe {
            (self.start_filter)(ch, FLOW_CONTROL_FILTER, &*mask, &*pattern, &*flow, &mut id)
        })
    }

    /// One message, waiting up to `timeout`. Returns the status code and the message if any.
    fn read_one(&self, ch: u32, timeout: Duration) -> (i32, Option<Box<PassThruMsg>>) {
        let mut m = msg(0, 0, &[]);
        let mut n = 1u32;
        let rc = unsafe { (self.read)(ch, &mut *m, &mut n, timeout.as_millis() as u32) };
        (rc, (n == 1).then_some(m))
    }

    fn write_msg(&self, ch: u32, m: &PassThruMsg, timeout_ms: u32) -> i32 {
        let mut n = 1u32;
        unsafe { (self.write)(ch, m, &mut n, timeout_ms) }
    }
}

struct Opts {
    dll: String,
    nominal: u32,
    data: u32,
    minutes: u64,
    count: u32,
    req: u32,
    resp: u32,
    ext: bool,
    pad: Option<u8>,
    classic: bool,
    dl: u32,
    payload: Vec<u8>,
    frames: u32,
    nobrs: bool,
}

fn parse(args: &[String]) -> Opts {
    let mut o = Opts {
        dll: "target/x86_64-pc-windows-msvc/release/gs_usb_j2534.dll".into(),
        nominal: 500_000,
        data: 2_000_000,
        minutes: 10,
        count: 20,
        req: 0x18DA10F1,
        resp: 0x18DAF110,
        ext: true,
        pad: Some(0xAA),
        classic: false,
        dl: 8,
        payload: vec![],
        frames: 40,
        nobrs: false,
    };
    let mut i = 0;
    while i < args.len() {
        let next = |i: &mut usize| {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--dll" => o.dll = next(&mut i),
            "--nominal" => o.nominal = next(&mut i).parse().unwrap(),
            "--data" => o.data = next(&mut i).parse().unwrap(),
            "--minutes" => o.minutes = next(&mut i).parse().unwrap(),
            "--count" => o.count = next(&mut i).parse().unwrap(),
            "--req" => o.req = u32::from_str_radix(&next(&mut i), 16).unwrap(),
            "--resp" => o.resp = u32::from_str_radix(&next(&mut i), 16).unwrap(),
            "--ext" => o.ext = true,
            "--std" => o.ext = false,
            "--pad" => o.pad = u8::from_str_radix(&next(&mut i), 16).ok(),
            "--nopad" => o.pad = None,
            "--classic" => o.classic = true,
            "--frames" => o.frames = next(&mut i).parse().unwrap(),
            "--nobrs" => o.nobrs = true,
            "--dl" => o.dl = next(&mut i).parse().unwrap(),
            "--payload" => {
                let s = next(&mut i);
                o.payload = (0..s.len() / 2)
                    .map(|k| u8::from_str_radix(&s[2 * k..2 * k + 2], 16).unwrap())
                    .collect();
            }
            other => eprintln!("ignoring unknown argument {other}"),
        }
        i += 1;
    }
    o
}

fn iso_channel(
    api: &Api,
    dev: u32,
    o: &Opts,
    set_data_rate: bool,
) -> Result<(u32, u32, u32), String> {
    let proto = if o.classic { ISO15765 } else { FD_ISO15765_PS };
    let ch = api.connect_channel(dev, proto, o.nominal)?;
    let mut cfg = vec![];
    if !o.classic {
        if set_data_rate {
            cfg.push((FD_CAN_DATA_PHASE_RATE, o.data));
        }
        cfg.push((FD_ISO15765_TX_DATA_LENGTH, o.dl));
    }
    if let Some(p) = o.pad {
        cfg.push((ISO15765_PAD_VALUE, p as u32));
    }
    api.set_config(ch, &cfg)?;
    api.flow_filter(ch, proto, o)?;
    let tx_flags = (if o.ext { CAN_29BIT_ID } else { 0 })
        | if o.pad.is_some() {
            ISO15765_FRAME_PAD
        } else {
            0
        }
        | if o.classic { 0 } else { FD_CAN_FORMAT };
    Ok((ch, proto, tx_flags))
}

fn raw_channel(api: &Api, dev: u32, o: &Opts) -> Result<u32, String> {
    let ch = api.connect_channel(dev, FD_CAN_PS, o.nominal)?;
    api.set_config(ch, &[(FD_CAN_DATA_PHASE_RATE, o.data)])?;
    api.pass_all(ch, FD_CAN_PS)?;
    Ok(ch)
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args.first().cloned().unwrap_or_default();
    let o = parse(&args[1.min(args.len())..]);
    let api = Api::load(&o.dll)?;
    match mode.as_str() {
        "uds" => uds(&api, &o),
        "soak" => soak(&api, &o),
        "cycle" => cycle(&api, &o),
        "unplug" => unplug(&api, &o),
        "brs" => brs(&api, &o),
        "burst" => burst(&api, &o),
        _ => Err("usage: stress uds|soak|cycle|unplug [options]".into()),
    }
}

fn nrc_name(code: u8) -> &'static str {
    match code {
        0x10 => "generalReject",
        0x11 => "serviceNotSupported",
        0x12 => "subFunctionNotSupported",
        0x13 => "incorrectMessageLengthOrInvalidFormat",
        0x22 => "conditionsNotCorrect",
        0x31 => "requestOutOfRange",
        0x33 => "securityAccessDenied",
        0x78 => "requestCorrectlyReceived-ResponsePending",
        0x7F => "serviceNotSupportedInActiveSession",
        _ => "?",
    }
}

fn uds(api: &Api, o: &Opts) -> Result<(), String> {
    if o.payload.is_empty() {
        return Err("--payload required".into());
    }
    let dev = api
        .open_device()
        .map_err(|e| format!("open: 0x{:X} {}", e.0, e.1))?;
    let result = (|| {
        let (ch, proto, tx_flags) = iso_channel(api, dev, o, true)?;
        api.set_config(ch, &[(LOOPBACK, 1)])?;
        let mut data = o.req.to_be_bytes().to_vec();
        data.extend_from_slice(&o.payload);
        println!("TX {:X} <- {}", o.req, hex(&o.payload));
        let t0 = Instant::now();
        api.check(
            "PassThruWriteMsgs",
            api.write_msg(ch, &msg(proto, tx_flags, &data), 2000),
        )?;
        println!("  write completed after {:?}", t0.elapsed());
        let deadline = Instant::now() + Duration::from_secs(6);
        let mut done = false;
        while !done && Instant::now() < deadline {
            let (rc, m) = api.read_one(ch, Duration::from_millis(200));
            let Some(m) = m else {
                if rc != ERR_BUFFER_EMPTY && rc != ERR_TIMEOUT {
                    return Err(format!("read rc=0x{rc:X} {}", api.text()));
                }
                continue;
            };
            let d = &m.data[..m.data_size as usize];
            let id = u32::from_be_bytes(d[..4].try_into().unwrap());
            let at = t0.elapsed();
            let kind = if m.rx_status & TX_MSG_TYPE != 0 {
                "tx-indication"
            } else if m.rx_status & START_OF_MESSAGE != 0 {
                "first-frame-indication"
            } else {
                "message"
            };
            println!(
                "  +{:>7.1?} {kind:<22} id={id:08X} status=0x{:X} len={} {}",
                at,
                m.rx_status,
                d.len() - 4,
                hex(&d[4..])
            );
            if kind == "message" {
                let p = &d[4..];
                if p.len() >= 3 && p[0] == 0x7F {
                    println!(
                        "      negative response to {:02X}: NRC {:02X} {}",
                        p[1],
                        p[2],
                        nrc_name(p[2])
                    );
                    done = p[2] != 0x78;
                } else {
                    let text: String = p
                        .iter()
                        .map(|&b| {
                            if (0x20..0x7F).contains(&b) {
                                b as char
                            } else {
                                '.'
                            }
                        })
                        .collect();
                    println!("      positive response, {} bytes, ascii: {text}", p.len());
                    done = true;
                }
            }
        }
        if !done {
            return Err("no complete response within 6 s".into());
        }
        Ok(())
    })();
    unsafe { (api.close)(dev) };
    result
}

#[derive(Default)]
struct IdStat {
    count: u64,
    last_ts: u32,
    sum_gap: u64,
    max_gap: u32,
    gaps: Vec<u32>,
}

fn soak(api: &Api, o: &Opts) -> Result<(), String> {
    let dev = api
        .open_device()
        .map_err(|e| format!("open: 0x{:X} {}", e.0, e.1))?;
    let raw = raw_channel(api, dev, o)?;
    let (iso, proto, tx_flags) = iso_channel(api, dev, o, false)?;
    let stop = AtomicBool::new(false);
    let started = Instant::now();
    let duration = Duration::from_secs(o.minutes * 60);
    println!(
        "soak for {} min: raw capture + TesterPresent 2 Hz to {:X}",
        o.minutes, o.req
    );

    let (stats, rc_hist, overflow_flags, total) = std::thread::scope(|s| {
        let capture = s.spawn(|| {
            let mut stats: HashMap<(u32, bool), IdStat> = HashMap::new();
            let mut rc_hist: BTreeMap<i32, u64> = BTreeMap::new();
            let (mut overflow, mut total) = (0u64, 0u64);
            while !stop.load(Ordering::Relaxed) {
                let (rc, m) = api.read_one(raw, Duration::from_millis(50));
                *rc_hist.entry(rc).or_default() += 1;
                if let Some(m) = m {
                    total += 1;
                    overflow += (m.rx_status & BUFFER_OVERFLOW_FLAG != 0) as u64;
                    let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
                    let st = stats
                        .entry((id, m.rx_status & CAN_29BIT_ID != 0))
                        .or_default();
                    if st.count > 0 {
                        let gap = m.timestamp.wrapping_sub(st.last_ts);
                        st.sum_gap += gap as u64;
                        st.max_gap = st.max_gap.max(gap);
                        st.gaps.push(gap);
                    }
                    st.count += 1;
                    st.last_ts = m.timestamp;
                }
            }
            (stats, rc_hist, overflow, total)
        });

        let (mut sent, mut ok, mut missing, mut write_err) = (0u32, 0u32, 0u32, 0u32);
        let mut latencies: Vec<Duration> = vec![];
        let mut next_report = Instant::now() + Duration::from_secs(60);
        let mut data = o.req.to_be_bytes().to_vec();
        data.extend_from_slice(&[0x3E, 0x00]);
        let tp = msg(proto, tx_flags, &data);
        while started.elapsed() < duration {
            let tick = Instant::now();
            sent += 1;
            let rc = api.write_msg(iso, &tp, 1000);
            if rc != 0 {
                write_err += 1;
                println!("  write rc=0x{rc:X} {}", api.text());
            } else {
                let t0 = Instant::now();
                let mut got = false;
                while t0.elapsed() < Duration::from_millis(400) && !got {
                    if let (_, Some(m)) = api.read_one(iso, Duration::from_millis(50)) {
                        let d = &m.data[..m.data_size as usize];
                        if d.len() >= 6
                            && d[4] == 0x7E
                            && m.rx_status & (TX_MSG_TYPE | START_OF_MESSAGE) == 0
                        {
                            got = true;
                            latencies.push(t0.elapsed());
                        }
                    }
                }
                if got {
                    ok += 1
                } else {
                    missing += 1;
                    println!("  no TesterPresent response (#{sent})")
                }
            }
            if Instant::now() >= next_report {
                next_report += Duration::from_secs(60);
                println!(
                    "  [{:>4}s] TP sent={sent} ok={ok} missing={missing} write_err={write_err}",
                    started.elapsed().as_secs()
                );
            }
            std::thread::sleep(Duration::from_millis(500).saturating_sub(tick.elapsed()));
        }
        stop.store(true, Ordering::Relaxed);
        let r = capture.join().unwrap();
        latencies.sort();
        let pct = |p: f64| {
            latencies
                .get(((latencies.len() as f64 - 1.0) * p) as usize)
                .copied()
                .unwrap_or_default()
        };
        println!(
            "TesterPresent: sent={sent} ok={ok} missing={missing} write_err={write_err}; latency p50={:?} p95={:?} max={:?}",
            pct(0.5), pct(0.95), latencies.last().copied().unwrap_or_default()
        );
        r
    });

    println!("raw capture: {total} frames in {:?} ({:.0}/s), BUFFER_OVERFLOW-flagged frames: {overflow_flags}", started.elapsed(), total as f64 / started.elapsed().as_secs_f64());
    println!("ReadMsgs status histogram: {rc_hist:X?}");
    println!("cyclic ids (count > 500, mean gap < 100 ms); a gap above 3x the mean hints at lost frames:");
    let mut suspicious = 0;
    let mut rows: Vec<_> = stats.iter().filter(|(_, s)| s.count > 500).collect();
    rows.sort_by_key(|(k, _)| **k);
    for ((id, ext), s) in rows {
        let mean = s.sum_gap as f64 / (s.count - 1) as f64;
        if mean > 100_000.0 {
            continue;
        }
        let over = s.gaps.iter().filter(|&&g| g as f64 > 3.0 * mean).count();
        suspicious += (over > 0) as u32;
        println!(
            "  {id:08X}{} n={:<6} mean={:>8.1}us max={:>8}us gaps>3x={over}",
            if *ext { "x" } else { " " },
            s.count,
            mean,
            s.max_gap
        );
    }
    println!("ids with suspicious gaps: {suspicious}");
    unsafe { (api.disconnect)(iso) };
    unsafe { (api.disconnect)(raw) };
    unsafe { (api.close)(dev) };
    Ok(())
}

fn cycle(api: &Api, o: &Opts) -> Result<(), String> {
    let mut failures = 0;
    for i in 0..o.count {
        let t0 = Instant::now();
        let dev = match api.open_device() {
            Ok(d) => d,
            Err((rc, t)) => {
                println!("cycle {i:>2}: open failed 0x{rc:X} {t}");
                failures += 1;
                continue;
            }
        };
        let r = (|| -> Result<(usize, Duration), String> {
            let ch = raw_channel(api, dev, o)?;
            let t1 = Instant::now();
            let (mut n, mut first) = (0usize, None);
            let end = Instant::now() + Duration::from_millis(400);
            while Instant::now() < end {
                if api.read_one(ch, Duration::from_millis(50)).1.is_some() {
                    n += 1;
                    first.get_or_insert(t1.elapsed());
                }
            }
            if i % 2 == 0 {
                api.check("PassThruDisconnect", unsafe { (api.disconnect)(ch) })?;
            } // odd cycles: PassThruClose alone must release the channel
            Ok((n, first.unwrap_or_default()))
        })();
        let closed = unsafe { (api.close)(dev) };
        match r {
            Ok((n, first)) if n > 0 && closed == 0 => println!(
                "cycle {i:>2}: {n:>4} frames, first after {first:>7.1?}, {} , total {:?}",
                if i % 2 == 0 {
                    "disconnect+close"
                } else {
                    "close only    "
                },
                t0.elapsed()
            ),
            other => {
                failures += 1;
                println!("cycle {i:>2}: FAILED {other:?} close=0x{closed:X}");
            }
        }
    }
    println!("{} cycles, {failures} failures", o.count);
    if failures == 0 {
        Ok(())
    } else {
        Err(format!("{failures} failed cycles"))
    }
}

fn unplug(api: &Api, o: &Opts) -> Result<(), String> {
    let dev = api
        .open_device()
        .map_err(|e| format!("open: 0x{:X} {}", e.0, e.1))?;
    let ch = raw_channel(api, dev, o)?;
    println!("capturing; UNPLUG the adapter now, then plug it back in (waiting up to 3 minutes)");
    let started = Instant::now();
    let mut per_second = 0u32;
    let mut tick = Instant::now() + Duration::from_secs(1);
    let mut lost_at = None;
    while started.elapsed() < Duration::from_secs(180) {
        let (rc, m) = api.read_one(ch, Duration::from_millis(100));
        per_second += m.is_some() as u32;
        if rc == ERR_DEVICE_NOT_CONNECTED {
            println!(
                "[{:>5.1?}] ReadMsgs -> ERR_DEVICE_NOT_CONNECTED ({})",
                started.elapsed(),
                api.text()
            );
            lost_at = Some(Instant::now());
            break;
        }
        if Instant::now() >= tick {
            println!("[{:>5.1?}] {per_second} frames/s", started.elapsed());
            per_second = 0;
            tick += Duration::from_secs(1);
        }
    }
    let Some(lost) = lost_at else {
        unsafe { (api.close)(dev) };
        return Err("adapter was never reported lost".into());
    };
    // Writing must fail cleanly too.
    let rc = api.write_msg(
        ch,
        &msg(
            FD_CAN_PS,
            FD_CAN_FORMAT | CAN_29BIT_ID,
            &[0x18, 0xDA, 0x10, 0xF1, 1],
        ),
        100,
    );
    println!("WriteMsgs on the lost adapter -> 0x{rc:X} ({})", api.text());
    println!(
        "Disconnect -> 0x{:X}, Close -> 0x{:X}",
        unsafe { (api.disconnect)(ch) },
        unsafe { (api.close)(dev) }
    );
    println!("waiting for the adapter to come back...");
    loop {
        if lost.elapsed() > Duration::from_secs(150) {
            return Err("adapter did not come back".into());
        }
        match api.open_device() {
            Ok(dev) => {
                let ch = raw_channel(api, dev, o);
                let n = ch.as_ref().map(|&ch| {
                    let end = Instant::now() + Duration::from_millis(500);
                    let mut n = 0;
                    while Instant::now() < end {
                        n += api.read_one(ch, Duration::from_millis(50)).1.is_some() as u32;
                    }
                    n
                });
                println!(
                    "reopened after {:?}: {n:?} frames in 500 ms",
                    lost.elapsed()
                );
                unsafe { (api.close)(dev) };
                return match n {
                    Ok(n) if n > 0 => Ok(()),
                    _ => Err("no traffic after reopening".into()),
                };
            }
            Err(_) => std::thread::sleep(Duration::from_millis(500)),
        }
    }
}

/// Replays a burst of benign `22 F1 xx` reads (the shape of a typical identification scan) as raw FD
/// frames, 10 ms apart, and reports how many were really transmitted. Useful to compare data-phase
/// timing settings (`GSUSB_J2534_DATA_SAMPLE`) or `GSUSB_J2534_TX_BRS=0` on a given bus.
fn brs(api: &Api, o: &Opts) -> Result<(), String> {
    let dev = api
        .open_device()
        .map_err(|e| format!("open: 0x{:X} {}", e.0, e.1))?;
    let result = (|| {
        let ch = raw_channel(api, dev, o)?;
        api.set_config(ch, &[(LOOPBACK, 1)])?;
        let flags = FD_CAN_FORMAT
            | if o.nobrs { 0 } else { FD_CAN_BRS }
            | if o.ext { CAN_29BIT_ID } else { 0 };
        let (mut sent, mut echoes, mut answers, mut lost_link) = (0u32, 0u32, 0u32, false);
        for i in 0..o.frames {
            let mut data = o.req.to_be_bytes().to_vec();
            data.extend_from_slice(&[
                0x03,
                0x22,
                0xF1,
                0x80 + (i % 0x40) as u8,
                0x55,
                0x55,
                0x55,
                0x55,
            ]);
            let rc = api.write_msg(ch, &msg(FD_CAN_PS, flags, &data), 200);
            if rc != 0 {
                println!("  frame {i}: write rc=0x{rc:X} ({})", api.text());
                if rc == ERR_DEVICE_NOT_CONNECTED {
                    lost_link = true;
                    break;
                }
                continue;
            }
            sent += 1;
            let until = Instant::now() + Duration::from_millis(10);
            while Instant::now() < until {
                let (rc, m) = api.read_one(ch, Duration::from_millis(2));
                if rc == ERR_DEVICE_NOT_CONNECTED {
                    lost_link = true;
                }
                if let Some(m) = m {
                    let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
                    if m.rx_status & TX_MSG_TYPE != 0 {
                        echoes += 1;
                    } else if id == o.resp {
                        answers += 1;
                    }
                }
            }
            if lost_link {
                break;
            }
        }
        // collect stragglers
        let end = Instant::now() + Duration::from_millis(1200);
        while Instant::now() < end {
            if let (_, Some(m)) = api.read_one(ch, Duration::from_millis(20)) {
                let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
                if m.rx_status & TX_MSG_TYPE != 0 {
                    echoes += 1;
                } else if id == o.resp {
                    answers += 1;
                }
            }
        }
        println!(
            "BRS {}: sent {sent}, transmitted (echo) {echoes}, ECU answered {answers}{}",
            if o.nobrs { "off" } else { "on " },
            if lost_link {
                "  -> adapter reported lost (repeated bus-off)"
            } else {
                ""
            }
        );
        Ok(())
    })();
    unsafe { (api.close)(dev) };
    result
}

/// A transfer-like load: `--count` bursts of 65 frames x 64 bytes, written back to back with
/// timeout 0 like an application that does not retry refused frames. Frames go to an id that no
/// node listens to. Every accepted frame must be confirmed (loopback) in submission order.
fn burst(api: &Api, o: &Opts) -> Result<(), String> {
    const ID: u32 = 0x1CFF_FF76;
    let dev = api
        .open_device()
        .map_err(|e| format!("open: 0x{:X} {}", e.0, e.1))?;
    let result = (|| {
        let ch = raw_channel(api, dev, o)?;
        api.set_config(ch, &[(LOOPBACK, 1)])?;
        let flags = FD_CAN_FORMAT | CAN_29BIT_ID;
        let (mut written, mut refused, mut other) = (0u32, 0u32, 0u32);
        let (mut echoed, mut out_of_order) = (0u32, 0u32);
        let mut next_expected: u16 = 0;
        let started = Instant::now();
        let mut longest_burst = Duration::ZERO;
        let check = |m: &PassThruMsg, echoed: &mut u32, out_of_order: &mut u32, next: &mut u16| {
            if m.rx_status & TX_MSG_TYPE != 0 && m.data_size >= 6 && m.data[..4] == ID.to_be_bytes()
            {
                let seq = u16::from_be_bytes([m.data[4], m.data[5]]);
                *echoed += 1;
                if seq != *next {
                    *out_of_order += 1;
                }
                *next = seq.wrapping_add(1);
            }
        };
        let mut seq: u16 = 0;
        for _ in 0..o.count {
            let t0 = Instant::now();
            for _ in 0..65 {
                let mut data = ID.to_be_bytes().to_vec();
                data.extend_from_slice(&seq.to_be_bytes());
                data.extend(std::iter::repeat(0x55).take(62));
                match api.write_msg(ch, &msg(FD_CAN_PS, flags, &data), 0) {
                    0 => {
                        written += 1;
                        seq = seq.wrapping_add(1);
                    }
                    ERR_BUFFER_FULL => refused += 1,
                    rc => {
                        other += 1;
                        println!("  write rc=0x{rc:X} ({})", api.text());
                    }
                }
            }
            longest_burst = longest_burst.max(t0.elapsed());
            // drain confirmations between bursts, like an application waiting for the answer
            let end = Instant::now() + Duration::from_millis(40);
            while Instant::now() < end {
                if let (_, Some(m)) = api.read_one(ch, Duration::from_millis(5)) {
                    check(&m, &mut echoed, &mut out_of_order, &mut next_expected);
                }
            }
        }
        let end = Instant::now() + Duration::from_secs(20);
        while echoed < written && Instant::now() < end {
            if let (_, Some(m)) = api.read_one(ch, Duration::from_millis(20)) {
                check(&m, &mut echoed, &mut out_of_order, &mut next_expected);
            }
        }
        println!(
            "{} bursts of 65 frames in {:?} (longest submit loop {:?}): written {written}, refused {refused}, other errors {other}; confirmed {echoed}, out of order {out_of_order}",
            o.count,
            started.elapsed(),
            longest_burst
        );
        if refused > 0 || other > 0 || echoed != written || out_of_order > 0 {
            return Err("burst test failed".into());
        }
        Ok(())
    })();
    unsafe { (api.close)(dev) };
    result
}
