//! Exercises the built DLL exactly as a J2534 application would: LoadLibrary + exports.
//!
//!   cargo run --release --example probe -- scan                       # listen-only bit-rate scan
//!   cargo run --release --example probe -- listen --nominal 500000 --data 2000000 --secs 3
//!   cargo run --release --example probe -- tp  --nominal 500000 --data 2000000 \
//!         --req 18DA10F1 --resp 18DAF110 --ext --pad AA                 # ISO15765-FD TesterPresent
//!   cargo run --release --example probe -- raw --nominal ... --req 18DA10F1 --resp 18DAF110 --ext
//!
//! `scan` and `listen` run the controller listen-only (GSUSB_J2534_LISTEN_ONLY=1): nothing is
//! transmitted and no ACK is given, so they are safe on a live vehicle bus. `tp` and `raw`
//! transmit exactly one UDS TesterPresent (3E 00) request.

use gs_usb_j2534::j2534::*;
use libloading::{Library, Symbol};
use std::collections::HashMap;
use std::ffi::{c_char, c_void, CStr};
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
type ReadVersion = unsafe extern "system" fn(u32, *mut c_char, *mut c_char, *mut c_char) -> i32;

struct Api<'a> {
    open: Symbol<'a, Open>,
    close: Symbol<'a, Close>,
    connect: Symbol<'a, Connect>,
    disconnect: Symbol<'a, Disconnect>,
    read: Symbol<'a, Read>,
    write: Symbol<'a, Write>,
    start_filter: Symbol<'a, StartFilter>,
    ioctl: Symbol<'a, Ioctl>,
    last_error: Symbol<'a, LastError>,
    read_version: Symbol<'a, ReadVersion>,
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

impl Api<'_> {
    fn check(&self, what: &str, rc: i32) -> Result<(), String> {
        if rc == 0 {
            return Ok(());
        }
        let mut buf = [0 as c_char; 128];
        unsafe { (self.last_error)(buf.as_mut_ptr()) };
        let text = unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        Err(format!("{what} failed: 0x{rc:X} ({text})"))
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

    fn drain(&self, ch: u32, wait: Duration, mut each: impl FnMut(&PassThruMsg)) -> usize {
        let deadline = Instant::now() + wait;
        let mut total = 0;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            // One message per call: ReadMsgs waits the full timeout for a larger request.
            let mut buf: Vec<PassThruMsg> = vec![*msg(0, 0, &[])];
            let mut n = 1u32;
            let rc = unsafe {
                (self.read)(
                    ch,
                    buf.as_mut_ptr(),
                    &mut n,
                    left.as_millis().min(100) as u32,
                )
            };
            for m in &buf[..n as usize] {
                each(m);
                total += 1;
            }
            if (rc != 0 && rc != ERR_BUFFER_EMPTY && rc != ERR_TIMEOUT)
                || Instant::now() >= deadline
            {
                return total;
            }
        }
    }

    fn pass_all(&self, ch: u32, protocol: u32) -> Result<(), String> {
        for flags in [0, CAN_29BIT_ID] {
            let mask = msg(protocol, flags, &[0, 0, 0, 0]);
            let pattern = msg(protocol, flags, &[0, 0, 0, 0]);
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
}

struct Opts {
    dll: String,
    nominal: u32,
    data: u32,
    secs: u64,
    req: u32,
    resp: u32,
    ext: bool,
    pad: Option<u8>,
    classic: bool,
    dl: u32,
    brs: bool,
}

fn parse(args: &[String]) -> Opts {
    let mut o = Opts {
        dll: "target/x86_64-pc-windows-msvc/release/gs_usb_j2534.dll".into(),
        nominal: 500_000,
        data: 2_000_000,
        secs: 3,
        req: 0x18DA10F1,
        resp: 0x18DAF110,
        ext: true,
        pad: Some(0xAA),
        classic: false,
        dl: 8,
        brs: false,
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
            "--secs" => o.secs = next(&mut i).parse().unwrap(),
            "--req" => o.req = u32::from_str_radix(&next(&mut i), 16).unwrap(),
            "--resp" => o.resp = u32::from_str_radix(&next(&mut i), 16).unwrap(),
            "--ext" => o.ext = true,
            "--std" => o.ext = false,
            "--pad" => o.pad = u8::from_str_radix(&next(&mut i), 16).ok(),
            "--nopad" => o.pad = None,
            "--classic" => o.classic = true,
            "--dl" => o.dl = next(&mut i).parse().unwrap(),
            "--brs" => o.brs = true,
            other => eprintln!("ignoring unknown argument {other}"),
        }
        i += 1;
    }
    o
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mode = args.first().cloned().unwrap_or_else(|| "listen".into());
    let o = parse(&args[1.min(args.len())..]);
    if matches!(mode.as_str(), "scan" | "listen") {
        std::env::set_var("GSUSB_J2534_LISTEN_ONLY", "1");
    }
    let lib = unsafe { Library::new(&o.dll) }.map_err(|e| format!("load {}: {e}", o.dll))?;
    macro_rules! sym {
        ($name:literal) => {
            unsafe { lib.get($name) }
                .map_err(|e| format!("missing export {}: {e}", String::from_utf8_lossy($name)))?
        };
    }
    let api = Api {
        open: sym!(b"PassThruOpen"),
        close: sym!(b"PassThruClose"),
        connect: sym!(b"PassThruConnect"),
        disconnect: sym!(b"PassThruDisconnect"),
        read: sym!(b"PassThruReadMsgs"),
        write: sym!(b"PassThruWriteMsgs"),
        start_filter: sym!(b"PassThruStartMsgFilter"),
        ioctl: sym!(b"PassThruIoctl"),
        last_error: sym!(b"PassThruGetLastError"),
        read_version: sym!(b"PassThruReadVersion"),
    };
    // The v05.00 marker must be absent, otherwise tools pick the wrong ABI.
    if unsafe { lib.get::<*const ()>(b"PassThruScanForDevices") }.is_ok() {
        return Err("DLL exports PassThruScanForDevices (would be detected as v05.00)".into());
    }
    for name in [
        &b"PassThruStopMsgFilter"[..],
        b"PassThruStartPeriodicMsg",
        b"PassThruStopPeriodicMsg",
        b"PassThruSetProgrammingVoltage",
    ] {
        unsafe { lib.get::<*const ()>(name) }
            .map_err(|e| format!("missing export {}: {e}", String::from_utf8_lossy(name)))?;
    }

    let mut dev = 0u32;
    api.check("PassThruOpen", unsafe {
        (api.open)(std::ptr::null(), &mut dev)
    })?;
    let (mut fw, mut dll, mut apiv) = ([0 as c_char; 80], [0 as c_char; 80], [0 as c_char; 80]);
    api.check("PassThruReadVersion", unsafe {
        (api.read_version)(dev, fw.as_mut_ptr(), dll.as_mut_ptr(), apiv.as_mut_ptr())
    })?;
    let s = |b: &[c_char; 80]| {
        unsafe { CStr::from_ptr(b.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    };
    println!(
        "device {dev}: firmware `{}`, dll `{}`, api {}",
        s(&fw),
        s(&dll),
        s(&apiv)
    );

    let result = run(&api, dev, &mode, &o);
    unsafe { (api.close)(dev) };
    result
}

fn run(api: &Api, dev: u32, mode: &str, o: &Opts) -> Result<(), String> {
    match mode {
        "scan" => {
            println!("listen-only scan (nothing is transmitted):");
            for nominal in [500_000u32, 250_000, 1_000_000, 125_000] {
                // 4/5 Mbit/s are skipped: not accepted on the candleLightFD bench (see j2534-bridge).
                let mut rates = vec![nominal, 1_000_000, 2_000_000];
                rates.dedup();
                for data in rates {
                    if data < nominal {
                        continue;
                    }
                    let mut ch = 0u32;
                    let rc =
                        unsafe { (api.connect)(dev, FD_CAN_PS, CAN_ID_BOTH, nominal, &mut ch) };
                    if rc != 0 {
                        println!(
                            "  {nominal:>7}/{data:<8} connect: {}",
                            api.check("connect", rc).unwrap_err()
                        );
                        continue;
                    }
                    let r = (|| -> Result<(usize, HashMap<u32, usize>), String> {
                        api.set_config(ch, &[(FD_CAN_DATA_PHASE_RATE, data)])?;
                        api.pass_all(ch, FD_CAN_PS)?;
                        let mut ids: HashMap<u32, usize> = HashMap::new();
                        let n = api.drain(ch, Duration::from_millis(700), |m| {
                            *ids.entry(u32::from_be_bytes(m.data[..4].try_into().unwrap()))
                                .or_default() += 1;
                        });
                        Ok((n, ids))
                    })();
                    unsafe { (api.disconnect)(ch) };
                    match r {
                        Ok((n, ids)) => println!(
                            "  {nominal:>7}/{data:<8} {n:>5} frames, {} distinct ids",
                            ids.len()
                        ),
                        Err(e) => println!("  {nominal:>7}/{data:<8} {e}"),
                    }
                }
            }
            Ok(())
        }
        "listen" => {
            let mut ch = 0u32;
            api.check("connect", unsafe {
                (api.connect)(dev, FD_CAN_PS, CAN_ID_BOTH, o.nominal, &mut ch)
            })?;
            api.set_config(ch, &[(FD_CAN_DATA_PHASE_RATE, o.data)])?;
            api.pass_all(ch, FD_CAN_PS)?;
            let mut counts: HashMap<(u32, bool, bool), (usize, Vec<u8>)> = HashMap::new();
            let n = api.drain(ch, Duration::from_secs(o.secs), |m| {
                let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
                let e = counts
                    .entry((
                        id,
                        m.rx_status & CAN_29BIT_ID != 0,
                        m.rx_status & FD_CAN_FORMAT != 0,
                    ))
                    .or_default();
                e.0 += 1;
                e.1 = m.data[4..m.data_size as usize].to_vec();
            });
            println!("{n} frames in {}s, {} distinct ids", o.secs, counts.len());
            let mut v: Vec<_> = counts.into_iter().collect();
            v.sort_by_key(|(k, _)| *k);
            for ((id, ext, fd), (c, last)) in v {
                println!(
                    "  {id:08X} {} {} x{c:<5} last: {}",
                    if ext { "ext" } else { "std" },
                    if fd { "FD" } else { "  " },
                    hex(&last)
                );
            }
            unsafe { (api.disconnect)(ch) };
            Ok(())
        }
        "raw" => {
            let mut ch = 0u32;
            api.check("connect", unsafe {
                (api.connect)(dev, FD_CAN_PS, CAN_ID_BOTH, o.nominal, &mut ch)
            })?;
            api.set_config(ch, &[(FD_CAN_DATA_PHASE_RATE, o.data), (LOOPBACK, 1)])?;
            api.pass_all(ch, FD_CAN_PS)?;
            let payload = [0x02, 0x3E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];
            let mut data = o.req.to_be_bytes().to_vec();
            data.extend_from_slice(&payload);
            let flags = FD_CAN_FORMAT
                | if o.brs { FD_CAN_BRS } else { 0 }
                | if o.ext { CAN_29BIT_ID } else { 0 };
            let m = msg(FD_CAN_PS, flags, &data);
            let mut n = 1u32;
            println!("TX {:X} FD {}", o.req, hex(&payload));
            api.check("PassThruWriteMsgs", unsafe {
                (api.write)(ch, &*m, &mut n, 1000)
            })?;
            let mut got = false;
            api.drain(ch, Duration::from_millis(1000), |m| {
                let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
                let tx = m.rx_status & TX_MSG_TYPE != 0;
                println!(
                    "  {} {:08X} fd={} brs={} {}",
                    if tx { "echo" } else { "RX  " },
                    id,
                    m.rx_status & FD_CAN_FORMAT != 0,
                    m.rx_status & FD_CAN_BRS != 0,
                    hex(&m.data[4..m.data_size as usize])
                );
                got |= !tx && id == o.resp;
            });
            unsafe { (api.disconnect)(ch) };
            if got {
                println!("OK: ECU {:X} answered", o.resp);
                Ok(())
            } else {
                Err(format!("no response from {:X}", o.resp))
            }
        }
        "tp" => {
            let proto = if o.classic { ISO15765 } else { FD_ISO15765_PS };
            let mut ch = 0u32;
            api.check("connect", unsafe {
                (api.connect)(dev, proto, CAN_ID_BOTH, o.nominal, &mut ch)
            })?;
            let mut cfg = vec![(LOOPBACK, 1)];
            if !o.classic {
                cfg.push((FD_CAN_DATA_PHASE_RATE, o.data));
                cfg.push((FD_ISO15765_TX_DATA_LENGTH, o.dl));
            }
            if let Some(p) = o.pad {
                cfg.push((ISO15765_PAD_VALUE, p as u32));
            }
            api.set_config(ch, &cfg)?;
            let id_flags = if o.ext { CAN_29BIT_ID } else { 0 };
            let tx_flags = id_flags
                | if o.pad.is_some() {
                    ISO15765_FRAME_PAD
                } else {
                    0
                }
                | if o.classic { 0 } else { FD_CAN_FORMAT }
                | if o.brs { FD_CAN_BRS } else { 0 };
            let mask = msg(proto, id_flags, &[0xFF; 4]);
            let pattern = msg(proto, id_flags, &o.resp.to_be_bytes());
            let flow = msg(
                proto,
                tx_flags & !FD_CAN_FORMAT & !FD_CAN_BRS,
                &o.req.to_be_bytes(),
            );
            let mut fid = 0u32;
            api.check("FLOW_CONTROL_FILTER", unsafe {
                (api.start_filter)(ch, FLOW_CONTROL_FILTER, &*mask, &*pattern, &*flow, &mut fid)
            })?;
            let mut data = o.req.to_be_bytes().to_vec();
            data.extend_from_slice(&[0x3E, 0x00]);
            let m = msg(proto, tx_flags, &data);
            let mut n = 1u32;
            println!(
                "TX ISO-TP {:X} -> 3E 00 (pad {:02X?}, {})",
                o.req,
                o.pad,
                if o.classic { "classic" } else { "FD" }
            );
            let t0 = Instant::now();
            api.check("PassThruWriteMsgs", unsafe {
                (api.write)(ch, &*m, &mut n, 1000)
            })?;
            let mut response = None;
            let mut latency = Duration::ZERO;
            api.drain(ch, Duration::from_millis(1500), |m| {
                let id = u32::from_be_bytes(m.data[..4].try_into().unwrap());
                println!(
                    "  rx_status=0x{:X} id={:08X} data={}",
                    m.rx_status,
                    id,
                    hex(&m.data[4..m.data_size as usize])
                );
                if m.rx_status & (TX_MSG_TYPE | START_OF_MESSAGE) == 0 && id == o.resp {
                    response = Some(m.data[4..m.data_size as usize].to_vec());
                    latency = t0.elapsed();
                }
            });
            unsafe { (api.disconnect)(ch) };
            match response {
                Some(r) if r.first() == Some(&0x7E) => {
                    println!(
                        "OK: positive TesterPresent response {} after {:?}",
                        hex(&r),
                        latency
                    );
                    Ok(())
                }
                Some(r) => Err(format!("unexpected response {}", hex(&r))),
                None => Err(format!("no response from {:X}", o.resp)),
            }
        }
        other => Err(format!("unknown mode `{other}` (scan | listen | raw | tp)")),
    }
}
