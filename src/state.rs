//! Process-wide device/channel registry and the logic behind Open/Connect/Ioctl.

use crate::bus::Bus;
use crate::channel::{err, ApiErr, ApiResult, Channel, Config};
use crate::gsusb::{CanAdapter, GsUsb, StartConfig, UsbError};
use crate::j2534::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

pub struct Device {
    pub id: u32,
    pub usb: Arc<GsUsb>,
    bus: Mutex<Option<Arc<Bus>>>,
    channels: Mutex<Vec<u32>>,
}

#[derive(Default)]
struct Global {
    next_device: u32,
    next_channel: u32,
    devices: HashMap<u32, Arc<Device>>,
    channels: HashMap<u32, (u32, Arc<Channel>)>,
}

fn global() -> &'static Mutex<Global> {
    static G: OnceLock<Mutex<Global>> = OnceLock::new();
    G.get_or_init(|| {
        Mutex::new(Global {
            next_device: 1,
            next_channel: 1,
            ..Default::default()
        })
    })
}

static LAST_ERROR: Mutex<String> = Mutex::new(String::new());

pub fn set_last_error(msg: impl Into<String>) {
    *lock(&LAST_ERROR) = msg.into();
}

pub fn last_error() -> String {
    lock(&LAST_ERROR).clone()
}

/// `GSUSB_J2534_LISTEN_ONLY=1` starts the controller in listen-only mode (no ACK, no
/// transmit): safe bit-rate probing on a live vehicle bus.
fn listen_only_requested() -> bool {
    std::env::var("GSUSB_J2534_LISTEN_ONLY").is_ok_and(|v| !v.is_empty() && v != "0")
}

pub fn map_usb(e: UsbError) -> ApiErr {
    let (code, m) = match e {
        UsbError::BaudRate(m) => (ERR_INVALID_BAUDRATE, m),
        UsbError::Unsupported(m) => (ERR_NOT_SUPPORTED, m),
        UsbError::NotFound(m) => (ERR_DEVICE_NOT_CONNECTED, m),
        UsbError::Busy(m) => (ERR_DEVICE_IN_USE, m),
        UsbError::Io(m) => (ERR_DEVICE_NOT_CONNECTED, m),
    };
    ApiErr(code, m)
}

pub fn open_device(name: Option<&str>) -> ApiResult<u32> {
    // Only `gs_usb:VID:PID:SERIAL` selects a specific adapter; other names (for example the
    // `J2534-2:...` prefix some tools pass) fall back to the first adapter.
    let selector = name.filter(|n| n.starts_with("gs_usb:"));
    let mut g = lock(global());
    if let Some(sel) = selector {
        if g.devices.values().any(|d| d.usb.info.selector == sel) {
            return err(ERR_DEVICE_IN_USE, "device already open");
        }
    }
    let usb = Arc::new(GsUsb::open(selector).map_err(map_usb)?);
    if g.devices
        .values()
        .any(|d| d.usb.info.selector == usb.info.selector)
    {
        return err(ERR_DEVICE_IN_USE, "device already open");
    }
    let id = g.next_device;
    g.next_device += 1;
    g.devices.insert(
        id,
        Arc::new(Device {
            id,
            usb,
            bus: Mutex::new(None),
            channels: Mutex::new(Vec::new()),
        }),
    );
    Ok(id)
}

pub fn device(id: u32) -> ApiResult<Arc<Device>> {
    lock(global())
        .devices
        .get(&id)
        .cloned()
        .ok_or(ApiErr(ERR_INVALID_DEVICE_ID, "invalid device id".into()))
}

pub fn channel(id: u32) -> ApiResult<Arc<Channel>> {
    lock(global())
        .channels
        .get(&id)
        .map(|(_, c)| Arc::clone(c))
        .ok_or(ApiErr(ERR_INVALID_CHANNEL_ID, "invalid channel id".into()))
}

fn device_of_channel(id: u32) -> ApiResult<Arc<Device>> {
    let g = lock(global());
    let (dev, _) = g
        .channels
        .get(&id)
        .ok_or(ApiErr(ERR_INVALID_CHANNEL_ID, "invalid channel id".into()))?;
    g.devices
        .get(dev)
        .cloned()
        .ok_or(ApiErr(ERR_INVALID_DEVICE_ID, "device closed".into()))
}

pub fn close_device(id: u32) -> ApiResult<()> {
    let dev = device(id)?;
    let ids: Vec<u32> = lock(&dev.channels).clone();
    for c in ids {
        let _ = disconnect(c);
    }
    lock(global()).devices.remove(&id);
    Ok(())
}

/// Default data phase rate when the application does not configure one.
/// 2 Mbit/s: the highest rate verified on candleLightFD hardware (4 Mbit/s is not).
pub fn default_data_rate(nominal: u32) -> u32 {
    nominal.max(2_000_000)
}

pub fn connect(device_id: u32, protocol: u32, flags: u32, baud: u32) -> ApiResult<u32> {
    let dev = device(device_id)?;
    if !is_supported_protocol(protocol) {
        return err(
            ERR_INVALID_PROTOCOL_ID,
            format!("protocol 0x{protocol:X} is not supported"),
        );
    }
    let fd = is_fd_protocol(protocol);
    if fd && !dev.usb.supports_fd() {
        return err(
            ERR_INVALID_PROTOCOL_ID,
            "this adapter does not advertise CAN FD",
        );
    }
    if flags & !(CAN_29BIT_ID | CAN_ID_BOTH | ISO9141_NO_CHECKSUM) != 0 {
        return err(ERR_INVALID_FLAGS, "unsupported connect flags");
    }
    if baud == 0 {
        return err(ERR_INVALID_BAUDRATE, "baud rate must be non-zero");
    }
    let mut bus_slot = lock(&dev.bus);
    let bus = match bus_slot.as_ref() {
        None => {
            let cfg = StartConfig {
                nominal: baud,
                // An FD-capable controller always runs in FD mode, also for classic channels: a
                // classic-only controller cannot decode FD frames on an FD bus and floods it with
                // error frames. Classic frames are unaffected by FD mode.
                data: dev.usb.supports_fd().then(|| default_data_rate(baud)),
                listen_only: listen_only_requested(),
            };
            let adapter: Arc<dyn CanAdapter> = dev.usb.clone();
            let bus = Bus::start(adapter, cfg).map_err(map_usb)?;
            *bus_slot = Some(Arc::clone(&bus));
            bus
        }
        Some(bus) => {
            let cur = bus.config();
            if cur.nominal != baud {
                return err(
                    ERR_CHANNEL_IN_USE,
                    format!("bus already running at {} bit/s", cur.nominal),
                );
            }
            Arc::clone(bus)
        }
    };
    let mut g = lock(global());
    let id = g.next_channel;
    g.next_channel += 1;
    let ch = Channel::new(id, protocol, flags, Arc::clone(&bus), Config::default());
    bus.add_sink(id, ch.clone());
    g.channels.insert(id, (device_id, ch));
    lock(&dev.channels).push(id);
    Ok(id)
}

pub fn disconnect(channel_id: u32) -> ApiResult<()> {
    let dev = device_of_channel(channel_id)?;
    let ch = {
        let mut g = lock(global());
        g.channels
            .remove(&channel_id)
            .map(|(_, c)| c)
            .ok_or(ApiErr(ERR_INVALID_CHANNEL_ID, "invalid channel id".into()))?
    };
    // Lock order everywhere: device bus slot, then device channel list.
    let mut bus_slot = lock(&dev.bus);
    if let Some(bus) = bus_slot.as_ref() {
        bus.remove_sink(channel_id);
    }
    ch.shutdown();
    let mut channels = lock(&dev.channels);
    channels.retain(|c| *c != channel_id);
    if channels.is_empty() {
        if let Some(bus) = bus_slot.take() {
            bus.shutdown();
        }
    }
    Ok(())
}

/// Shared helper for DATA_RATE / FD_CAN_DATA_PHASE_RATE changes: reprograms the whole bus.
fn reconfigure_bus(
    channel_id: u32,
    f: impl FnOnce(StartConfig) -> ApiResult<StartConfig>,
) -> ApiResult<()> {
    let dev = device_of_channel(channel_id)?;
    let bus = lock(&dev.bus)
        .clone()
        .ok_or(ApiErr(ERR_FAILED, "bus not running".into()))?;
    let cur = bus.config();
    let new = f(cur)?;
    if new.nominal == cur.nominal && new.data == cur.data {
        return Ok(());
    }
    if lock(&dev.channels).len() > 1 {
        return err(
            ERR_INVALID_IOCTL_VALUE,
            "bit rates cannot change while other channels share the adapter",
        );
    }
    bus.restart(new).map_err(|e| match map_usb(e) {
        ApiErr(ERR_INVALID_BAUDRATE, m) => ApiErr(ERR_INVALID_IOCTL_VALUE, m),
        other => other,
    })
}

pub fn get_config(channel_id: u32, parameter: u32) -> ApiResult<u32> {
    let ch = channel(channel_id)?;
    let cfg = ch.config();
    let bus_cfg = || -> ApiResult<StartConfig> {
        let dev = device_of_channel(channel_id)?;
        let b = lock(&dev.bus).clone();
        b.map(|b| b.config())
            .ok_or(ApiErr(ERR_FAILED, "bus not running".into()))
    };
    Ok(match parameter {
        DATA_RATE => bus_cfg()?.nominal,
        FD_CAN_DATA_PHASE_RATE if is_fd_protocol(ch.protocol) => bus_cfg()?.data.unwrap_or(0),
        LOOPBACK | ECHO_PHYSICAL_CHANNEL_TX => cfg.loopback as u32,
        ISO15765_BS => cfg.bs as u32,
        ISO15765_STMIN => cfg.stmin as u32,
        BS_TX => cfg.bs_tx,
        STMIN_TX => cfg.stmin_tx,
        ISO15765_WAIT_LIMIT => cfg.wait_limit,
        ISO15765_PAD_VALUE => cfg.pad_value as u32,
        N_AS_MAX => cfg.n_as,
        N_AR_MAX => cfg.n_ar,
        N_BS_MAX => cfg.n_bs,
        N_CR_MAX => cfg.n_cr,
        FD_ISO15765_TX_DATA_LENGTH if is_fd_protocol(ch.protocol) => cfg.tx_dl as u32,
        CAN_MIXED_FORMAT => cfg.can_mixed_format,
        J1962_PINS => cfg.j1962_pins,
        _ => {
            return err(
                ERR_INVALID_IOCTL_VALUE,
                format!("config parameter 0x{parameter:X} is not supported"),
            )
        }
    })
}

pub fn set_config(channel_id: u32, parameter: u32, value: u32) -> ApiResult<()> {
    let ch = channel(channel_id)?;
    let byte = |v: u32| -> ApiResult<u8> {
        u8::try_from(v).or(err(ERR_INVALID_IOCTL_VALUE, "value must be 0..255"))
    };
    match parameter {
        DATA_RATE => {
            return reconfigure_bus(channel_id, |c| {
                if value == 0 {
                    return err(ERR_INVALID_IOCTL_VALUE, "DATA_RATE must be non-zero");
                }
                // Keep an FD data rate that is still valid for the new nominal rate.
                let data = c.data.map(|d| {
                    if d < value {
                        default_data_rate(value)
                    } else {
                        d
                    }
                });
                Ok(StartConfig {
                    nominal: value,
                    data,
                    ..c
                })
            });
        }
        FD_CAN_DATA_PHASE_RATE if is_fd_protocol(ch.protocol) => {
            return reconfigure_bus(channel_id, |c| {
                if value < c.nominal {
                    return err(
                        ERR_INVALID_IOCTL_VALUE,
                        "data phase rate below nominal rate",
                    );
                }
                Ok(StartConfig {
                    data: Some(value),
                    ..c
                })
            });
        }
        LOOPBACK | ECHO_PHYSICAL_CHANNEL_TX => {
            if value > 1 {
                return err(ERR_INVALID_IOCTL_VALUE, "LOOPBACK must be 0 or 1");
            }
            ch.update_config(|c| c.loopback = value == 1);
        }
        ISO15765_BS => {
            let v = byte(value)?;
            ch.update_config(|c| c.bs = v);
        }
        ISO15765_STMIN => {
            let v = byte(value)?;
            ch.update_config(|c| c.stmin = v);
        }
        BS_TX => {
            if value > 0xFF && value != 0xFFFF {
                return err(ERR_INVALID_IOCTL_VALUE, "BS_TX must be 0..255 or 0xFFFF");
            }
            ch.update_config(|c| c.bs_tx = value);
        }
        STMIN_TX => {
            if value > 0xFF && value != 0xFFFF {
                return err(ERR_INVALID_IOCTL_VALUE, "STMIN_TX must be 0..255 or 0xFFFF");
            }
            ch.update_config(|c| c.stmin_tx = value);
        }
        ISO15765_WAIT_LIMIT => ch.update_config(|c| c.wait_limit = value),
        ISO15765_PAD_VALUE => {
            let v = byte(value)?;
            ch.update_config(|c| c.pad_value = v);
        }
        N_AS_MAX => ch.update_config(|c| c.n_as = value),
        N_AR_MAX => ch.update_config(|c| c.n_ar = value),
        N_BS_MAX => ch.update_config(|c| c.n_bs = value),
        N_CR_MAX => ch.update_config(|c| c.n_cr = value),
        FD_ISO15765_TX_DATA_LENGTH if is_fd_protocol(ch.protocol) => {
            if !matches!(value, 8 | 12 | 16 | 20 | 24 | 32 | 48 | 64) {
                return err(
                    ERR_INVALID_IOCTL_VALUE,
                    "TX data length must be a valid CAN FD length >= 8",
                );
            }
            ch.update_config(|c| c.tx_dl = value as usize);
        }
        CAN_MIXED_FORMAT => {
            if value > 2 {
                return err(ERR_INVALID_IOCTL_VALUE, "CAN_MIXED_FORMAT must be 0..2");
            }
            ch.update_config(|c| c.can_mixed_format = value);
        }
        J1962_PINS => ch.update_config(|c| c.j1962_pins = value),
        _ => {
            return err(
                ERR_INVALID_IOCTL_VALUE,
                format!("config parameter 0x{parameter:X} is not supported"),
            )
        }
    }
    Ok(())
}

pub fn device_info(device_or_channel: u32) -> ApiResult<Arc<Device>> {
    device(device_or_channel).or_else(|_| device_of_channel(device_or_channel))
}
