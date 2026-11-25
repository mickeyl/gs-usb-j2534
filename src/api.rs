//! Exported SAE J2534-1 04.04 entry points (x64: a single calling convention, plain names).
//!
//! `PassThruScanForDevices` is deliberately *not* exported: j2534-bridge and other tools
//! treat that symbol as the marker of the incompatible v05.00 ABI.

#![allow(non_snake_case, clippy::missing_safety_doc)]

use crate::channel::{err, ApiErr, ApiResult};
use crate::j2534::*;
use crate::state;
use std::ffi::{c_char, c_void, CStr};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::Duration;

pub const DLL_VERSION: &str = concat!("gs-usb-j2534 ", env!("CARGO_PKG_VERSION"));
const API_VERSION: &str = "04.04";

/// Run `f`, converting errors and panics into a J2534 status and the last-error text.
fn guard(name: &str, f: impl FnOnce() -> ApiResult<()>) -> i32 {
    crate::trace!("{name}");
    let rc = match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => STATUS_NOERROR,
        Ok(Err(ApiErr(code, msg))) => {
            state::set_last_error(msg.clone());
            crate::trace!("  {name} -> 0x{code:X}: {msg}");
            code
        }
        Err(_) => {
            state::set_last_error("internal driver error");
            crate::trace!("  {name} -> panic");
            ERR_FAILED
        }
    };
    if rc == STATUS_NOERROR {
        crate::trace!("  {name} -> ok");
    }
    rc
}

/// Like [`guard`], but the body decides the status itself (partial results).
fn guard_rc(name: &str, f: impl FnOnce() -> (i32, Option<String>)) -> i32 {
    crate::trace!("{name}");
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok((rc, msg)) => {
            if let Some(m) = msg {
                state::set_last_error(m);
            }
            crate::trace!("  {name} -> 0x{rc:X}");
            rc
        }
        Err(_) => {
            state::set_last_error("internal driver error");
            ERR_FAILED
        }
    }
}

unsafe fn write_cstr(dst: *mut c_char, text: &str) {
    if dst.is_null() {
        return;
    }
    let bytes = text.as_bytes();
    let n = bytes.len().min(79);
    std::ptr::copy_nonoverlapping(bytes.as_ptr(), dst as *mut u8, n);
    *dst.add(n) = 0;
}

fn nonnull<T>(p: *const T, what: &str) -> ApiResult<()> {
    if p.is_null() {
        err(ERR_NULL_PARAMETER, format!("{what} is NULL"))
    } else {
        Ok(())
    }
}

#[no_mangle]
pub unsafe extern "system" fn PassThruOpen(p_name: *const c_void, p_device_id: *mut u32) -> i32 {
    guard("PassThruOpen", || {
        nonnull(p_device_id, "pDeviceID")?;
        let name = if p_name.is_null() {
            None
        } else {
            CStr::from_ptr(p_name as *const c_char)
                .to_str()
                .ok()
                .map(str::to_owned)
        };
        crate::trace!("  name={name:?}");
        *p_device_id = state::open_device(name.as_deref())?;
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruClose(device_id: u32) -> i32 {
    guard("PassThruClose", || state::close_device(device_id))
}

#[no_mangle]
pub unsafe extern "system" fn PassThruConnect(
    device_id: u32,
    protocol_id: u32,
    flags: u32,
    baud_rate: u32,
    p_channel_id: *mut u32,
) -> i32 {
    guard("PassThruConnect", || {
        nonnull(p_channel_id, "pChannelID")?;
        crate::trace!(
            "  device={device_id} protocol=0x{protocol_id:X} flags=0x{flags:X} baud={baud_rate}"
        );
        *p_channel_id = state::connect(device_id, protocol_id, flags, baud_rate)?;
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruDisconnect(channel_id: u32) -> i32 {
    guard("PassThruDisconnect", || state::disconnect(channel_id))
}

#[no_mangle]
pub unsafe extern "system" fn PassThruReadMsgs(
    channel_id: u32,
    p_msg: *mut PassThruMsg,
    p_num_msgs: *mut u32,
    timeout: u32,
) -> i32 {
    guard_rc("PassThruReadMsgs", || {
        if p_msg.is_null() || p_num_msgs.is_null() {
            return (ERR_NULL_PARAMETER, Some("pMsg or pNumMsgs is NULL".into()));
        }
        let ch = match state::channel(channel_id) {
            Ok(c) => c,
            Err(ApiErr(c, m)) => return (c, Some(m)),
        };
        let requested = *p_num_msgs as usize;
        let msgs = ch.read(requested, Duration::from_millis(timeout as u64));
        if msgs.is_empty() {
            if let Some(e) = ch.device_error() {
                *p_num_msgs = 0;
                return (ERR_DEVICE_NOT_CONNECTED, Some(e));
            }
        }
        for (i, m) in msgs.iter().enumerate() {
            let out = &mut *p_msg.add(i);
            let n = m.data.len().min(MSG_DATA_MAX);
            out.protocol_id = ch.protocol;
            out.rx_status = m.rx_status;
            out.tx_flags = 0;
            out.timestamp = m.timestamp;
            out.data_size = n as u32;
            out.extra_data_index = n as u32;
            out.data[..n].copy_from_slice(&m.data[..n]);
        }
        *p_num_msgs = msgs.len() as u32;
        let rc = if msgs.is_empty() {
            ERR_BUFFER_EMPTY
        } else if msgs.len() < requested && timeout > 0 {
            ERR_TIMEOUT
        } else {
            STATUS_NOERROR
        };
        (rc, None)
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruWriteMsgs(
    channel_id: u32,
    p_msg: *const PassThruMsg,
    p_num_msgs: *mut u32,
    timeout: u32,
) -> i32 {
    guard_rc("PassThruWriteMsgs", || {
        if p_msg.is_null() || p_num_msgs.is_null() {
            return (ERR_NULL_PARAMETER, Some("pMsg or pNumMsgs is NULL".into()));
        }
        let ch = match state::channel(channel_id) {
            Ok(c) => c,
            Err(ApiErr(c, m)) => return (c, Some(m)),
        };
        let n = *p_num_msgs as usize;
        let msgs: Vec<&PassThruMsg> = (0..n).map(|i| &*p_msg.add(i)).collect();
        let (sent, error) = ch.write(&msgs, Duration::from_millis(timeout as u64));
        *p_num_msgs = sent as u32;
        match error {
            None => (STATUS_NOERROR, None),
            Some(ApiErr(c, m)) => (c, Some(m)),
        }
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruStartPeriodicMsg(
    channel_id: u32,
    p_msg: *const PassThruMsg,
    p_msg_id: *mut u32,
    time_interval: u32,
) -> i32 {
    guard("PassThruStartPeriodicMsg", || {
        nonnull(p_msg, "pMsg")?;
        nonnull(p_msg_id, "pMsgID")?;
        *p_msg_id = state::channel(channel_id)?.start_periodic(&*p_msg, time_interval)?;
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruStopPeriodicMsg(channel_id: u32, msg_id: u32) -> i32 {
    guard("PassThruStopPeriodicMsg", || {
        state::channel(channel_id)?.stop_periodic(msg_id)
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruStartMsgFilter(
    channel_id: u32,
    filter_type: u32,
    p_mask_msg: *const PassThruMsg,
    p_pattern_msg: *const PassThruMsg,
    p_flow_control_msg: *const PassThruMsg,
    p_filter_id: *mut u32,
) -> i32 {
    guard("PassThruStartMsgFilter", || {
        nonnull(p_mask_msg, "pMaskMsg")?;
        nonnull(p_pattern_msg, "pPatternMsg")?;
        nonnull(p_filter_id, "pFilterID")?;
        if !matches!(
            filter_type,
            PASS_FILTER | BLOCK_FILTER | FLOW_CONTROL_FILTER
        ) {
            return err(ERR_NOT_SUPPORTED, "unknown filter type");
        }
        let ch = state::channel(channel_id)?;
        let flow = (!p_flow_control_msg.is_null()).then(|| &*p_flow_control_msg);
        *p_filter_id = ch.start_filter(filter_type, &*p_mask_msg, &*p_pattern_msg, flow)?;
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruStopMsgFilter(channel_id: u32, filter_id: u32) -> i32 {
    guard("PassThruStopMsgFilter", || {
        state::channel(channel_id)?.stop_filter(filter_id)
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruSetProgrammingVoltage(
    _device_id: u32,
    _pin_number: u32,
    _voltage: u32,
) -> i32 {
    guard("PassThruSetProgrammingVoltage", || {
        err(
            ERR_NOT_SUPPORTED,
            "the adapter has no programmable voltage output",
        )
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruReadVersion(
    device_id: u32,
    p_firmware_version: *mut c_char,
    p_dll_version: *mut c_char,
    p_api_version: *mut c_char,
) -> i32 {
    guard("PassThruReadVersion", || {
        if p_firmware_version.is_null() || p_dll_version.is_null() || p_api_version.is_null() {
            return err(ERR_NULL_PARAMETER, "version buffer is NULL");
        }
        let dev = state::device(device_id)?;
        write_cstr(
            p_firmware_version,
            &format!("{} {}", dev.usb.info.name, dev.usb.info.bcd_device),
        );
        write_cstr(p_dll_version, DLL_VERSION);
        write_cstr(p_api_version, API_VERSION);
        Ok(())
    })
}

#[no_mangle]
pub unsafe extern "system" fn PassThruGetLastError(p_error_description: *mut c_char) -> i32 {
    if p_error_description.is_null() {
        return ERR_NULL_PARAMETER;
    }
    write_cstr(p_error_description, &state::last_error());
    STATUS_NOERROR
}

#[no_mangle]
pub unsafe extern "system" fn PassThruIoctl(
    channel_id: u32,
    ioctl_id: u32,
    p_input: *mut c_void,
    p_output: *mut c_void,
) -> i32 {
    guard("PassThruIoctl", || {
        crate::trace!("  channel={channel_id} ioctl=0x{ioctl_id:X}");
        match ioctl_id {
            GET_CONFIG | SET_CONFIG => {
                nonnull(p_input, "pInput")?;
                let list = &*(p_input as *const SConfigList);
                if list.num_of_params > 0 {
                    nonnull(list.config_ptr, "ConfigPtr")?;
                }
                for i in 0..list.num_of_params as usize {
                    let item = &mut *list.config_ptr.add(i);
                    if ioctl_id == GET_CONFIG {
                        item.value = state::get_config(channel_id, item.parameter)?;
                    } else {
                        crate::trace!("  set 0x{:X} = {}", item.parameter, item.value);
                        state::set_config(channel_id, item.parameter, item.value)?;
                    }
                }
                Ok(())
            }
            CLEAR_TX_BUFFER => {
                state::channel(channel_id)?.clear_tx();
                Ok(())
            }
            CLEAR_RX_BUFFER => {
                state::channel(channel_id)?.clear_rx();
                Ok(())
            }
            CLEAR_PERIODIC_MSGS => {
                state::channel(channel_id)?.clear_periodic();
                Ok(())
            }
            CLEAR_MSG_FILTERS => {
                state::channel(channel_id)?.clear_filters();
                Ok(())
            }
            BUS_ON => {
                state::channel(channel_id)?;
                Ok(())
            }
            READ_VBATT | READ_PROG_VOLTAGE => {
                state::device_info(channel_id)?;
                err(
                    ERR_NOT_SUPPORTED,
                    "the adapter cannot measure the vehicle supply voltage",
                )
            }
            FIVE_BAUD_INIT
            | FAST_INIT
            | CLEAR_FUNCT_MSG_LOOKUP_TABLE
            | ADD_TO_FUNCT_MSG_LOOKUP_TABLE
            | DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE => err(ERR_NOT_SUPPORTED, "not applicable to CAN"),
            GET_DEVICE_INFO | GET_PROTOCOL_INFO => {
                let dev = state::device_info(channel_id)?;
                nonnull(p_input, "pInput")?;
                let list = &*(p_input as *const SParamList);
                for i in 0..list.num_of_params as usize {
                    let p = &mut *list.param_ptr.add(i);
                    let (value, supported) = match (ioctl_id, p.parameter) {
                        // J2534-2 identifiers published by Quantex/ScanDoc.
                        (GET_DEVICE_INFO, 0x4C | 0x4D) => {
                            (dev.usb.supports_fd() as u32, dev.usb.supports_fd())
                        }
                        (GET_PROTOCOL_INFO, 0x01) => (20_000, true), // MAX_RX_BUFFER_SIZE
                        (GET_PROTOCOL_INFO, 0x12) => (10, true),     // MAX_FLOW_CONTROL_FILTER
                        _ => (0, false),
                    };
                    p.value = value;
                    p.supported = supported as u32;
                }
                Ok(())
            }
            _ => {
                let _ = p_output;
                err(
                    ERR_INVALID_IOCTL_ID,
                    format!("ioctl 0x{ioctl_id:X} is not supported"),
                )
            }
        }
    })
}
