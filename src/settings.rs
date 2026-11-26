//! Machine-wide driver settings, read from `HKLM\SOFTWARE\GsUsbJ2534` (64-bit view, also for the
//! 32-bit DLL) so that they reach applications started from a shortcut. Environment variables
//! override them for a single process.
//!
//! | value (registry) | environment | meaning |
//! |---|---|---|
//! | `TraceFile` (string) | `GSUSB_J2534_LOG` | trace file |
//! | `TxBrs` (dword, 0 = strip) | `GSUSB_J2534_TX_BRS` (`0` strip, `1` honor) | transmit with the BRS bit the application asked for (default: honor) |

use std::ffi::OsString;

#[cfg(windows)]
mod registry {
    use std::ffi::{c_void, OsString};
    use std::os::windows::ffi::OsStringExt;

    #[link(name = "advapi32")]
    extern "system" {
        fn RegGetValueW(
            hkey: isize,
            sub_key: *const u16,
            value: *const u16,
            flags: u32,
            kind: *mut u32,
            data: *mut c_void,
            size: *mut u32,
        ) -> i32;
    }
    const HKEY_LOCAL_MACHINE: isize = 0x8000_0002_u32 as i32 as isize;
    const RRF_RT_REG_SZ: u32 = 0x2;
    const RRF_RT_REG_DWORD: u32 = 0x10;
    // A 32-bit process would otherwise be redirected to WOW6432Node.
    const RRF_SUBKEY_WOW6464KEY: u32 = 0x0001_0000;

    fn wide(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(Some(0)).collect()
    }

    fn get(name: &str, flags: u32, data: *mut c_void, size: &mut u32) -> bool {
        let (key, value) = (wide(r"SOFTWARE\GsUsbJ2534"), wide(name));
        // SAFETY: `data` points to a live buffer of `*size` bytes owned by the caller.
        unsafe {
            RegGetValueW(
                HKEY_LOCAL_MACHINE,
                key.as_ptr(),
                value.as_ptr(),
                flags | RRF_SUBKEY_WOW6464KEY,
                std::ptr::null_mut(),
                data,
                size,
            ) == 0
        }
    }

    pub fn string(name: &str) -> Option<OsString> {
        let mut buf = [0u16; 520];
        let mut bytes = std::mem::size_of_val(&buf) as u32;
        if !get(name, RRF_RT_REG_SZ, buf.as_mut_ptr().cast(), &mut bytes) {
            return None;
        }
        let len = (bytes as usize / 2).saturating_sub(1).min(buf.len());
        (len > 0).then(|| OsString::from_wide(&buf[..len]))
    }

    pub fn dword(name: &str) -> Option<u32> {
        let mut value = 0u32;
        let mut bytes = 4u32;
        get(
            name,
            RRF_RT_REG_DWORD,
            (&mut value as *mut u32).cast(),
            &mut bytes,
        )
        .then_some(value)
    }
}

#[cfg(not(windows))]
mod registry {
    use std::ffi::OsString;
    pub fn string(_: &str) -> Option<OsString> {
        None
    }
    pub fn dword(_: &str) -> Option<u32> {
        None
    }
}

pub fn registry_string(name: &str) -> Option<OsString> {
    registry::string(name)
}

pub fn registry_dword(name: &str) -> Option<u32> {
    registry::dword(name)
}

/// Whether FD frames are transmitted with the BRS bit the application requested.
///
/// On by default: BRS at 2 Mbit/s is clean on the candleLightFD (bench and vehicle bus, 30,000
/// frames and a complete ECU flash, see docs/brs-investigation.md). Earlier versions stripped it
/// because of bus-offs that no longer occur; `TxBrs=0` or `GSUSB_J2534_TX_BRS=0` restores that for
/// a bus or adapter where BRS transmit is marginal.
pub fn honor_tx_brs() -> bool {
    match std::env::var("GSUSB_J2534_TX_BRS").as_deref() {
        Ok("1") => true,
        Ok("0") => false,
        _ => registry_dword("TxBrs") != Some(0),
    }
}
