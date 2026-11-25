//! Optional call/frame trace, disabled (and nearly free) by default.
//!
//! Enabled by `GSUSB_J2534_LOG=<file>` in the environment of the application, or machine-wide by
//! the registry value `HKLM\SOFTWARE\GsUsbJ2534\TraceFile` (`make trace-on`; see `settings`), which also reaches
//! programs that are started from a shortcut. Several processes may append to the same file;
//! every line carries the process id.

use std::ffi::OsString;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

struct Sink {
    file: Mutex<std::fs::File>,
    start: Instant,
    pid: u32,
}

static SINK: OnceLock<Option<Sink>> = OnceLock::new();

fn registry_trace_path() -> Option<OsString> {
    crate::settings::registry_string("TraceFile")
}

fn sink() -> Option<&'static Sink> {
    SINK.get_or_init(|| {
        let path = std::env::var_os("GSUSB_J2534_LOG").or_else(registry_trace_path)?;
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok()?;
        let sink = Sink {
            file: Mutex::new(file),
            start: Instant::now(),
            pid: std::process::id(),
        };
        if let Ok(mut f) = sink.file.lock() {
            let exe = std::env::current_exe().unwrap_or_default();
            let _ = writeln!(f, "--- trace start pid {} {}", sink.pid, exe.display());
        }
        Some(sink)
    })
    .as_ref()
}

pub fn enabled() -> bool {
    sink().is_some()
}

pub fn write(args: std::fmt::Arguments) {
    if let Some(sink) = sink() {
        if let Ok(mut file) = sink.file.lock() {
            let _ = writeln!(
                file,
                "[{:>5} {:>10.3}] {}",
                sink.pid,
                sink.start.elapsed().as_secs_f64(),
                args
            );
        }
    }
}

#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        if $crate::log::enabled() {
            $crate::log::write(format_args!($($arg)*));
        }
    };
}
