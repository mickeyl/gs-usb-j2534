//! 64-bit SAE J2534-1 04.04 PassThru driver for gs_usb / candleLight(FD) adapters.
//!
//! Layering: [`gsusb`] (USB wire protocol) → [`bus`] (reader thread, TX slots, fan-out)
//! → [`channel`] (filters, ISO-TP via [`isotp`]) → [`state`] (registry) → [`api`]
//! (exported `PassThru*` functions).

pub mod api;
pub mod bus;
pub mod channel;
pub mod gsusb;
pub mod isotp;
pub mod j2534;
pub mod log;
pub mod settings;
pub mod state;

pub use api::*;
