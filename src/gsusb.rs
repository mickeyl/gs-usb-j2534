//! gs_usb / candleLight(FD) USB transport.
//!
//! The wire contract (control requests, host-frame layout, timing constants) follows
//! the candleLight firmware and the `gs_usb.rs` backend of j2534-bridge, which has
//! been validated on the same candleLightFD hardware. Unlike the bridge backend this
//! type is `Sync`: one thread may read while others write.

use rusb::{DeviceHandle, GlobalContext};
use std::time::Duration;

use crate::j2534::{is_valid_fd_len, FD_LENGTHS};

const TIMEOUT: Duration = Duration::from_secs(1);

/// VID/PID pairs of gs_usb compatible adapters (same list as j2534-bridge discovery).
pub const KNOWN_IDS: [(u16, u16); 9] = [
    (0x1d50, 0x606f), // candleLight / candleLightFD
    (0x1209, 0x2323),
    (0x1cd2, 0x606f),
    (0x16d0, 0x10b8),
    (0x16d0, 0x0f30),
    (0x1209, 0xca01),
    (0x1d50, 0x606d),
    (0x1d50, 0x6070),
    (0x1209, 0x0001),
];

// bRequest values
const REQ_HOST_FORMAT: u8 = 0;
const REQ_BITTIMING: u8 = 1;
const REQ_MODE: u8 = 2;
const REQ_BT_CONST: u8 = 4;
const REQ_DATA_BITTIMING: u8 = 10;
const REQ_BT_CONST_EXT: u8 = 12;

// Device feature bits (GS_CAN_FEATURE_*)
pub const FEAT_LISTEN_ONLY: u32 = 0x001;
pub const FEAT_HW_TIMESTAMP: u32 = 0x010;
pub const FEAT_FD: u32 = 0x100;
pub const FEAT_BT_CONST_EXT: u32 = 0x400;

// Mode flags
const MODE_LISTEN_ONLY: u32 = 0x001;
const MODE_HW_TIMESTAMP: u32 = 0x010;
const MODE_FD: u32 = 0x100;

// Host frame flags
const FLAG_OVERFLOW: u8 = 1;
const FLAG_FD: u8 = 2;
const FLAG_BRS: u8 = 4;
const FLAG_ESI: u8 = 8;

pub const ECHO_NONE: u32 = u32::MAX;
pub const CAN_EFF_FLAG: u32 = 0x8000_0000;
pub const CAN_RTR_FLAG: u32 = 0x4000_0000;
pub const CAN_ERR_FLAG: u32 = 0x2000_0000;

#[derive(Debug)]
pub enum UsbError {
    /// Requested bit rate cannot be generated exactly by the controller.
    BaudRate(String),
    Unsupported(String),
    /// No matching adapter is attached.
    NotFound(String),
    /// Adapter exists but is owned by another process / handle.
    Busy(String),
    Io(String),
}

impl std::fmt::Display for UsbError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UsbError::BaudRate(s)
            | UsbError::Unsupported(s)
            | UsbError::NotFound(s)
            | UsbError::Busy(s)
            | UsbError::Io(s) => f.write_str(s),
        }
    }
}

impl From<rusb::Error> for UsbError {
    fn from(e: rusb::Error) -> Self {
        match e {
            rusb::Error::Access | rusb::Error::Busy => UsbError::Busy(e.to_string()),
            rusb::Error::NoDevice | rusb::Error::NotFound => UsbError::NotFound(e.to_string()),
            _ => UsbError::Io(e.to_string()),
        }
    }
}

fn word(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn words(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    pub selector: String,
    pub name: String,
    pub serial: String,
    pub vid: u16,
    pub pid: u16,
    pub bcd_device: String,
}

#[derive(Clone, Copy, Debug)]
pub struct StartConfig {
    pub nominal: u32,
    pub data: Option<u32>,
    pub listen_only: bool,
}

/// A CAN frame as handled by the driver core.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub ext: bool,
    pub fd: bool,
    pub brs: bool,
    pub esi: bool,
    pub data: Vec<u8>,
}

impl CanFrame {
    pub fn classic(id: u32, ext: bool, data: &[u8]) -> Self {
        Self {
            id,
            ext,
            fd: false,
            brs: false,
            esi: false,
            data: data.to_vec(),
        }
    }
}

/// One host frame received from the adapter (RX frame or TX echo).
#[derive(Clone, Debug)]
pub struct RxPacket {
    pub echo_id: u32,
    pub frame: CanFrame,
    pub rtr: bool,
    pub error: bool,
    pub overflow: bool,
    pub hw_timestamp: Option<u32>,
}

impl RxPacket {
    pub fn is_echo(&self) -> bool {
        self.echo_id != ECHO_NONE
    }
}

/// Serialise a host frame for the bulk-OUT endpoint.
pub fn encode_tx(echo_id: u32, frame: &CanFrame) -> Result<Vec<u8>, String> {
    let max_id = if frame.ext { 0x1fff_ffff } else { 0x7ff };
    if frame.id > max_id {
        return Err("CAN id exceeds selected width".into());
    }
    let dlc = FD_LENGTHS
        .iter()
        .position(|n| *n == frame.data.len())
        .ok_or("invalid CAN data length")? as u8;
    if !frame.fd && frame.data.len() > 8 {
        return Err("classic CAN frame longer than 8 bytes".into());
    }
    if frame.brs && !frame.fd {
        return Err("BRS requires an FD frame".into());
    }
    let mut bytes = vec![0u8; if frame.fd { 76 } else { 20 }];
    bytes[..4].copy_from_slice(&echo_id.to_le_bytes());
    let can_id = frame.id | if frame.ext { CAN_EFF_FLAG } else { 0 };
    bytes[4..8].copy_from_slice(&can_id.to_le_bytes());
    bytes[8] = dlc;
    bytes[10] = if frame.fd {
        FLAG_FD | if frame.brs { FLAG_BRS } else { 0 }
    } else {
        0
    };
    bytes[12..12 + frame.data.len()].copy_from_slice(&frame.data);
    Ok(bytes)
}

/// Re-assembles host frames from the bulk-IN byte stream.
pub struct StreamDecoder {
    buf: Vec<u8>,
    timestamps: bool,
}

impl StreamDecoder {
    pub fn new(timestamps: bool) -> Self {
        Self {
            buf: Vec::new(),
            timestamps,
        }
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Next complete packet, `Ok(None)` when more bytes are needed. On a malformed
    /// header the buffer is discarded so the stream can resynchronise.
    pub fn next_packet(&mut self) -> Result<Option<RxPacket>, String> {
        if self.buf.len() < 12 {
            return Ok(None);
        }
        let flags = self.buf[10];
        let is_fd = flags & FLAG_FD != 0;
        let size = (if is_fd { 76 } else { 20 }) + if self.timestamps { 4 } else { 0 };
        if self.buf.len() < size {
            return Ok(None);
        }
        let packet: Vec<u8> = self.buf.drain(..size).collect();
        decode_packet(&packet, self.timestamps)
            .map(Some)
            .map_err(|e| {
                self.buf.clear();
                e
            })
    }
}

pub fn decode_packet(packet: &[u8], timestamps: bool) -> Result<RxPacket, String> {
    if packet.len() < 12 {
        return Err("short gs_usb header".into());
    }
    let flags = packet[10];
    let is_fd = flags & FLAG_FD != 0;
    let base = if is_fd { 76 } else { 20 };
    if packet.len() != base + if timestamps { 4 } else { 0 } {
        return Err("invalid gs_usb packet size".into());
    }
    let raw_id = word(packet, 4);
    let dlc = packet[8] as usize;
    let len = *FD_LENGTHS.get(dlc).ok_or("invalid gs_usb DLC")?;
    if !is_fd && len > 8 {
        // Classic DLC > 8 means 8 bytes on the wire.
        return Err("invalid classic CAN length".into());
    }
    let error = raw_id & CAN_ERR_FLAG != 0;
    let rtr = raw_id & CAN_RTR_FLAG != 0;
    let ext = raw_id & CAN_EFF_FLAG != 0;
    let hw_timestamp = timestamps.then(|| word(packet, base));
    Ok(RxPacket {
        echo_id: word(packet, 0),
        frame: CanFrame {
            id: raw_id & 0x1fff_ffff,
            ext,
            fd: is_fd,
            brs: flags & FLAG_BRS != 0,
            esi: flags & FLAG_ESI != 0,
            data: packet[12..12 + len].to_vec(),
        },
        rtr,
        error,
        overflow: flags & FLAG_OVERFLOW != 0,
        hw_timestamp,
    })
}

/// Controller bit timing in gs_usb order: prop_seg, phase_seg1, phase_seg2, sjw, brp.
/// `GSUSB_J2534_DATA_SAMPLE=<permille>` moves the data-phase sample point (tuning knob for buses
/// where transmitting with BRS fails because of transceiver loop delay).
fn data_sample_override(request: u8) -> Option<u32> {
    if request != REQ_DATA_BITTIMING {
        return None;
    }
    std::env::var("GSUSB_J2534_DATA_SAMPLE")
        .ok()?
        .parse::<u32>()
        .ok()
        .filter(|v| (500..=950).contains(v))
}

/// `GSUSB_J2534_DATA_TIMING=prop,ph1,ph2,sjw,brp` replaces the computed data-phase timing
/// (experiments with transmitter timing; the values are checked against the controller limits).
fn data_timing_override() -> Option<[u32; 5]> {
    let v = std::env::var("GSUSB_J2534_DATA_TIMING").ok()?;
    let p: Vec<u32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
    (p.len() == 5).then(|| [p[0], p[1], p[2], p[3], p[4]])
}

/// Check an explicit timing `[prop, phase1, phase2, sjw, brp]` against the controller limits and
/// the requested bit rate.
pub fn validate_timing(
    clock: u32,
    limits: &[u8],
    bitrate: u32,
    t: [u32; 5],
) -> Result<(), UsbError> {
    let [t1_min, t1_max, t2_min, t2_max, sjw_max, brp_min, brp_max, brp_inc]: [u32; 8] =
        std::array::from_fn(|i| word(limits, i * 4));
    let [prop, ph1, ph2, sjw, brp] = t;
    let t1 = prop + ph1;
    let bad = |what: &str| Err(UsbError::BaudRate(format!("data timing {t:?}: {what}")));
    if !(t1_min..=t1_max).contains(&t1) {
        return bad("prop+phase1 outside the controller range");
    }
    if !(t2_min..=t2_max).contains(&ph2) {
        return bad("phase2 outside the controller range");
    }
    if sjw == 0 || sjw > sjw_max || sjw > ph2 || sjw > ph1 {
        return bad("sjw must be 1..=min(phase1, phase2, sjw_max)");
    }
    if !(brp_min..=brp_max).contains(&brp) || (brp - brp_min) % brp_inc.max(1) != 0 {
        return bad("brp outside the controller range");
    }
    let quanta = 1 + t1 + ph2;
    if u64::from(clock) != u64::from(bitrate) * u64::from(brp) * u64::from(quanta) {
        return bad("does not produce the requested bit rate");
    }
    Ok(())
}

pub fn calculate_timing(
    request: u8,
    clock: u32,
    limits: &[u8],
    bitrate: u32,
) -> Result<[u32; 5], UsbError> {
    if limits.len() != 32 || clock == 0 || bitrate == 0 {
        return Err(UsbError::BaudRate("invalid timing constants/rate".into()));
    }
    let [t1_min, t1_max, t2_min, t2_max, sjw_max, brp_min, brp_max, brp_inc]: [u32; 8] =
        std::array::from_fn(|i| word(limits, i * 4));
    // Sample point target in permille: 80% (75% for data phases >= 4 Mbit/s).
    let target = data_sample_override(request).unwrap_or(
        if request == REQ_DATA_BITTIMING && bitrate >= 4_000_000 {
            750
        } else {
            800
        },
    );
    let mut best: Option<(u32, [u32; 5])> = None;
    for brp in (brp_min.max(1)..=brp_max.min(1024)).step_by(brp_inc.max(1) as usize) {
        let divisor = u64::from(bitrate) * u64::from(brp);
        if divisor == 0 || u64::from(clock) % divisor != 0 {
            continue;
        }
        let quanta = (u64::from(clock) / divisor) as u32;
        for t2 in t2_min.max(2)..=t2_max.min(512) {
            let Some(t1) = quanta.checked_sub(t2 + 1) else {
                continue;
            };
            if t1 < t1_min || t1 > t1_max {
                continue;
            }
            let sample = (1 + t1) * 1000 / quanta;
            if sample > target {
                continue;
            }
            let error = sample.abs_diff(target);
            if best.as_ref().map_or(true, |(score, _)| error <= *score) {
                best = Some((
                    error,
                    [t1 / 2, t1 - t1 / 2, t2, sjw_max.min(t2).max(1), brp],
                ));
            }
        }
    }
    best.map(|(_, t)| t)
        .ok_or_else(|| UsbError::BaudRate(format!("no exact gs_usb timing for {bitrate} bit/s")))
}

/// What the bus layer needs from an adapter; implemented by [`GsUsb`] and by test doubles.
pub trait CanAdapter: Send + Sync {
    fn start(&self, cfg: &StartConfig) -> Result<(), UsbError>;
    fn stop(&self) -> Result<(), UsbError>;
    /// Raw bulk-IN bytes (host frames); `Ok(0)` on timeout.
    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, UsbError>;
    fn write_frame(&self, echo_id: u32, frame: &CanFrame) -> Result<(), UsbError>;
    /// Whether received host frames carry a trailing 32-bit hardware timestamp.
    fn has_hw_timestamp(&self) -> bool;
}

impl CanAdapter for GsUsb {
    fn start(&self, cfg: &StartConfig) -> Result<(), UsbError> {
        GsUsb::start(self, cfg)
    }
    fn stop(&self) -> Result<(), UsbError> {
        GsUsb::stop(self)
    }
    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, UsbError> {
        GsUsb::read(self, buf, timeout)
    }
    fn write_frame(&self, echo_id: u32, frame: &CanFrame) -> Result<(), UsbError> {
        GsUsb::write_frame(self, echo_id, frame)
    }
    fn has_hw_timestamp(&self) -> bool {
        GsUsb::has_hw_timestamp(self)
    }
}

pub struct GsUsb {
    handle: DeviceHandle<GlobalContext>,
    interface: u8,
    input: u8,
    output: u8,
    pub info: DeviceInfo,
    pub features: u32,
    pub clock: u32,
    nominal_limits: [u8; 32],
    data_limits: Option<[u8; 32]>,
}

/// All attached gs_usb adapters. Opens each device to read its strings; devices
/// that are busy or inaccessible are skipped.
pub fn enumerate() -> Vec<DeviceInfo> {
    let mut out = Vec::new();
    let Ok(devices) = rusb::devices() else {
        return out;
    };
    for device in devices.iter() {
        let Ok(desc) = device.device_descriptor() else {
            continue;
        };
        let (vid, pid) = (desc.vendor_id(), desc.product_id());
        if !KNOWN_IDS.contains(&(vid, pid)) {
            continue;
        }
        let Ok(handle) = device.open() else { continue };
        let serial = handle
            .read_serial_number_string_ascii(&desc)
            .unwrap_or_default();
        let name = handle
            .read_product_string_ascii(&desc)
            .unwrap_or_else(|_| "gs_usb".into());
        out.push(DeviceInfo {
            selector: selector(vid, pid, &serial),
            name,
            serial,
            vid,
            pid,
            bcd_device: bcd(&desc),
        });
    }
    out
}

fn selector(vid: u16, pid: u16, serial: &str) -> String {
    format!(
        "gs_usb:{vid:04x}:{pid:04x}:{}",
        if serial.is_empty() { "*" } else { serial }
    )
}

fn bcd(desc: &rusb::DeviceDescriptor) -> String {
    let v = desc.device_version();
    format!("{}.{}.{}", v.major(), v.minor(), v.sub_minor())
}

impl GsUsb {
    /// `selector` is `gs_usb:VID:PID:SERIAL` (as listed by [`enumerate`]) or `None`
    /// for the first adapter found.
    pub fn open(selector_str: Option<&str>) -> Result<Self, UsbError> {
        let wanted = match selector_str {
            Some(s) if !s.is_empty() => {
                let p: Vec<_> = s.split(':').collect();
                if p.len() != 4 || p[0] != "gs_usb" {
                    return Err(UsbError::Unsupported(format!(
                        "device name must be gs_usb:VID:PID:SERIAL, got `{s}`"
                    )));
                }
                let vid = u16::from_str_radix(p[1], 16).map_err(|e| UsbError::Io(e.to_string()))?;
                let pid = u16::from_str_radix(p[2], 16).map_err(|e| UsbError::Io(e.to_string()))?;
                Some((vid, pid, p[3].to_string()))
            }
            _ => None,
        };
        let devices = rusb::devices()?;
        let mut found = None;
        for device in devices.iter() {
            let Ok(desc) = device.device_descriptor() else {
                continue;
            };
            let ids = (desc.vendor_id(), desc.product_id());
            if !KNOWN_IDS.contains(&ids) {
                continue;
            }
            if let Some((vid, pid, serial)) = &wanted {
                if ids != (*vid, *pid) {
                    continue;
                }
                if serial != "*" {
                    let ok = device
                        .open()
                        .and_then(|h| h.read_serial_number_string_ascii(&desc))
                        .is_ok_and(|s| &s == serial);
                    if !ok {
                        continue;
                    }
                }
            }
            found = Some((device, desc));
            break;
        }
        let (device, desc) =
            found.ok_or_else(|| UsbError::NotFound("no gs_usb adapter found".into()))?;

        let config = device.active_config_descriptor()?;
        let interface = config
            .interfaces()
            .next()
            .ok_or_else(|| UsbError::Io("no USB interface".into()))?;
        let idesc = interface
            .descriptors()
            .next()
            .ok_or_else(|| UsbError::Io("no USB interface descriptor".into()))?;
        let bulk: Vec<_> = idesc
            .endpoint_descriptors()
            .filter(|e| e.transfer_type() == rusb::TransferType::Bulk)
            .collect();
        let input = bulk
            .iter()
            .find(|e| e.direction() == rusb::Direction::In)
            .ok_or_else(|| UsbError::Io("no bulk-IN endpoint".into()))?
            .address();
        let output = bulk
            .iter()
            .find(|e| e.direction() == rusb::Direction::Out)
            .ok_or_else(|| UsbError::Io("no bulk-OUT endpoint".into()))?
            .address();
        let handle = device.open().map_err(|e| match UsbError::from(e) {
            UsbError::Busy(m) => UsbError::Busy(format!(
                "open gs_usb: {m}; another program owns the adapter or no WinUSB driver is bound"
            )),
            other => other,
        })?;
        let serial = handle
            .read_serial_number_string_ascii(&desc)
            .unwrap_or_default();
        let name = handle
            .read_product_string_ascii(&desc)
            .unwrap_or_else(|_| "gs_usb".into());
        handle.claim_interface(idesc.interface_number())?;

        let mut usb = Self {
            handle,
            interface: idesc.interface_number(),
            input,
            output,
            info: DeviceInfo {
                selector: selector(desc.vendor_id(), desc.product_id(), &serial),
                name,
                serial,
                vid: desc.vendor_id(),
                pid: desc.product_id(),
                bcd_device: bcd(&desc),
            },
            features: 0,
            clock: 0,
            nominal_limits: [0; 32],
            data_limits: None,
        };
        usb.control_at(REQ_HOST_FORMAT, 1, usb.interface as u16, &words(&[0xbeef]))?;
        usb.control(REQ_MODE, &words(&[0, 0]))?;
        let constants = usb.read_control(REQ_BT_CONST, 40)?;
        usb.features = word(&constants, 0);
        usb.clock = word(&constants, 4);
        usb.nominal_limits.copy_from_slice(&constants[8..40]);
        if usb.features & FEAT_FD != 0 && usb.features & FEAT_BT_CONST_EXT != 0 {
            match usb.read_control(REQ_BT_CONST_EXT, 72) {
                Ok(ext) => {
                    let mut limits = [0u8; 32];
                    limits.copy_from_slice(&ext[40..72]);
                    usb.data_limits = Some(limits);
                }
                // Legacy candleLightFD advertises BT_CONST_EXT but stalls it; use the same
                // conservative data-phase limits as CANsole's GsUsbDevice::fdFallbackBtConst.
                Err(e)
                    if (usb.info.vid, usb.info.pid, usb.clock, usb.features)
                        == (0x1d50, 0x606f, 40_000_000, 0x5b3) =>
                {
                    crate::trace!(
                        "legacy candleLightFD BT_CONST_EXT stalled ({e}); using fallback limits"
                    );
                    let mut limits = [0u8; 32];
                    limits.copy_from_slice(&words(&[1, 32, 1, 16, 16, 1, 32, 1]));
                    usb.data_limits = Some(limits);
                }
                Err(e) => return Err(e),
            }
        }
        crate::trace!(
            "gs_usb {:04x}:{:04x} features=0x{:X} clock={} Hz",
            usb.info.vid,
            usb.info.pid,
            usb.features,
            usb.clock
        );
        Ok(usb)
    }

    pub fn supports_fd(&self) -> bool {
        self.features & FEAT_FD != 0 && self.data_limits.is_some()
    }

    pub fn supports_listen_only(&self) -> bool {
        self.features & FEAT_LISTEN_ONLY != 0
    }

    pub fn has_hw_timestamp(&self) -> bool {
        self.features & FEAT_HW_TIMESTAMP != 0
    }

    /// Program bit timing and start the controller.
    pub fn start(&self, cfg: &StartConfig) -> Result<(), UsbError> {
        if cfg.listen_only && !self.supports_listen_only() {
            return Err(UsbError::Unsupported(
                "adapter has no listen-only mode".into(),
            ));
        }
        // Reset first so the timing registers may be written.
        self.control(REQ_MODE, &words(&[0, 0]))?;
        let nominal =
            calculate_timing(REQ_BITTIMING, self.clock, &self.nominal_limits, cfg.nominal)?;
        crate::trace!("gs_usb nominal {} -> {:?}", cfg.nominal, nominal);
        self.control(REQ_BITTIMING, &words(&nominal))?;
        if let Some(rate) = cfg.data {
            let limits = match (self.supports_fd(), &self.data_limits) {
                (true, Some(l)) => l,
                _ => {
                    return Err(UsbError::Unsupported(
                        "adapter does not support CAN FD".into(),
                    ))
                }
            };
            let timing = match data_timing_override() {
                Some(t) => {
                    validate_timing(self.clock, limits, rate, t)?;
                    t
                }
                None => calculate_timing(REQ_DATA_BITTIMING, self.clock, limits, rate)?,
            };
            crate::trace!("gs_usb data {} -> {:?}", rate, timing);
            self.control(REQ_DATA_BITTIMING, &words(&timing))?;
        }
        self.drain();
        let flags = if cfg.data.is_some() { MODE_FD } else { 0 }
            | if self.has_hw_timestamp() {
                MODE_HW_TIMESTAMP
            } else {
                0
            }
            | if cfg.listen_only { MODE_LISTEN_ONLY } else { 0 };
        self.control(REQ_MODE, &words(&[1, flags]))?;
        Ok(())
    }

    /// Reset the controller (CAN off). Safe to call repeatedly.
    pub fn stop(&self) -> Result<(), UsbError> {
        self.control(REQ_MODE, &words(&[0, 0]))
    }

    /// Discard bytes left in the IN pipe by an earlier session.
    fn drain(&self) {
        let mut stale = [0u8; 4096];
        for _ in 0..32 {
            if self
                .handle
                .read_bulk(self.input, &mut stale, Duration::from_millis(10))
                .is_err()
            {
                break;
            }
        }
    }

    /// Blocking bulk read; `Ok(0)` on timeout.
    pub fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, UsbError> {
        match self.handle.read_bulk(self.input, buf, timeout) {
            Ok(n) => Ok(n),
            Err(rusb::Error::Timeout) => Ok(0),
            Err(e) => Err(e.into()),
        }
    }

    pub fn write_frame(&self, echo_id: u32, frame: &CanFrame) -> Result<(), UsbError> {
        let bytes = encode_tx(echo_id, frame).map_err(UsbError::Unsupported)?;
        let written = self.handle.write_bulk(self.output, &bytes, TIMEOUT)?;
        if written != bytes.len() {
            return Err(UsbError::Io("short gs_usb bulk write".into()));
        }
        Ok(())
    }

    fn control_at(
        &self,
        request: u8,
        value: u16,
        index: u16,
        bytes: &[u8],
    ) -> Result<(), UsbError> {
        let size = self
            .handle
            .write_control(0x41, request, value, index, bytes, TIMEOUT)?;
        if size != bytes.len() {
            return Err(UsbError::Io("short gs_usb control write".into()));
        }
        Ok(())
    }

    fn control(&self, request: u8, bytes: &[u8]) -> Result<(), UsbError> {
        self.control_at(request, 0, 0, bytes)
    }

    fn read_control(&self, request: u8, size: usize) -> Result<Vec<u8>, UsbError> {
        let mut bytes = vec![0; size];
        let received = self
            .handle
            .read_control(0xc1, request, 0, 0, &mut bytes, TIMEOUT)?;
        if received != size {
            return Err(UsbError::Io(format!(
                "short gs_usb capability read: {received}/{size}"
            )));
        }
        Ok(bytes)
    }
}

impl Drop for GsUsb {
    fn drop(&mut self) {
        let _ = self.stop();
        let _ = self.handle.release_interface(self.interface);
    }
}

/// Validates a frame against channel capabilities (shared by raw and ISO-TP paths).
pub fn validate_frame(frame: &CanFrame, fd_channel: bool) -> Result<(), String> {
    if frame.fd && !fd_channel {
        return Err("FD frame on a classic channel".into());
    }
    if frame.fd {
        if !is_valid_fd_len(frame.data.len()) {
            return Err("invalid CAN FD data length".into());
        }
    } else if frame.data.len() > 8 {
        return Err("classic CAN data longer than 8 bytes".into());
    }
    if frame.brs && !frame.fd {
        return Err("BRS requires an FD frame".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits_40mhz() -> Vec<u8> {
        words(&[1, 32, 1, 16, 16, 1, 32, 1])
    }

    fn rx_packet(fd: bool, dlc: u8, flags: u8, id: u32, ts: bool) -> Vec<u8> {
        let base = if fd { 76 } else { 20 };
        let mut p = vec![0; base + if ts { 4 } else { 0 }];
        p[..4].copy_from_slice(&ECHO_NONE.to_le_bytes());
        p[4..8].copy_from_slice(&id.to_le_bytes());
        p[8] = dlc;
        p[10] = flags | if fd { FLAG_FD } else { 0 };
        if ts {
            p[base..].copy_from_slice(&0xfedcba98u32.to_le_bytes());
        }
        p
    }

    #[test]
    fn decodes_fd_frame_with_brs_esi_extended_id_and_timestamp() {
        let mut wire = rx_packet(true, 15, FLAG_BRS | FLAG_ESI, 0x8000_0123, true);
        for (i, b) in wire[12..76].iter_mut().enumerate() {
            *b = i as u8;
        }
        let p = decode_packet(&wire, true).unwrap();
        assert_eq!(p.frame.data, (0..64).collect::<Vec<u8>>());
        assert_eq!(p.frame.id, 0x123);
        assert!(p.frame.ext && p.frame.fd && p.frame.brs && p.frame.esi);
        assert_eq!(p.hw_timestamp, Some(0xfedcba98));
        assert!(!p.is_echo());
    }

    #[test]
    fn rejects_bad_lengths_and_marks_error_frames() {
        assert!(decode_packet(&[0; 8], false).is_err());
        assert!(decode_packet(&rx_packet(false, 9, 0, 0x123, false), false).is_err());
        let e = decode_packet(&rx_packet(false, 8, 0, 0x2000_0004, false), false).unwrap();
        assert!(e.error);
    }

    #[test]
    fn stream_decoder_handles_split_and_back_to_back_packets() {
        let a = rx_packet(false, 8, 0, 0x100, true);
        let b = rx_packet(true, 8, 0, 0x18DA_F110 | CAN_EFF_FLAG, true);
        let mut all = a.clone();
        all.extend_from_slice(&b);
        let mut d = StreamDecoder::new(true);
        d.push(&all[..10]);
        assert!(d.next_packet().unwrap().is_none());
        d.push(&all[10..30]);
        let first = d.next_packet().unwrap().unwrap();
        assert_eq!(first.frame.id, 0x100);
        assert!(d.next_packet().unwrap().is_none());
        d.push(&all[30..]);
        let second = d.next_packet().unwrap().unwrap();
        assert_eq!(second.frame.id, 0x18DA_F110);
        assert!(second.frame.fd);
        assert!(d.next_packet().unwrap().is_none());
    }

    #[test]
    fn encode_roundtrips_through_decode_layout() {
        let f = CanFrame {
            id: 0x18DA_10F1,
            ext: true,
            fd: true,
            brs: true,
            esi: false,
            data: vec![0x02, 0x3E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA],
        };
        let wire = encode_tx(3, &f).unwrap();
        assert_eq!(wire.len(), 76);
        assert_eq!(&wire[..4], &3u32.to_le_bytes());
        assert_eq!(&wire[4..8], &(0x18DA_10F1u32 | CAN_EFF_FLAG).to_le_bytes());
        assert_eq!(wire[8], 8);
        assert_eq!(wire[10], FLAG_FD | FLAG_BRS);
        assert_eq!(&wire[12..20], &f.data[..]);
        let classic = CanFrame::classic(0x7DF, false, &[1, 2, 3]);
        assert_eq!(encode_tx(0, &classic).unwrap().len(), 20);
        assert!(encode_tx(0, &CanFrame::classic(0x800, false, &[])).is_err());
        assert!(encode_tx(0, &CanFrame::classic(1, false, &[0; 9])).is_err());
    }

    #[test]
    fn exact_timing_respects_clock_and_limits() {
        for rate in [500_000, 1_000_000, 2_000_000, 4_000_000] {
            let t =
                calculate_timing(REQ_DATA_BITTIMING, 40_000_000, &limits_40mhz(), rate).unwrap();
            assert_eq!(40_000_000, rate * t[4] * (1 + t[0] + t[1] + t[2]));
            assert!(t[0] + t[1] <= 32 && t[2] <= 16 && t[3] <= t[2]);
        }
        assert!(matches!(
            calculate_timing(REQ_BITTIMING, 40_000_000, &limits_40mhz(), 3_000_001),
            Err(UsbError::BaudRate(_))
        ));
        assert!(calculate_timing(REQ_BITTIMING, 0, &limits_40mhz(), 500_000).is_err());
    }

    #[test]
    fn explicit_data_timing_is_validated_against_limits_and_rate() {
        let limits = limits_40mhz();
        // 2 Mbit/s with 20 time quanta and a 80 % sample point, brp 1 (the PEAK-like layout)
        assert!(validate_timing(40_000_000, &limits, 2_000_000, [7, 8, 4, 4, 1]).is_ok());
        // the computed default: brp 2, 10 time quanta
        assert!(validate_timing(40_000_000, &limits, 2_000_000, [3, 4, 2, 2, 2]).is_ok());
        // wrong rate, sjw larger than phase 2, brp out of range
        assert!(validate_timing(40_000_000, &limits, 2_000_000, [7, 8, 5, 4, 1]).is_err());
        assert!(validate_timing(40_000_000, &limits, 2_000_000, [7, 8, 4, 5, 1]).is_err());
        assert!(validate_timing(40_000_000, &limits, 2_000_000, [3, 4, 2, 2, 0]).is_err());
    }

    #[test]
    fn frame_validation() {
        let fd8 = CanFrame {
            fd: true,
            ..CanFrame::classic(1, false, &[0; 12])
        };
        assert!(validate_frame(&fd8, true).is_ok());
        assert!(validate_frame(&fd8, false).is_err());
        let bad = CanFrame {
            fd: true,
            ..CanFrame::classic(1, false, &[0; 9])
        };
        assert!(validate_frame(&bad, true).is_err());
    }
}
