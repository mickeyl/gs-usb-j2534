//! SAE J2534-1 04.04 ABI definitions (constants verified against SAE J2534-1_0500
//! JAN2022 tables 72-87, plus the J2534-2 CAN FD identifiers published by PEAK).

#![allow(dead_code)]

pub type Ulong = u32; // Windows LLP64: `unsigned long` is 32 bit, also on x64.

// --- return values -----------------------------------------------------------
pub const STATUS_NOERROR: i32 = 0x00;
pub const ERR_NOT_SUPPORTED: i32 = 0x01;
pub const ERR_INVALID_CHANNEL_ID: i32 = 0x02;
pub const ERR_INVALID_PROTOCOL_ID: i32 = 0x03;
pub const ERR_NULL_PARAMETER: i32 = 0x04;
pub const ERR_INVALID_IOCTL_VALUE: i32 = 0x05;
pub const ERR_INVALID_FLAGS: i32 = 0x06;
pub const ERR_FAILED: i32 = 0x07;
pub const ERR_DEVICE_NOT_CONNECTED: i32 = 0x08;
pub const ERR_TIMEOUT: i32 = 0x09;
pub const ERR_INVALID_MSG: i32 = 0x0A;
pub const ERR_INVALID_TIME_INTERVAL: i32 = 0x0B;
pub const ERR_EXCEEDED_LIMIT: i32 = 0x0C;
pub const ERR_INVALID_MSG_ID: i32 = 0x0D;
pub const ERR_DEVICE_IN_USE: i32 = 0x0E;
pub const ERR_INVALID_IOCTL_ID: i32 = 0x0F;
pub const ERR_BUFFER_EMPTY: i32 = 0x10;
pub const ERR_BUFFER_FULL: i32 = 0x11;
pub const ERR_BUFFER_OVERFLOW: i32 = 0x12;
pub const ERR_PIN_INVALID: i32 = 0x13;
pub const ERR_CHANNEL_IN_USE: i32 = 0x14;
pub const ERR_MSG_PROTOCOL_ID: i32 = 0x15;
pub const ERR_INVALID_FILTER_ID: i32 = 0x16;
pub const ERR_NO_FLOW_CONTROL: i32 = 0x17; // 04.04 name; 05.00: ERR_MSG_NOT_ALLOWED
pub const ERR_NOT_UNIQUE: i32 = 0x18;
pub const ERR_INVALID_BAUDRATE: i32 = 0x19;
pub const ERR_INVALID_DEVICE_ID: i32 = 0x1A;
pub const ERR_BUFFER_TOO_SMALL: i32 = 0x23;

// --- protocol ids ------------------------------------------------------------
pub const CAN: u32 = 0x05;
pub const ISO15765: u32 = 0x06;
/// SAE J2534-2 CAN FD (raw frames), identical to PEAK's registry value 32785.
pub const FD_CAN_PS: u32 = 0x8011;
/// SAE J2534-2 ISO 15765 on CAN FD, PEAK registry value 32786.
pub const FD_ISO15765_PS: u32 = 0x8012;

// --- Connect flags -----------------------------------------------------------
pub const CAN_29BIT_ID: u32 = 0x100;
pub const ISO9141_NO_CHECKSUM: u32 = 0x200;
pub const CAN_ID_BOTH: u32 = 0x800;

// --- RxStatus ----------------------------------------------------------------
pub const TX_MSG_TYPE: u32 = 1 << 0;
pub const START_OF_MESSAGE: u32 = 1 << 1;
pub const RX_BREAK: u32 = 1 << 2;
pub const TX_INDICATION_SUCCESS: u32 = 1 << 3;
pub const ISO15765_PADDING_ERROR: u32 = 1 << 4;
pub const ERROR_INDICATION: u32 = 1 << 5;
pub const BUFFER_OVERFLOW_FLAG: u32 = 1 << 6;
pub const ISO15765_ADDR_TYPE: u32 = 1 << 7;
pub const TX_FAILED: u32 = 1 << 9;
pub const FD_CAN_BRS: u32 = 1 << 19;
pub const FD_CAN_FORMAT: u32 = 1 << 20;
pub const FD_CAN_ESI: u32 = 1 << 21;

// --- TxFlags -----------------------------------------------------------------
pub const ISO15765_FRAME_PAD: u32 = 1 << 6;
// CAN_29BIT_ID (bit 8), ISO15765_ADDR_TYPE (bit 7), FD_CAN_FORMAT/BRS shared with RxStatus.

// --- filter types ------------------------------------------------------------
pub const PASS_FILTER: u32 = 1;
pub const BLOCK_FILTER: u32 = 2;
pub const FLOW_CONTROL_FILTER: u32 = 3;

// --- ioctl ids ---------------------------------------------------------------
pub const GET_CONFIG: u32 = 0x01;
pub const SET_CONFIG: u32 = 0x02;
pub const READ_VBATT: u32 = 0x03;
pub const FIVE_BAUD_INIT: u32 = 0x04;
pub const FAST_INIT: u32 = 0x05;
pub const CLEAR_TX_BUFFER: u32 = 0x07;
pub const CLEAR_RX_BUFFER: u32 = 0x08;
pub const CLEAR_PERIODIC_MSGS: u32 = 0x09;
pub const CLEAR_MSG_FILTERS: u32 = 0x0A;
pub const CLEAR_FUNCT_MSG_LOOKUP_TABLE: u32 = 0x0B;
pub const ADD_TO_FUNCT_MSG_LOOKUP_TABLE: u32 = 0x0C;
pub const DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE: u32 = 0x0D;
pub const READ_PROG_VOLTAGE: u32 = 0x0E;
pub const BUS_ON: u32 = 0x0F;
pub const GET_DEVICE_INFO: u32 = 0x800C;
pub const GET_PROTOCOL_INFO: u32 = 0x800D;

// --- config parameters -------------------------------------------------------
pub const DATA_RATE: u32 = 0x01;
pub const LOOPBACK: u32 = 0x03;
pub const ISO15765_BS: u32 = 0x1E;
pub const ISO15765_STMIN: u32 = 0x1F;
pub const BS_TX: u32 = 0x22;
pub const STMIN_TX: u32 = 0x23;
pub const ISO15765_WAIT_LIMIT: u32 = 0x25;
pub const ISO15765_PAD_VALUE: u32 = 0x2B;
pub const N_AS_MAX: u32 = 0x2C;
pub const N_AR_MAX: u32 = 0x2D;
pub const N_BS_MAX: u32 = 0x2E;
pub const N_CR_MAX: u32 = 0x2F;
pub const ECHO_PHYSICAL_CHANNEL_TX: u32 = 0x31;
pub const CAN_MIXED_FORMAT: u32 = 0x8000;
pub const J1962_PINS: u32 = 0x8001;
pub const FD_CAN_DATA_PHASE_RATE: u32 = 0x805C;
/// J2534-2 value; contiguous with the published FD parameters, not independently verified.
pub const FD_ISO15765_TX_DATA_LENGTH: u32 = 0x805D;
pub const HS_CAN_TERMINATION: u32 = 0x805E;

// --- limits ------------------------------------------------------------------
pub const MSG_DATA_MAX: usize = 4128;
pub const MAX_PERIODIC_MSGS: usize = 10;

/// 04.04 message layout with inline data buffer.
#[repr(C)]
#[derive(Clone)]
pub struct PassThruMsg {
    pub protocol_id: Ulong,
    pub rx_status: Ulong,
    pub tx_flags: Ulong,
    pub timestamp: Ulong,
    pub data_size: Ulong,
    pub extra_data_index: Ulong,
    pub data: [u8; MSG_DATA_MAX],
}

impl PassThruMsg {
    pub fn payload(&self) -> &[u8] {
        &self.data[..(self.data_size as usize).min(MSG_DATA_MAX)]
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SConfig {
    pub parameter: Ulong,
    pub value: Ulong,
}

#[repr(C)]
pub struct SConfigList {
    pub num_of_params: Ulong,
    pub config_ptr: *mut SConfig,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SParam {
    pub parameter: Ulong,
    pub value: Ulong,
    pub supported: Ulong,
}

#[repr(C)]
pub struct SParamList {
    pub num_of_params: Ulong,
    pub param_ptr: *mut SParam,
}

/// Valid CAN-FD data lengths (DLC 0..15 mapped to byte counts).
pub const FD_LENGTHS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

pub fn is_valid_fd_len(len: usize) -> bool {
    FD_LENGTHS.contains(&len)
}

/// Smallest valid FD frame length that holds `len` bytes.
pub fn round_up_fd_len(len: usize) -> Option<usize> {
    FD_LENGTHS.iter().copied().find(|&l| l >= len)
}

pub fn is_fd_protocol(protocol: u32) -> bool {
    matches!(protocol, FD_CAN_PS | FD_ISO15765_PS)
}

pub fn is_iso_protocol(protocol: u32) -> bool {
    matches!(protocol, ISO15765 | FD_ISO15765_PS)
}

pub fn is_supported_protocol(protocol: u32) -> bool {
    matches!(protocol, CAN | ISO15765 | FD_CAN_PS | FD_ISO15765_PS)
}
