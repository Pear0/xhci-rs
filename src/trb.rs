use core::fmt::{Debug, Formatter};

use modular_bitfield::prelude::*;

use crate::consts::*;
use crate::HAL;

/* --------------------------- TRBs --------------------------- */

#[derive(Debug)]
pub enum TRBType {
    Unknown(TRB),
    Link(LinkTRB),
    PortStatusChange(PortStatusChangeTRB),
    CommandCompletion(CommandCompletionTRB),
    HostControllerEvent(HostControllerEventTRB),
    TransferEvent(TransferEventTRB),
}

impl From<TRB> for TRBType {
    fn from(t: TRB) -> Self {
        use TRBType::*;
        match t.type_id() {
            TRB_TYPE_LINK => Link(unsafe { t.link }),
            TRB_TYPE_EVNT_PORT_STATUS_CHG => PortStatusChange(unsafe { t.port_status_change }),
            TRB_TYPE_EVNT_CMD_COMPLETE => CommandCompletion(unsafe { t.command_completion }),
            TRB_TYPE_EVNT_HC => HostControllerEvent(unsafe { t.host_controller_event }),
            TRB_TYPE_EVNT_TRANSFER => TransferEvent(unsafe { t.transfer_event }),
            _ => TRBType::Unknown(t),
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub union TRB {
    pub command: CommandTRB,
    pub setup: SetupStageTRB,
    pub data: DataStageTRB,
    pub status_stage: StatusStageTRB,
    pub event_data: EventDataTRB,
    pub normal: NormalTRB,
    pub link: LinkTRB,
    pub port_status_change: PortStatusChangeTRB,
    pub command_completion: CommandCompletionTRB,
    pub host_controller_event: HostControllerEventTRB,
    pub transfer_event: TransferEventTRB,
    pseudo: PseudoTRB,
}

const_assert_size!(TRB, 16);

impl Debug for TRB {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "Generic TRB with type={}", self.type_id())
    }
}

impl TRB {
    pub fn set_cycle_state(&mut self, val: u8) {
        let mut tmp = unsafe { self.pseudo.flags };
        tmp &= !TRB_COMMON_CYCLE_STATE_MASK;
        tmp |= (val as u16) & TRB_COMMON_CYCLE_STATE_MASK;
        self.pseudo.flags = tmp
    }

    pub fn get_cycle_state(&self) -> u8 {
        (unsafe { self.pseudo.flags } & TRB_COMMON_CYCLE_STATE_MASK) as u8
    }

    pub fn get_type(&self) -> u16 {
        unsafe { (self.pseudo.flags & TRB_COMMON_TYPE_MASK) >> TRB_COMMON_TYPE_SHIFT }
    }
}

impl TRB {
    pub fn type_id(&self) -> u16 {
        (unsafe { self.pseudo.flags } & TRB_COMMON_TYPE_MASK) >> TRB_COMMON_TYPE_SHIFT
    }
}

impl Default for TRB {
    fn default() -> Self {
        TRB {
            pseudo: Default::default(),
        }
    }
}
/* ------------ Event TRBs -------- */
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PortStatusChangeTRB {
    _res0: [u8; 3],
    pub port_id: u8,
    _res1: [u8; 7],
    pub completion_code: u8,
    pub flags: u16,
    _res2: u16,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct HostControllerEventTRB {
    _res: [u8; 11],
    complete_code: u8,
    flags: u16,
    _res1: u16,
}

#[bitfield]
#[derive(Copy, Clone, Debug)]
pub struct TransferEventTRBStatusWord {
    bytes_remain: B24,
    code: B8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct TransferEventTRB {
    pub trb_pointer: u64,
    pub status: TransferEventTRBStatusWord,
    pub flags: u16,
    pub endpoint: u8,
    pub slot: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct CommandCompletionTRB {
    pub trb_pointer: u64,
    pub params: [u8; 3],
    pub code: u8,
    pub flags: u16,
    pub vfid: u8,
    pub slot: u8,
}

/* ------------ Pseudo TRB -------- */
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PseudoTRB {
    _res0: [u32; 3],
    flags: u16,
    _res1: u16,
}

impl Default for PseudoTRB {
    fn default() -> Self {
        unsafe {
            core::mem::zeroed()
        }
    }
}

/* -------- Link TRB -------------- */
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct LinkTRB {
    pub next_trb: u64,
    _res0: [u8; 3],
    pub int_target: u8,
    /// refer to Section 6.4.4.1
    pub link_flags: u16,
    _res1: u16,
}

impl LinkTRB {
    pub fn new(link: u64) -> LinkTRB {
        LinkTRB {
            next_trb: link,
            _res0: [0u8; 3],
            int_target: 0,
            link_flags: ((TRB_TYPE_LINK as u16) << TRB_COMMON_TYPE_SHIFT),
            _res1: 0,
        }
    }
}

/* -------- Command TRB ------------ */
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct CommandTRB {
    pub payload: [u32; 4],
}

impl Debug for CommandTRB {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let type_id = (self.payload[3] & TRB_COMMON_TYPE_MASK as u32) >> TRB_COMMON_TYPE_SHIFT as u32;
        write!(f, "TRB {{ type: {} }}", type_id)
    }
}

impl CommandTRB {
    pub fn noop() -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_NOOP_COMMAND as u32) << (TRB_COMMON_TYPE_SHIFT as u32);
        trb
    }

    pub fn enable_slot() -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_ENABLE_SLOT_CMD as u32) << (TRB_COMMON_TYPE_SHIFT as u32);
        trb
    }

    pub fn address_device(slot: u8, context_ptr: u64, block: bool) -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_ADDRESS_DEVICE_CMD as u32) << TRB_COMMON_TYPE_SHIFT as u32;
        assert_eq!(context_ptr & 0b1111, 0, "alignment");
        trb.payload[0] = context_ptr as u32;
        trb.payload[1] = (context_ptr >> 32) as u32;
        trb.payload[3] |= (slot as u32) << 24;
        if block {
            trb.payload[3] |= 1u32 << 9;
        }
        trb
    }

    pub fn configure_endpoint(slot: u8, context_ptr: u64) -> Self {
        let mut trb = Self::default();
        trb.payload[3] |= (TRB_TYPE_CONFIG_ENDPOINT_CMD as u32) << TRB_COMMON_TYPE_SHIFT as u32;
        assert_eq!(context_ptr & 0b1111, 0, "alignment");
        trb.payload[0] = context_ptr as u32;
        trb.payload[1] = (context_ptr >> 32) as u32;
        trb.payload[3] |= (slot as u32) << 24;
        trb
    }

}

impl Into<TRB> for CommandTRB {
    fn into(self) -> TRB {
        TRB { command: self }
    }
}

/// Common to all Transfer TRBs
#[bitfield]
#[derive(Copy, Clone, Debug, Default)]
pub struct TransferTRBDW3 {
    transfer_size: B17,
    td_size: B5,
    interrupter: B10,
}

/// Normal TRB
#[bitfield]
#[derive(Debug, Copy, Clone, Default)]
pub struct NormalTRBDW4 {
    cycle: bool,
    eval_next: bool,
    int_on_short_packet: bool,
    ns: bool,
    chain: bool,
    ioc: bool,
    imm: bool,
    _res0: B2,
    block_event_interrupt: bool,
    trb_type: B6,
    _res1: B16,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct NormalTRB {
    pub buffer: u64,
    pub int_len: TransferTRBDW3,
    pub meta: NormalTRBDW4,
}

impl NormalTRB {
    pub fn new<'a>(hal: &'a dyn HAL, buf: &[u8], max_len: usize) -> Self {
        // TODO Support max len
        assert!(buf.len() <= max_len, "exceed max len not supported");
        let buf_ptr =  hal.translate_addr(buf.as_ptr() as u64);
        let mut thing = Self {
            buffer: buf_ptr,
            int_len: Default::default(),
            meta: Default::default(),
        };
        thing.int_len.set_transfer_size(buf.len() as u32);
        thing.int_len.set_td_size(0);
        thing.meta.set_ioc(true);
        thing.meta.set_trb_type(TRB_TYPE_NORMAL as u8);
        thing
    }
}

/* ------- Setup TRB ------------- */
#[bitfield]
#[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
pub struct SetupStageDW3 {
    trb_length: B17,
    _res0: B5,
    interrupter: B10,
}


#[bitfield]
#[derive(PartialEq, Eq, Debug, Copy, Clone, Default)]
pub struct SetupStageDW4 {
    cycle: bool,
    _res0: B4,
    ioc: bool,
    imm: bool,
    _res1: B3,
    trb_type: B6,
    trt: B2,
    _res2: B14,
}

#[repr(C)]
#[derive(Default, Copy, Clone)]
pub struct SetupStageTRB {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
    /// Interrupt Target: (31:22), TRBLength (16:0)
    pub int_target_trb_length: SetupStageDW3,
    /// Transfer Type: (17:16)
    /// > 0: No Data, 1: Reserved, 2: Out Data, 3: In Data
    /// Transfer Type (15:10)
    /// > Should be set to Setup Stage TRB Type (XHCI Table 6-91)
    /// immData (6:6)
    /// Interrupt On Complete (5:5)
    /// Cycle Bit (0:0)
    pub metadata: SetupStageDW4,
}

/* ----------- Data Stage TRB --------------- */

#[bitfield]
#[derive(Copy, Clone, Debug, Default)]
pub struct DataStageDW4 {
    cycle: bool,
    eval_next: bool,
    isp: bool,
    ns: bool,
    chain: bool,
    ioc: bool,
    idt: bool,
    _res: B3,
    trb_type: B6,
    read: bool,
    _res1: B15,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct DataStageTRB {
    pub buffer: u64,
    pub params: TransferTRBDW3,
    pub meta: DataStageDW4,
}

impl Default for DataStageTRB {
    fn default() -> Self {
        Self {
            buffer: 0,
            params: Default::default(),
            meta: Default::default(),
        }
    }
}

/* -------------- Event Data TRB ---------- */

#[bitfield]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct EventDataTRBDW4 {
    cycle: bool,
    eval_next: bool,
    _res: B2,
    chain: bool,
    ioc: bool,
    _res2: B3,
    block_event_interrupt: bool,
    trb_type: B6,
    _res3: B16,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct EventDataTRB {
    pub ptr: u64,
    interrupter: u32,
    // This needs to change to a bitfield, it's wrong
    pub meta: EventDataTRBDW4,
}

/* ------------- Status Stage TRB ----------- */
#[bitfield]
#[derive(Default, Debug, Copy, Clone)]
pub struct StatusStageTRBDW3 {
    _res: B22,
    interrupter: B10,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct StatusStageTRB {
    _resz: u64,
    pub interrupter: StatusStageTRBDW3,
    pub meta: DataStageDW4, // This is the same as DataStageDW4, so reused
}