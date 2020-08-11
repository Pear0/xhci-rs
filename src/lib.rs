#![feature(allocator_api)]
#![feature(const_in_array_repeat_expressions)]
#![feature(global_asm)]
#![feature(llvm_asm)]

#![allow(dead_code, unused_imports, unused_parens)]

#![cfg_attr(not(test), no_std)]

// TODO: https://docs.rs/downcast-rs/1.2.0/downcast_rs/index.html for USB implementation specifics

extern crate alloc;
#[macro_use]
extern crate log;

use alloc::alloc::{AllocInit, AllocRef, Global, Layout};
use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::ops::Deref;
use core::ptr::NonNull;
use core::time::Duration;

use hashbrown::HashMap;
use spin::{Mutex, RwLock};
use volatile::*;

use usb_host::descriptor::*;
use usb_host::items::{Port, EndpointType};

use crate::consts::*;
use crate::descriptor::{USBConfigurationDescriptor, USBConfigurationDescriptorSet, USBDeviceDescriptor, USBEndpointDescriptor, USBHubDescriptor, USBInterfaceDescriptor, USBInterfaceDescriptorSet};
use crate::extended_capability::{ExtendedCapabilityTag, ExtendedCapabilityTags};
use crate::items::{Port, TransferDirection};
use crate::quirks::XHCIQuirks;
use crate::registers::{DoorBellRegister, InterrupterRegisters};
use crate::structs::{DeviceContextArray, DeviceContextBaseAddressArray, EventRingSegmentTable, InputContext, PortStatus, ScratchPadBufferArray, SlotContext, XHCIRing, XHCIRingSegment};
use crate::trb::{CommandCompletionTRB, CommandTRB, DataStageTRB, EventDataTRB, NormalTRB, SetupStageTRB, StatusStageTRB, TransferEventTRB, TRB, TRBType};
use crate::trb::TRBType::TransferEvent;
use usb_host::traits::{USBHostController, USBPipe, USBMeta};
use usb_host::structs::USBDevice;
use downcast_rs::Downcast;

pub mod quirks;
#[macro_use]
pub(crate) mod consts;
pub(crate) mod extended_capability;
pub(crate) mod registers;
pub(crate) mod structs;
pub(crate) mod trb;

#[derive(Clone, Debug)]
pub enum Error {
    Str(&'static str),
    Completion(TRBCompletionCode),
}

impl From<&'static str> for Error {
    fn from(s: &'static str) -> Self {
        Error::Str(s)
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct XHCICapabilityRegisters {
    length_and_ver: ReadOnly<u32>,
    hcs_params: [ReadOnly<u32>; 3],
    hcc_param1: ReadOnly<u32>,
    doorbell_offset: ReadOnly<u32>,
    rts_offset: ReadOnly<u32>,
    hcc_param2: ReadOnly<u32>,
}

#[derive(Debug)]
#[repr(C)]
pub struct XHCIOperationalRegisters {
    command: Volatile<u32>,
    status: Volatile<u32>,
    /// Page size is 2^(n+12), n being the value read
    page_size: ReadOnly<u32>,
    _res1: [u32; 2],
    dnctlr: Volatile<u32>,
    /// 63:6 pointer | 5:4 res | Command Ring Running
    /// | Command Abort | Command Stop | RingCycleState
    command_ring_control: Volatile<u64>,
    _res2: [u32; 4],
    device_context_base_addr_array_ptr: Volatile<u64>,
    config: Volatile<u32>,
}


impl XHCIOperationalRegisters {
    pub fn as_ptr(&mut self) -> *mut Self {
        self as *mut Self
    }

    pub fn get_port_operational_register(&self, port: u8) -> &'static mut XHCIPortOperationalRegisters {
        assert_ne!(port, 0, "port can't be zero");
        unsafe { &mut *(((self as *const Self as usize) + 0x400 + 0x10 * (port as usize - 1)) as *mut XHCIPortOperationalRegisters) }
    }
}

#[repr(C)]
pub struct XHCIPortOperationalRegisters {
    /// Port Status & Ctrl
    pub portsc: Volatile<u32>,
    /// Port Power Management Status & Ctrl
    pub portpmsc: Volatile<u32>,
    /// Port Link Info
    pub portli: Volatile<u32>,
    /// Hardware LMP Control
    pub porthwlpmc: Volatile<u32>,
}

fn get_registers<T>(base: u64) -> &'static mut T {
    unsafe { &mut *(base as *mut T) }
}

fn as_slice<T>(t: &mut T) -> &mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(t as *mut T as *mut u8, core::mem::size_of::<T>()) }
}

#[derive(Copy, Clone, Debug)]
pub enum FlushType {
    /// Flush dirty cache lines to RAM (does not drop cache lines)
    Clean,
    /// Drop (potentially dirty) cache lines without writing back to RAM
    Invalidate,
    /// Clean then Drop
    CleanAndInvalidate,
}

pub trait HAL {
    fn current_time(&self) -> Duration;
    fn sleep(&self, dur: Duration);
    fn memory_barrier(&self);
    fn translate_addr(&self, addr: u64) -> u64;
    fn flush_cache(&self, addr: u64, len: u64, flush: FlushType);

    fn alloc_noncached(&self, layout: Layout) -> Option<u64> {
        Global.alloc(layout, AllocInit::Zeroed).ok().map(|x| x.ptr.as_ptr() as u64)
    }

    fn free_noncached(&self, ptr: u64, layout: Layout) {
        unsafe {
            Global.dealloc(NonNull::new_unchecked(ptr as *mut u8), layout);
        }
    }
}

pub struct Xhci<'a> {
    mmio_virt_base: u64,
    cap: &'static mut XHCICapabilityRegisters,
    op: &'static mut XHCIOperationalRegisters,
    hal: &'a dyn HAL,
    info: XHCIInfo,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    device_contexts: [Option<Box<DeviceContextArray>>; 255],

    // (slot, endpoint)
    transfer_rings: HashMap<(u8, u8), Arc<Mutex<XHCIRing<'a>>>>,

    command_ring: Option<XHCIRing<'a>>,
    event_ring: Option<XHCIRing<'a>>,
    event_ring_table: Option<Box<EventRingSegmentTable>>,
    scratchpads: Option<Box<ScratchPadBufferArray>>,
    pub quirks: XHCIQuirks,
}

#[derive(Default)]
pub struct XHCIInfo {
    max_slot: u8,
    /// This field also double as the num of ports,
    /// since the port number starts from 1
    max_port: u8,
    page_size: u32,
    big_context: bool,
    doorbell_offset: u32,
    runtime_offset: u32,
}

impl<'a> Xhci<'a> {
    pub fn new(base_address: u64, hal: &'a dyn HAL) -> Self {
        let cap = get_registers::<XHCICapabilityRegisters>(base_address);
        let cap_size = (cap.length_and_ver.read() & 0xFF) as u64;

        let op_regs = get_registers::<XHCIOperationalRegisters>(base_address + cap_size);

        info!("Page Size: {}", op_regs.page_size.read());
        info!("Status: {:#x}", op_regs.status.read());

        let mut info = XHCIInfo::default();
        info.doorbell_offset = cap.doorbell_offset.read() & CAP_DBOFFSET_MASK;
        info.runtime_offset = cap.rts_offset.read() & CAP_RTSOFFSET_MASK;

        Self {
            mmio_virt_base: base_address,
            cap,
            op: op_regs,
            hal,
            info,
            device_context_baa: None,
            device_contexts: [None; 255],
            transfer_rings: HashMap::new(),
            command_ring: None,
            event_ring: None,
            event_ring_table: None,
            scratchpads: None,
            quirks: Default::default(),
        }
    }

    fn get_runtime_interrupt_register(&mut self, offset: u8) -> &'static mut InterrupterRegisters {
        let base_ptr = self.cap as *const XHCICapabilityRegisters as u64;
        unsafe {
            &mut *((base_ptr + (self.info.runtime_offset as u64)
                + 0x20 + (offset as u64) * 0x20) as *mut InterrupterRegisters)
        }
    }

    fn get_doorbell_register(&mut self, offset: u8) -> &'static mut DoorBellRegister {
        let base_ptr = self.cap as *const XHCICapabilityRegisters as u64;
        unsafe {
            &mut *((base_ptr + (self.info.doorbell_offset as u64) +
                (offset as u64 * 4)) as *mut DoorBellRegister)
        }
    }

    fn wait_until<E, F: Fn(&mut Self) -> bool>(&mut self, e: E, timeout: Duration, func: F) -> Result<(), E> {
        let wait_limit = self.hal.current_time() + timeout;
        loop {
            if func(self) {
                return Ok(());
            }
            if self.hal.current_time() > wait_limit {
                return Err(e);
            }
            self.hal.sleep(Duration::from_millis(1));
        }
    }

    fn flush_trb(&self, ptr: u64, flush: FlushType) {
        self.hal.flush_cache(ptr, 16, flush);
    }

    fn flush_struct<T>(&self, val: &T, flush: FlushType) {
        self.hal.flush_cache(val as *const T as u64, core::mem::size_of_val(val) as u64, flush);
    }

    fn flush_slice<T>(&self, val: &[T], flush: FlushType) {
        self.hal.flush_cache(val.as_ptr() as u64, core::mem::size_of_val(val) as u64, flush);
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.op.command.update(|x| *x |= OP_CMD_RESET_MASK);
        self.hal.memory_barrier();

        self.wait_until(Error::Str("did not reset"), RESET_TIMEOUT, |this| {
            let cmd = this.op.command.read();
            let sts = this.op.status.read();
            (cmd & OP_CMD_RESET_MASK == 0) && (sts & OP_STS_CNR_MASK == 0)
        })?;

        Ok(())
    }

    fn get_ptr<T>(&self, t: &T) -> u64 {
        self.hal.translate_addr(t as *const T as u64)
    }

    fn do_reset_root_port(&mut self, port_id: u8) -> Result<(), Error> {
        self.op.get_port_operational_register(port_id).portsc.update(|tmp| {
            *tmp &= !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
            *tmp |= OP_PORT_STATUS_RESET_MASK | OP_PORT_STATUS_PRC_MASK;
        });

        self.wait_until(Error::Str("failed to reset port"), PORT_RESET_TIMEOUT, |this| {
            let val = this.op.get_port_operational_register(port_id).portsc.read();
            (val & OP_PORT_STATUS_RESET_MASK == 0) && (OP_PORT_STATUS_PRC_MASK != 0)
        })?;

        self.op.get_port_operational_register(port_id).portsc.update(|tmp| {
            *tmp &= !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
            *tmp |= OP_PORT_STATUS_PRC_MASK;
        });

        debug!("[XHCI] port {} reset", port_id);
        Ok(())
    }

    fn reset_port(&mut self, port: &Port) -> Result<(), Error> {
        match &port.parent {
            None => self.do_reset_root_port(port.port_id),
            Some(parent) => {
                self.set_feature(parent.slot_id, port.port_id, FEATURE_PORT_RESET)?;

                self.wait_until("failed to reset port", PORT_RESET_TIMEOUT, |this| {
                    if let Ok(status) = this.fetch_port_status(parent.slot_id, port.port_id) {
                        status.get_change_reset()
                    } else {
                        false
                    }
                })?;

                self.clear_feature(parent.slot_id, port.port_id, FEATURE_C_PORT_RESET)?;

                // self.reset_tt(parent.slot_id, REQUEST_RESET_TT_DEFAULT_TT)?;
                // self.hal.sleep(Duration::from_millis(15));

                Ok(())
            }
        }
    }

    fn get_speed(&mut self, port: &Port) -> Result<u8, Error> {
        match &port.parent {
            None => {
                let portsc = self.op.get_port_operational_register(port.port_id).portsc.read();
                Ok(((portsc & OP_PORT_STATUS_SPEED_MASK) >> OP_PORT_STATUS_SPEED_SHIFT) as u8)
            }
            Some(parent) => {
                let status = self.fetch_port_status(parent.slot_id, port.port_id)?;
                if status.get_low_speed() {
                    Ok(OP_PORT_STATUS_SPEED_LOW)
                } else if status.get_high_speed() {
                    Ok(OP_PORT_STATUS_SPEED_HIGH)
                } else {
                    Ok(OP_PORT_STATUS_SPEED_FULL)
                }
            }
        }
    }

    fn send_slot_enable(&mut self) -> Result<u8, Error> {
        let cmd = CommandTRB::enable_slot();
        let ptr = self.command_ring.as_mut()
            .expect("no cmd ring found").push(cmd.into());
        trace!("[XHCI] Sending Slot EN");

        match self.wait_command_complete(ptr) {
            Some(trb) => Ok(trb.slot),
            _ => Err("USBError::CommandTimeout".into())
        }
    }

    fn get_epctx_index(&self, endpoint_address: u8) -> u8 {
        let is_in = Self::is_ep_input(endpoint_address);
        if endpoint_address == 0 {
            0
        } else {
            (endpoint_address * 2) - if is_in { 0 } else { 1 }
        }
    }

    fn is_ep_input(endpoint_address: u8) -> bool {
        endpoint_address & 0x80 != 0
    }

    fn configure_endpoint(&mut self, slot_id: u8, input_ctx: &mut InputContext,
                          endpoint_address: u8, endpoint_type: u8, max_packet_size: u16,
                          interval: u8, esit_payload_size: u32) -> (u8, u8) {
        let transfer_ring = XHCIRing::new_with_capacity(self.hal, 1, true);
        let transfer_ring_ptr = self.get_ptr::<XHCIRingSegment>(transfer_ring.segments[0].as_ref());
        assert_eq!(transfer_ring_ptr & 0b1111, 0, "alignment");
        trace!("[XHCI] Setting Transfer Ring Pointer to {:#x}", transfer_ring_ptr);

        let index = self.get_epctx_index(endpoint_address);

        // TODO: Rethink populate max_entries field
        let current_entries = input_ctx.get_slot_mut().dword1.get_context_entries();
        if index + 1 > current_entries {
            input_ctx.get_slot_mut().dword1.set_context_entries(index + 1);
        }

        debug!("configure_endpoint(slot_id: {}, raw_ep: {}, index: {})", slot_id, endpoint_address, index);

        let epctx = input_ctx.get_endpoint_mut(index as usize);
        if let Some(ctx) = self.device_contexts[slot_id as usize - 1].as_ref() {
            *epctx = ctx.get_endpoint(index as usize).clone();
        }
        epctx.set_lsa_bit(); // Disable Streams
        epctx.set_cerr(3); // Max value (2 bit only)
        epctx.set_ep_type(endpoint_type);
        epctx.set_interval(interval);
        assert_ne!(max_packet_size, 0);
        epctx.max_packet_size = max_packet_size;
        epctx.average_trb_len = if endpoint_type == EP_TYPE_CONTROL_BIDIR { 8 } else { esit_payload_size as u16 };
        epctx.max_esit_payload_lo = esit_payload_size as u16;
        epctx.dequeu_pointer = transfer_ring_ptr | (transfer_ring.cycle_state & 0x1) as u64; // Cycle Bit

        // update input context adds/removes
        input_ctx.get_input_mut()[1] |= 1;
        input_ctx.get_input_mut()[1] |= 0b1 << (index + 1);

        self.transfer_rings.insert((slot_id, index), Arc::new(Mutex::new(transfer_ring)));
        (slot_id, index)
    }

    fn create_slot_context(&mut self, input_ctx: &mut InputContext, port: &Port, speed: u8, max_packet_size: u16) {
        input_ctx.get_slot_mut().dword1.set_speed(speed);
        input_ctx.get_slot_mut().dword1.set_context_entries(1); // TODO Maybe not hardcode 1?
        input_ctx.get_slot_mut().root_hub_port_number = port.get_root_port_id();
        input_ctx.get_slot_mut().dword1.set_route_string(port.construct_route_string());
        input_ctx.get_slot_mut().slot_state = 0;

        if let Some(parent) = port.parent.as_ref() {
            if port.is_low_or_full_speed && !parent.is_low_or_full_speed {
                input_ctx.get_slot_mut().parent_hub_slot_id = parent.slot_id;
                input_ctx.get_slot_mut().parent_port_number = port.port_id;
            }
        }
    }

    fn setup_slot(&mut self, port: &Port, speed: u8, max_packet_size: u16, block_cmd: bool) -> Box<InputContext> {
        debug!("setup_slot(port_id: {}, speed: {}, max_packet_size: {}, block_cmd: {})", port.port_id, speed, max_packet_size, block_cmd);
        let slot = port.slot_id as usize;

        let mut input_ctx = Box::new(if self.info.big_context { InputContext::new_big() } else { InputContext::new_normal() });

        if let Some(output_ctx) = self.device_contexts[port.slot_id as usize].as_ref() {
            self.flush_struct::<DeviceContextArray>(output_ctx.as_ref(), FlushType::Invalidate);
            // Clone slot context from Output -> Input
            *input_ctx.get_slot_mut() = output_ctx.get_slot().clone();

            // Clone CtrlEP0
            *input_ctx.get_endpoint_mut(0) = output_ctx.get_endpoint(0).clone();
        } else {
            let mut dev_ctx = Box::new(if self.info.big_context { DeviceContextArray::new_big() } else { DeviceContextArray::new_normal() });
            let ctx_ptr = self.get_ptr::<DeviceContextArray>(dev_ctx.as_ref());

            self.flush_struct::<DeviceContextArray>(dev_ctx.as_ref(), FlushType::Clean);

            // Activate Entry
            self.device_contexts[slot - 1] = Some(dev_ctx);
            self.device_context_baa.as_mut().expect("").entries[slot] = ctx_ptr;

            self.flush_struct::<DeviceContextBaseAddressArray>(self.device_context_baa.as_ref().unwrap(), FlushType::Clean);
        }

        self.create_slot_context(&mut input_ctx, port, speed, max_packet_size);
        self.configure_endpoint(port.slot_id, input_ctx.as_mut(),
                                0, EP_TYPE_CONTROL_BIDIR, max_packet_size,
                                0, 0);

        input_ctx.get_input_mut()[1] = 0b11;

        self.hal.flush_cache(input_ctx.get_ptr_va(), input_ctx.get_size() as u64, FlushType::Clean);

        let input_ctx_ptr = self.hal.translate_addr(input_ctx.get_ptr_va());
        let ptr = self.command_ring.as_mut().expect("").push(
            TRB { command: CommandTRB::address_device(slot as u8, input_ctx_ptr, block_cmd) }
        );
        self.wait_command_complete(ptr).expect("command_complete");

        self.hal.sleep(Duration::from_millis(10));

        input_ctx
    }

    fn send_control_command(&mut self, slot_id: u8, request_type: TypeTriple, request: u8,
                            value: u16, index: u16, length: u16,
                            write_to_usb: Option<&[u8]>, read_from_usb: Option<&mut [u8]>)
                            -> Result<usize, Error>
    {
        let mut trbs: Vec<TRB> = Vec::new();
        let mut min_length = length;

        if let Some(write) = write_to_usb {
            self.flush_slice(write, FlushType::Clean);
            min_length = core::cmp::min(min_length, write.len() as u16);
        }

        if let Some(read) = &read_from_usb {
            self.flush_slice(read, FlushType::Invalidate);
            min_length = core::cmp::min(min_length, read.len() as u16);
        }

        let setup_trt = if write_to_usb.is_none() && read_from_usb.is_none() {
            0u8
        } else if write_to_usb.is_some() && read_from_usb.is_none() {
            2u8
        } else if read_from_usb.is_some() && write_to_usb.is_none() {
            3u8
        } else {
            return Err("USBError::InvalidArgument".into());
        };
        let mut setup = SetupStageTRB {
            request_type: request_type.into(),
            request,
            value,
            index,
            length: min_length,
            int_target_trb_length: Default::default(),
            metadata: Default::default(),
        };
        setup.metadata.set_imm(true);
        setup.metadata.set_trb_type(TRB_TYPE_SETUP as u8);
        setup.metadata.set_trt(setup_trt);
        setup.int_target_trb_length.set_trb_length(8); // Always 8: Section 6.4.1.2.1, Table 6-25
        trbs.push(TRB { setup });

        if write_to_usb.is_some() || read_from_usb.is_some() {
            // Data TRB
            let mut data = DataStageTRB::default();
            data.buffer = if write_to_usb.is_some() {
                self.hal.translate_addr(write_to_usb.as_ref().unwrap().as_ptr() as u64)
            } else {
                self.hal.translate_addr(read_from_usb.as_ref().unwrap().as_ptr() as u64)
            };
            data.params.set_transfer_size(min_length as u32);

            // Calculate TDSize
            let max = (1u32 << (21 - 17 + 1)) - 1;
            let td_size = core::cmp::min((min_length as u32) >> 10, max);
            data.params.set_td_size(td_size as u8);

            if read_from_usb.is_some() {
                data.meta.set_isp(true); // Interrupt on Short Packet only for IN
            }
            data.meta.set_read(read_from_usb.is_some());
            data.meta.set_trb_type(TRB_TYPE_DATA as u8);
            // data.meta.set_eval_next(true);
            // data.meta.set_chain(true);

            trbs.push(TRB { data });
        }

        // Status TRB
        let mut status_stage = StatusStageTRB::default();
        status_stage.meta.set_trb_type(TRB_TYPE_STATUS as u8);
        status_stage.meta.set_ioc(true);
        // status_stage.meta.set_eval_next(true);
        // status_stage.meta.set_chain(true);
        if !(min_length > 0 && read_from_usb.is_some()) {
            // status stage read is the reverse of the data stage read.
            // See Table 4-7 in the Intel xHCI manual
            status_stage.meta.set_read(true);
        }
        trbs.push(TRB { status_stage });

        // Event Data TRB
        // let mut event_data = EventDataTRB::default();
        // event_data.meta.set_trb_type(TRB_TYPE_EVENT_DATA as u8);
        // trbs.push(TRB { event_data });

        {
            let mut lock = self.transfer_rings.get(&(slot_id, 0)).as_ref().unwrap().lock();
            lock.push_group(trbs.as_slice());
        }

        // Section 5.6: Table 5-43: Doorbell values
        self.get_doorbell_register(slot_id).reg.write(1); // CTRL EP DB is 1

        loop {
            let result = self.poll_event_ring_trb();
            if let Some(trb) = result {
                match trb {
                    TRBType::TransferEvent(t) => {
                        let code = TRBCompletionCode::from(t.status.get_code());
                        debug!("Transfer Complete: status = {:?}", code);
                        if t.status.get_code() != TRBCompletionCode::ShortPacket as u8 {
                            if !matches!(code, TRBCompletionCode::Success) {
                                {
                                    let mut lock = self.transfer_rings.get(&(slot_id, 0)).as_ref().unwrap().lock();
                                    lock.push_group(trbs.as_slice());
                                }

                                return Err(Error::Completion(code));
                            }
                        } else {
                            warn!("ControlMessage Short Packet Detected, {:?}", t);
                        }
                        let bytes_remain = t.status.get_bytes_remain() as usize;
                        let bytes_requested = if write_to_usb.is_some() {
                            write_to_usb.unwrap().len()
                        } else if read_from_usb.is_some() {
                            read_from_usb.as_ref().unwrap().len()
                        } else {
                            0
                        };

                        if let Some(read) = read_from_usb {
                            self.flush_slice(read, FlushType::Invalidate);
                        }

                        return Ok(bytes_requested - bytes_remain);
                    }
                    _ => {
                        debug!("[XHCI] Unexpected TRB: {:?}", &trb);
                    }
                }
            } else {
                error!("[XHCI] Poll TRB timedout");
                return Err(Error::Str("USBError::ControlEndpointTimeout"));
            }
        }
    }

    fn send_transfer(&mut self, slot_id: u8, endpoint: u8, request_type: u8, request: u8,
                     value: u16, index: u16, length: u16, transfer: TransferDirection)
                     -> Result<usize, Error>
    {
        match &transfer {
            // TODO maybe don't always use DC CIVAC...
            TransferDirection::Read(read) => self.flush_slice(read, FlushType::Invalidate),
            TransferDirection::Write(write) => self.flush_slice(write, FlushType::Clean),
            _ => {}
        }

        let setup_trt = match &transfer {
            TransferDirection::Read(_) => 3,
            TransferDirection::Write(_) => 2,
            TransferDirection::None => 0,
        };

        let mut setup = SetupStageTRB {
            request_type,
            request,
            value,
            index,
            length,
            int_target_trb_length: Default::default(),
            metadata: Default::default(),
        };
        setup.metadata.set_imm(true);
        setup.metadata.set_trb_type(TRB_TYPE_SETUP as u8);
        setup.metadata.set_trt(setup_trt);
        setup.int_target_trb_length.set_trb_length(8); // Always 8: Section 6.4.1.2.1, Table 6-25

        {
            let ring = self.transfer_rings.get(&(slot_id, endpoint)).ok_or("unknown (slot, endpoint)")?;
            let mut ring = ring.lock();
            ring.push(TRB { setup });
        }

        if !matches!(&transfer, TransferDirection::None) {
            // Data TRB
            let mut data = DataStageTRB::default();

            match &transfer {
                TransferDirection::Read(read) => {
                    data.buffer = self.hal.translate_addr(read.as_ptr() as u64);
                }
                TransferDirection::Write(write) => {
                    data.buffer = self.hal.translate_addr(write.as_ptr() as u64);
                }
                TransferDirection::None => panic!(),
            }

            data.params.set_transfer_size(match &transfer {
                TransferDirection::Read(read) => read.len() as u32,
                TransferDirection::Write(write) => write.len() as u32,
                TransferDirection::None => 0,
            });
            data.meta.set_read(matches!(&transfer, TransferDirection::Read(_)));
            data.meta.set_trb_type(TRB_TYPE_DATA as u8);
            // TODO sketchies
            // data.meta.set_eval_next(true);
            // data.meta.set_chain(true);

            {
                let ring = self.transfer_rings.get(&(slot_id, endpoint)).ok_or("unknown (slot, endpoint)")?;
                let mut ring = ring.lock();
                ring.push(TRB { data });
            }
        }
        // Status TRB
        let mut status_stage = StatusStageTRB::default();
        status_stage.meta.set_trb_type(TRB_TYPE_STATUS as u8);
        status_stage.meta.set_ioc(true);

        {
            let ring = self.transfer_rings.get(&(slot_id, endpoint)).ok_or("unknown (slot, endpoint)")?;
            let mut ring = ring.lock();
            ring.push(TRB { status_stage });
        }

        // Event Data TRB
        let mut event_data = EventDataTRB::default();
        event_data.meta.set_trb_type(TRB_TYPE_EVENT_DATA as u8);

        {
            let ring = self.transfer_rings.get(&(slot_id, endpoint)).ok_or("unknown (slot, endpoint)")?;
            let mut ring = ring.lock();
            ring.push(TRB { event_data });
        }


        // Section 5.6: Table 5-43: Doorbell values
        self.get_doorbell_register(slot_id).reg.write(1); // CTRL EP DB is 1

        loop {
            let result = self.poll_event_ring_trb();
            if let Some(trb) = result {
                match trb {
                    TRBType::TransferEvent(t) => {
                        let bytes_remain = t.status.get_bytes_remain() as usize;
                        let bytes_requested = match &transfer {
                            TransferDirection::Read(read) => read.len(),
                            TransferDirection::Write(write) => write.len(),
                            TransferDirection::None => 0,
                        };

                        if let TransferDirection::Read(read) = &transfer {
                            self.flush_slice(read, FlushType::Invalidate);
                        }

                        return Ok(bytes_requested - bytes_remain);
                    }
                    _ => {
                        debug!("[XHCI] Unexpected TRB: {:?}", &trb);
                    }
                }
            } else {
                error!("[XHCI] Poll TRB timedout");
                return Err(Error::Str("USBError::ControlEndpointTimeout"));
            }
        }
    }

    fn fetch_descriptor(&mut self, slot_id: u8, desc_type: u8, desc_index: u8,
                        w_index: u16, buf: &mut [u8]) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(DeviceToHost, Standard, Device)), 0x80u8);
        self.send_control_command(slot_id, request_type!(DeviceToHost, Standard, Device), REQUEST_GET_DESCRIPTOR,
                                  ((desc_type as u16) << 8) | (desc_index as u16),
                                  w_index, buf.len() as u16, None, Some(buf),
        )
    }

    fn fetch_class_descriptor(&mut self, slot_id: u8, desc_type: u8, desc_index: u8,
                              w_index: u16, buf: &mut [u8]) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(DeviceToHost, Class, Device)), 0xA0u8);
        self.send_control_command(slot_id, request_type!(DeviceToHost, Class, Device), REQUEST_GET_DESCRIPTOR,
                                  ((desc_type as u16) << 8) | (desc_index as u16),
                                  w_index, buf.len() as u16, None, Some(buf),
        )
    }

    fn fetch_hub_descriptor(&mut self, slot_id: u8, device_descriptor: &USBDeviceDescriptor) -> Result<USBHubDescriptor, Error> {
        let mut hub_descriptor = USBHubDescriptor::default();
        let desc_type = if device_descriptor.get_max_packet_size() >= 512 { DESCRIPTOR_TYPE_SS_HUB } else { DESCRIPTOR_TYPE_HUB };

        self.fetch_class_descriptor(slot_id, desc_type, 0, 0, &mut (as_slice(&mut hub_descriptor))[..4])?;

        let length = hub_descriptor.length as usize;
        self.fetch_class_descriptor(slot_id, desc_type, 0, 0, &mut (as_slice(&mut hub_descriptor))[..length])?;

        Ok(hub_descriptor)
    }

    fn fetch_hid_report(&mut self, slot_id: u8, desc_type: u8, desc_index: u8,
                        w_index: u16, buf: &mut [u8]) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(DeviceToHost, Class, Interface)), 0xA1u8);
        self.send_control_command(slot_id, request_type!(DeviceToHost, Class, Interface), REQUEST_GET_REPORT,
                                  ((desc_type as u16) << 8) | (desc_index as u16),
                                  w_index, buf.len() as u16, None, Some(buf),
        )
    }

    fn set_hid_report(&mut self, slot_id: u8, desc_type: u8, desc_index: u8,
                      w_index: u16, value: u8) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Class, Interface)), 0x21u8);
        self.send_control_command(slot_id, request_type!(HostToDevice, Class, Interface), REQUEST_SET_REPORT,
                                  ((desc_type as u16) << 8) | (desc_index as u16),
                                  w_index, 1, Some(core::slice::from_ref(&value)), None)
    }

    fn set_hid_idle(&mut self, slot_id: u8) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Class, Interface)), 0x21u8);
        self.send_control_command(slot_id, request_type!(HostToDevice, Class, Interface), REQUEST_SET_IDLE, 0,
                                  0, 0, None, None)
    }

    fn set_hid_protocol(&mut self, slot_id: u8, value: u8) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Class, Interface)), 0x21u8);
        self.send_control_command(slot_id, request_type!(HostToDevice, Class, Interface), REQUEST_SET_PROTOCOL, value as u16,
                                  0, 0, None, None)
    }

    fn set_feature(&mut self, slot_id: u8, port_id: u8, feature: u8) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Class, Other)), 0x23u8);
        self.send_control_command(slot_id, request_type!(HostToDevice, Class, Other), REQUEST_SET_FEATURE, feature as u16,
                                  port_id as u16, 0, None, None)
    }

    fn clear_feature(&mut self, slot_id: u8, port_id: u8, feature: u8) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Class, Other)), 0x23u8);
        self.send_control_command(slot_id, request_type!(HostToDevice, Class, Other), REQUEST_CLEAR_FEATURE, feature as u16,
                                  port_id as u16, 0, None, None)
    }

    fn reset_tt(&mut self, slot_id: u8, tt_id: u16) -> Result<usize, Error>
    {
        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Class, Other)), 0x23u8);
        self.send_control_command(slot_id, request_type!(HostToDevice, Class, Other), REQUEST_RESET_TT, 0,
                                  tt_id, 0, None, None)
    }

    fn fetch_port_status(&mut self, slot_id: u8, port_id: u8) -> Result<PortStatus, Error> {
        let mut status = PortStatus::default();
        assert_eq!(Into::<u8>::into(request_type!(DeviceToHost, Class, Other)), 0xA3u8);
        self.send_control_command(slot_id, request_type!(DeviceToHost, Class, Other), REQUEST_GET_STATUS, 0, port_id as u16, 4, None, Some(as_slice(&mut status)))?;
        Ok(status)
    }

    fn fetch_device_descriptor(&mut self, slot_id: u8) -> Result<USBDeviceDescriptor, Error> {
        let mut buf2 = [0u8; 18];
        self.fetch_descriptor(slot_id, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut buf2)?;
        Ok(unsafe { core::mem::transmute(buf2) })
    }

    fn fetch_string_descriptor(&mut self, slot: u8, index: u8, lang: u16) -> Result<String, Error> {
        if index == 0 {
            return Err(Error::Str("invalid descriptor index 0"));
        }

        let mut buf = [0u8; 1];
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_STRING,
                              index, lang, &mut buf)?;
        if buf[0] == 0 {
            return Err(Error::Str("USBError::DescriptorNotAvailable"));
        }
        let mut buf2: Vec<u8> = Vec::new();
        buf2.resize(buf[0] as usize, 0);
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_STRING,
                              index, lang, &mut buf2)?;
        assert_eq!(buf2[1], DESCRIPTOR_TYPE_STRING);
        let buf2: Vec<u16> = buf2.chunks_exact(2)
            .map(|l| { u16::from_ne_bytes([l[0], l[1]]) }).collect();
        Ok(String::from_utf16_lossy(&buf2[1..]))
    }

    fn fetch_configuration_descriptor(&mut self, slot_id: u8) -> Result<USBConfigurationDescriptorSet, Error> {
        // use pretty_hex::*;
        let mut config_descriptor = [0u8; 9];
        self.fetch_descriptor(slot_id, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, &mut config_descriptor)?;
        let config: USBConfigurationDescriptor = unsafe { core::mem::transmute(config_descriptor) };
        let mut buf2: Vec<u8> = Vec::new();
        buf2.resize(config.get_total_length() as usize, 0);
        self.fetch_descriptor(slot_id, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, &mut buf2)?;
        let mut current_index = core::mem::size_of::<USBConfigurationDescriptor>();
        let mut interfaces: Vec<USBInterfaceDescriptorSet> = Default::default();
        let mut interface_set: Option<USBInterfaceDescriptorSet> = None;
        loop {
            if current_index + 2 > buf2.len() {
                if current_index != buf2.len() {
                    warn!("[USB] Descriptor not fully fetched");
                }
                break;
            }
            let desc_size = buf2[current_index] as usize;
            if desc_size == 0 {
                break;
            }
            let desc_type = buf2[current_index + 1];
            match desc_type {
                DESCRIPTOR_TYPE_INTERFACE => {
                    if interface_set.is_some() {
                        interfaces.push(interface_set.take().unwrap());
                    }
                    let desc: USBInterfaceDescriptor = Self::into_type(&buf2[current_index..current_index + desc_size]);
                    interface_set = Some(USBInterfaceDescriptorSet::new(desc));
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    let desc: USBEndpointDescriptor = Self::into_type(&buf2[current_index..current_index + desc_size]);
                    match &mut interface_set {
                        Some(ifset) => {
                            ifset.endpoints.push(desc);
                        }
                        _ => {
                            error!("[USB] EP Descriptor without IF");
                        }
                    }
                }
                _ => {
                    debug!("[USB] Unexpected descriptor type: {}", desc_type);
                }
            }
            current_index += desc_size;
        }
        if let Some(ifset) = interface_set {
            interfaces.push(ifset);
        }
        Ok(USBConfigurationDescriptorSet { config, ifsets: interfaces })
    }

    fn get_max_esti_payload(&self, epdesc: &USBEndpointDescriptor) -> u32 {
        use crate::descriptor::USBEndpointTransferType;
        match epdesc.transfer_type() {
            USBEndpointTransferType::Control |
            USBEndpointTransferType::Bulk => {
                0
            }
            _ => {
                let maxp_mult = (epdesc.max_packet_size >> 11) & 0x3;
                maxp_mult as u32 * epdesc.max_packet_size as u32
            }
        }
    }

    fn with_input_context<R, F: FnOnce(&mut Self, &mut InputContext) -> Result<R, Error>>(&mut self, port: &Port, func: F) -> Result<R, Error> {
        let mut input_ctx = Box::new(if self.info.big_context { InputContext::new_big() } else { InputContext::new_normal() });

        let ctx = self.device_contexts[(port.slot_id - 1) as usize].as_ref().expect("No context for slot");
        self.hal.flush_cache(ctx.get_ptr_va(), ctx.get_size() as u64, FlushType::Invalidate);

        *input_ctx.get_slot_mut() = ctx.get_slot().clone();
        // 31 is not a typo
        for i in 0..31 {
            *input_ctx.get_endpoint_mut(i) = ctx.get_endpoint(i).clone();
        }

        let result = func(self, input_ctx.as_mut())?;

        // always update slot context
        input_ctx.get_input_mut()[1] |= 1;

        self.hal.flush_cache(input_ctx.get_ptr_va(), input_ctx.get_size() as u64, FlushType::Clean);
        let input_ctx_ptr = self.hal.translate_addr(input_ctx.get_ptr_va());

        let ptr = self.command_ring.as_mut().expect("").push(
            TRB { command: CommandTRB::configure_endpoint(port.slot_id, input_ctx_ptr) }
        );
        self.wait_command_complete(ptr).ok_or(Error::Str("command did not complete"))?;
        Ok(result)
    }

    fn trigger_ring_doorbell(&mut self, ring: (u8, u8)) {
        self.get_doorbell_register(ring.0).reg.write(ring.1 as u32 + 1);
    }

    fn wait_for_transfer_event(&mut self, ptr: u64) -> Option<TransferEventTRB> {
        self.hal.memory_barrier();
        loop {
            let trb = self.poll_event_ring_trb()?;
            match trb {
                TRBType::TransferEvent(c) => {
                    if c.trb_pointer == ptr {
                        debug!("Got transfer event: {:?} -> bytes remain: {}", c, c.status.get_bytes_remain());
                        let code = c.status.get_code();
                        if code != 1 {
                            panic!("EHHHH WTF: {:?}", TRBCompletionCode::from(code));
                        }
                        return Some(c);
                    } else {
                        debug!("trb_pointer badddddd: {:?}", c);
                    }
                }
                e => {
                    debug!("Lol: {:?}", e);
                }
            }
        }
    }

    fn transfer_single_bulk_ring(&mut self, ring: (u8, u8), mut transfer: TransferDirection) -> Result<usize, Error> {
        debug!("transfer_single_bulk_ring(ring:{:?}, transfer:{:?})", ring, transfer);
        let mut my_read_buffer = Box::new([0u8; 512]);

        let max_packet_size = 64;
        let packet_size = core::cmp::min(max_packet_size, transfer.len());
        let my_buffer = &mut my_read_buffer[..packet_size];

        if let TransferDirection::Write(slice) = &transfer {
            my_buffer.copy_from_slice(&slice[..packet_size]);
            self.flush_slice(my_buffer.as_ref(), FlushType::Clean);
        }

        if let TransferDirection::Read(_) = &mut transfer {
            self.flush_slice(my_buffer.as_ref(), FlushType::Invalidate);
        }

        let transfer_ring = self.transfer_rings.get(&ring).unwrap();
        let mut transfer_ring = transfer_ring.lock();

        let normal = NormalTRB::new(self.hal, my_buffer.as_ref(), my_buffer.len());

        let ptr = transfer_ring.push(TRB { normal });

        core::mem::drop(transfer_ring);

        self.trigger_ring_doorbell(ring);

        let event = self.wait_for_transfer_event(ptr).ok_or(Error::Str("timed out waiting for transfer response"))?;
        let amount_transferred = packet_size - event.status.get_bytes_remain() as usize;

        if let TransferDirection::Read(slice) = &mut transfer {
            self.flush_slice(my_buffer.as_ref(), FlushType::Invalidate);
            (&mut slice[..amount_transferred]).copy_from_slice(&my_buffer[..amount_transferred]);
        }

        Ok(amount_transferred)
    }

    fn transfer_bulk_ring(&mut self, ring: (u8, u8), mut transfer: TransferDirection) -> Result<(), Error> {
        loop {
            if transfer.len() == 0 {
                return Ok(());
            }

            let amount = self.transfer_single_bulk_ring(ring, transfer.clone_mut())?;
            if amount == 0 {
                return Err(Error::Str("transferred zero bytes, giving up"));
            }

            transfer = match transfer {
                TransferDirection::Read(slice) => TransferDirection::Read(&mut slice[amount..]),
                TransferDirection::Write(slice) => TransferDirection::Write(&slice[amount..]),
                TransferDirection::None => TransferDirection::None,
            };
        }
    }

    fn setup_new_device(&mut self, port: &mut Port) -> Result<u8, Error> {
        self.reset_port(&port)?;
        let slot = self.send_slot_enable()? as usize;
        port.slot_id = slot as u8;
        info!("Got slot id: {}", slot);

        // Ok(slot as u8)

        let speed = self.get_speed(&port)?;
        let max_packet_size = match speed {
            0 => {
                error!("[XHCI] unknown device speed on port {}", port.port_id);
                64
            }
            OP_PORT_STATUS_SPEED_LOW => 8,
            OP_PORT_STATUS_SPEED_FULL |
            OP_PORT_STATUS_SPEED_HIGH => 64,
            _ => 512,
        };

        if speed == OP_PORT_STATUS_SPEED_LOW || speed == OP_PORT_STATUS_SPEED_FULL {
            port.is_low_or_full_speed = true;
        }

        assert_ne!(slot, 0, "invalid slot 0 received");
        self.setup_slot(&port, speed, max_packet_size, true);
        debug!("Slot Setup -> max_packet_size: {}", max_packet_size);
        let mut buf = [0u8; 8];
        self.fetch_descriptor(port.slot_id, DESCRIPTOR_TYPE_DEVICE,
                              0, 0, &mut buf)?;

        debug!("First descriptor: {:?}", &buf);

        let max_packet_size = match buf[7] {
            8 => 8,
            16 => 16,
            32 => 32,
            64 => 64,
            9 => 512,
            _ => panic!("Invalid maxPacketSize = {}", buf[7])
        };
        self.setup_slot(&port, speed, max_packet_size, false);
        let desc = self.fetch_device_descriptor(port.slot_id)?;
        debug!("Device Descriptor: {:#?}", desc);

        let configuration = self.fetch_configuration_descriptor(port.slot_id)?;
        debug!("configuration: {:#?}", configuration);

        assert_eq!(Into::<u8>::into(request_type!(HostToDevice, Standard, Device)), 0x00u8);
        self.send_control_command(port.slot_id, request_type!(HostToDevice, Standard, Device), REQUEST_SET_CONFIGURATION, configuration.config.config_val as u16, 0, 0, None, None)?;
        debug!("Applied Config {}", configuration.config.config_val);
        self.hal.sleep(Duration::from_millis(10));

        // Fetching language is removed due to various issues and most OS doesn't do it.
        // let mut buf = [0u8; 2];
        // self.fetch_descriptor(port.slot_id, DESCRIPTOR_TYPE_STRING,
        //                       0, 0, &mut buf)?;
        // assert_eq!(buf[1], DESCRIPTOR_TYPE_STRING, "Descriptor is not STRING");
        // assert!(buf[0] >= 4, "has language");
        // let mut buf2: Vec<u8> = Vec::new();
        // buf2.resize(buf[0] as usize, 0);
        // self.fetch_descriptor(port.slot_id, DESCRIPTOR_TYPE_STRING,
        //                       0, 0, &mut buf2)?;
        // let lang = buf2[2] as u16 | ((buf2[3] as u16) << 8);
        //
        // debug!("Language code: {:#x}", lang);

        // Display things
        let mfg = self.fetch_string_descriptor(port.slot_id, desc.manufacturer_index, 0x409).unwrap_or(String::from("(no manufacturer name)"));
        let prd = self.fetch_string_descriptor(port.slot_id, desc.product_index, 0x409).unwrap_or(String::from("(no product name)"));
        let serial = self.fetch_string_descriptor(port.slot_id, desc.serial_index, 0x409).unwrap_or(String::from("(no serial number)"));
        debug!("[XHCI] New device:\n  MFG: {}\n  Prd:{}\n  Serial:{}", mfg, prd, serial);

        for interface in &configuration.ifsets {
            if interface.interface.alt_set != 0 {
                debug!("Skipping non-default altSetting Interface");
                continue;
            }
            match interface.interface.class {
                CLASS_CODE_MASS => {
                    if interface.interface.sub_class != 0x6 {
                        debug!("Skipping MSD with sub-class other than 0x6 (Transparent SCSI)");
                        continue;
                    }

                    if interface.interface.protocol != 0x50 {
                        debug!("Skipping MSD with protocol other than bulk-only");
                        continue;
                    }

                    if interface.endpoints.len() < 2 {
                        warn!("MSD has not enough endpoints!");
                        continue;
                    }

                    let mut input_ring: Option<(u8, u8)> = None;
                    let mut output_ring: Option<(u8, u8)> = None;

                    self.with_input_context(port, |this, input_ctx| {
                        for endpoint in interface.endpoints.iter() {
                            if endpoint.attr != EP_ATTR_BULK {
                                continue;
                            }

                            let typ = if Self::is_ep_input(endpoint.address) { EP_TYPE_BULK_IN } else { EP_TYPE_BULK_OUT };

                            let ring = this.configure_endpoint(port.slot_id, input_ctx,
                                                               endpoint.address,
                                                               typ, endpoint.max_packet_size,
                                                               endpoint.interval, this.get_max_esti_payload(endpoint));

                            if Self::is_ep_input(endpoint.address) {
                                input_ring = Some(ring);
                            } else {
                                output_ring = Some(ring);
                            }
                        }

                        input_ctx.set_configure_ep_meta(configuration.config.config_val,
                                                        interface.interface.interface_number, interface.interface.alt_set);

                        Ok(())
                    })?;

                    let input_ring = input_ring.ok_or(Error::Str("MSD has no bulk input endpoint"))?;
                    let output_ring = output_ring.ok_or(Error::Str("MSD has no bulk output endpoint"))?;

                    debug!("MSD endpoints initialized with bulk_in:{:?} bulk_out:{:?}", input_ring, output_ring);

                    let mut buf: Vec<u8> = Vec::new();
                    buf.resize(31, 0);

                    let t = [0x55, 0x53, 0x42, 0x43, 0x13, 0x37, 0x04, 0x20, 0x24, 0x00, 0x00, 0x00, 0x80, 0x00, 6, 0x12, 0x00, 0x00, 0x00, 0x24, 0x00];
                    (&mut buf[..t.len()]).copy_from_slice(&t);

                    let r = self.transfer_bulk_ring(output_ring, TransferDirection::Write(buf.as_ref()));
                    info!("Transfer Result: {:?}", r);

                    let mut buf: Vec<u8> = Vec::new();
                    buf.resize(36, 0);

                    let r = self.transfer_bulk_ring(input_ring, TransferDirection::Read(buf.as_mut()));
                    info!("Transfer Result: {:?}", r);
                    info!("Got: {:x?}", buf.as_slice());

                    let mut buf: Vec<u8> = Vec::new();
                    buf.resize(13, 0);

                    let r = self.transfer_bulk_ring(input_ring, TransferDirection::Read(buf.as_mut()));
                    info!("Transfer Result: {:?}", r);
                    info!("Got: {:x?}", buf.as_slice());

                    self.hal.sleep(Duration::from_secs(5));
                }
                CLASS_CODE_HID => {
                    if interface.interface.sub_class != 1 {
                        debug!("Skipping non bios-mode HID device");
                        continue;
                    }

                    if interface.interface.protocol != 1 {
                        debug!("Skipping non keyboard");
                        continue;
                    }

                    if interface.endpoints.len() == 0 {
                        warn!("keyboard with no endpoints!");
                        continue;
                    }


                    // Enable keyboard

                    self.with_input_context(port, |_this, input_ctx| {

                        // self.configure_endpoint(port.slot_id, input_ctx.as_mut(),
                        //                         interface.endpoints[0].address,
                        //                         EP_TYPE_INTERRUPT_IN, interface.endpoints[0].max_packet_size,
                        //                         interface.endpoints[0].interval, self.get_max_esti_payload(&interface.endpoints[0]));


                        input_ctx.set_configure_ep_meta(configuration.config.config_val,
                                                        interface.interface.interface_number, interface.interface.alt_set);

                        Ok(())
                    })?;

                    debug!("done keyboard configure endpoint");

                    self.hal.sleep(Duration::from_millis(100));

                    debug!("set hid idle");
                    self.set_hid_idle(port.slot_id)?;

                    // 0 is Boot Protocol
                    debug!("set hid protocol");
                    self.set_hid_protocol(port.slot_id, 0)?;

                    // self.send_control_command(port.slot_id, 0x0, REQUEST_SET_CONFIGURATION, 1, 0, 0, None, None)?;
                    // debug!("Applied Config {}", configuration.config.config_val);


                    let index = self.get_epctx_index(interface.endpoints[0].address);

                    // for i in 0..10 {
                    //     // debug!("set hid report");
                    //     // match self.set_hid_report(port.slot_id, 1, 0, interface.interface.interface_number as u16, if i % 2 == 0 { 0 } else { 0xFF }) {
                    //     //     Ok(_) => {
                    //     //         info!("set_hid success");
                    //     //     }
                    //     //     Err(e) => {
                    //     //         warn!("set_hid: failed to read: {:?}", e);
                    //     //     }
                    //     // }
                    //
                    //     debug!("fetch hid report");
                    //     let mut buf = Box::new([0u8; 128]);
                    //     match self.fetch_hid_report(port.slot_id, 1, 0, interface.interface.interface_number as u16, &mut buf[0..8]) {
                    //         Ok(_) => {
                    //             info!("got response: {:?}", &buf[0..8]);
                    //         }
                    //         Err(e) => {
                    //             warn!("failed to read: {:?}", e);
                    //         }
                    //     }
                    //
                    //     self.hal.sleep(Duration::from_millis(500));
                    // }
                }
                CLASS_CODE_HUB => {
                    if interface.endpoints.len() == 0 {
                        warn!("Hub with no endpoints!");
                        continue;
                    }
                    if interface.endpoints.len() > 1 {
                        warn!("Hub with more than 1 endpoint!");
                        continue;
                    }

                    let hub_descriptor = self.fetch_hub_descriptor(port.slot_id, &desc)?;
                    info!("Hub Descriptor: {:?}", hub_descriptor);

                    // // Get Status
                    // let mut buf = [0u8; 2];
                    // self.send_control_command(port.slot_id,
                    //                           0x80,
                    //                           0x0, // Get Status
                    //                           0x0, 0x0,
                    //                           2, None, Some(&mut buf),
                    // )?;
                    // debug!("Status Read back: {:?}", buf);

                    // Setup EPs
                    debug!("Found {} eps on this interface", interface.endpoints.len());


                    // Reconfigure to hub
                    {
                        self.with_input_context(port, |_this, input_ctx| {
                            let mut slot_ctx = input_ctx.get_slot_mut();
                            slot_ctx.dword1.set_hub(true);
                            slot_ctx.numbr_ports = hub_descriptor.num_ports;
                            slot_ctx.slot_state = 0;

                            slot_ctx.interrupter_ttt = 0;

                            Ok(())
                        })?;

                        self.with_input_context(port, |this, input_ctx| {
                            this.configure_endpoint(port.slot_id, input_ctx,
                                                    interface.endpoints[0].address,
                                                    EP_TYPE_INTERRUPT_IN, interface.endpoints[0].max_packet_size,
                                                    interface.endpoints[0].interval, this.get_max_esti_payload(&interface.endpoints[0]));

                            input_ctx.set_configure_ep_meta(configuration.config.config_val,
                                                            interface.interface.interface_number, interface.interface.alt_set);

                            Ok(())
                        })?;
                    }

                    debug!("slot state = {}", self.device_contexts[port.slot_id as usize - 1].as_ref().unwrap().get_slot().slot_state);

                    let hub_descriptor = self.fetch_hub_descriptor(port.slot_id, &desc)?;
                    info!("Hub Descriptor Pt2: {:?}", hub_descriptor);


                    for num in 1..=hub_descriptor.num_ports {
                        self.set_feature(port.slot_id, num, FEATURE_PORT_POWER)?;

                        self.hal.sleep(Duration::from_millis(hub_descriptor.potpgt as u64 * 2));

                        self.clear_feature(port.slot_id, num, FEATURE_C_PORT_CONNECTION)?;

                        let status = self.fetch_port_status(slot as u8, num)?;

                        debug!("Port {}: status={:?}", num, status);

                        if !status.get_device_connected() {
                            continue;
                        }

                        let mut child_port = port.child_port(num);
                        match self.setup_new_device(&mut child_port) {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Failed to configure child of port {}: {:?}", port.port_id, e);
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        // let usb_dev = Arc::new(USBXHCIDevice {
        //     // Device
        //     dev_descriptor: desc,
        //     manufacture: mfg,
        //     product: prd,
        //     serial,
        //     // XHCI
        //     control_slot: slot as u8,
        //     xhci_port: port_id,
        //     controller_id: self.id,
        //     // USB System
        //     system_id: G_USB.issue_device_id(),
        // });
        //
        // self.devices.write().push(usb_dev.clone());
        // G_USB.register_device(usb_dev);
        //
        // // let configs = self.fetch_configuration_descriptor(slot as u8)?;
        // // let config_val = configs.config.config_val;
        // // debug!("[USB] Applying Config {}", config_val);
        // // self.send_control_command(slot as u8, 0x0, REQUEST_SET_CONFIGURATION,
        // //                           config_val as u16, 0, 0, None, None)?;
        // // // SETUP EPs BEGIN
        // // for interf in configs.ifsets.iter() {
        // //     for ep in interf.endpoints.iter() {
        // //         let epnum = ep.address & 0b1111;
        // //         let is_in = (ep.address & 0x80) >> 7;
        // //         let xhci_ep_number = epnum << 1 | is_in;
        // //         let ring = XHCIRing::new_with_capacity(1, true);
        // //
        // //         debug!("[USB] Endpoint #{}, is_out: {}: => ({:#x})", epnum, is_in, xhci_ep_number);
        // //     }
        // // }
        // // // SETUP EPs END
        // // // GET MAX LUN BEGIN
        // // use crate::device::usb::consts::*;
        // // let request_type = USB_REQUEST_TYPE_DIR_DEVICE_TO_HOST | USB_REQUEST_TYPE_TYPE_CLASS | USB_REQUEST_TYPE_RECP_INTERFACE;
        // // let mut lun = [0u8;1];
        // // let size = self.send_control_command(slot as u8, request_type, 0xFE, // GET MAX LUN
        // // 0,0,1, None, Some(&mut lun))?;
        // // assert_eq!(size, 1, "lun size wrong");
        // // let lun = if lun[0] == 0xFF { 0x0 } else {lun[0]};
        // // debug!("MAX LUN: {}", lun);
        // // GET MAX LUN END
        // debug!("[XHCI] Completed setup port {} on slot {}", port_id, slot);
        return Ok(slot as u8);
    }

    pub fn do_stuff(&mut self) -> Result<(), Error> {
        self.transfer_ownership()?;

        self.reset()?;
        debug!("did reset");

        {
            let hcsparams1 = self.cap.hcs_params[0].read();
            debug!("[XHCI] HCS1: {:#x}", hcsparams1);
            self.info.max_port = ((hcsparams1 & CAP_HCSPARAMS1_MAX_PORT_MASK) >> CAP_HCSPARAMS1_MAX_PORT_SHIFT) as u8;
            self.info.max_slot = (hcsparams1 & CAP_HCSPARAMS1_SLOTS_MASK) as u8;
            self.info.page_size = self.op.page_size.read() << 12;
            debug!("[XHCI] PageSize = {}", self.info.page_size);
            if self.info.page_size > 4096 {
                error!("[XHCI] PageSize > 4096 not supported");
                return Err(Error::Str("PageSize > 4096 not supported"));
            }
            self.info.big_context = self.cap.hcc_param1.read() & CAP_HCCPARAMS1_CSZ_MASK != 0;
            debug!("[XHCI] controller use {} bytes context", if self.info.big_context { 64 } else { 32 });
            if self.info.big_context {
                error!("[XHCI] 64 bytes context not supported yet");
            }

            let hccparams1 = self.cap.hcc_param1.read();
            if hccparams1 & CAP_HCCPARAMS1_PORT_POWER_CTRL_MASK != 0 {
                info!("[XHCI] Controller Support Power Power");
            }
        }

        // Step 3: Setup opRegs->config
        {
            let mut tmp = self.op.config.read();
            tmp = (tmp & (!CAP_HCSPARAMS1_SLOTS_MASK)) | (self.info.max_slot as u32);
            self.op.config.write(tmp);
            debug!("[XHCI] nmbrSlot= {}, nmbrPorts = {}", self.info.max_slot, self.info.max_port);
        }

        // Step 4: Initialize Memory Structures
        self.initialize_memory_structures()?;
        debug!("[XHCI] Done memory initialization");

        // Step 5: Start the Controller !
        self.start()?;

        // Clear Interrupt Ctrl / Pending
        self.get_runtime_interrupt_register(0).iman.write(INT_IRQ_FLAG_INT_PENDING_MASK);
        self.get_runtime_interrupt_register(0).imod.write(0);

        let ver = (self.cap.length_and_ver.read() & CAP_HC_VERSION_MASK) >> CAP_HC_VERSION_SHIFT;
        debug!("[XHCI] Controller with version {:04x}", ver);

        self.send_nop();
        self.poll_ports();
        let crcr = self.op.command_ring_control.read();
        info!("current crcr: {:#x}", crcr);

        Ok(())
    }

    fn initialize_memory_structures(&mut self) -> Result<(), Error> {
        // Step 1: Setup Device Context Base Address Array
        if self.device_context_baa.is_none() {
            self.device_context_baa = Some(Box::new(DeviceContextBaseAddressArray::default()));
        }

        let dcbaa_pa = self.get_ptr::<DeviceContextBaseAddressArray>(self.device_context_baa.as_ref().unwrap());
        self.op.device_context_base_addr_array_ptr.write(dcbaa_pa);

        debug!("[XHCI] DCBAA Setup complete");

        // Step 2: Setup Command Ring (CRCR)
        if self.command_ring.is_none() {
            self.command_ring = Some(XHCIRing::new_with_capacity(self.hal, 1, true));
        }

        for i in 0..EVENT_RING_NUM_SEGMENTS {
            self.hal.flush_cache(self.command_ring.as_ref().unwrap().segments[i].as_ref() as *const XHCIRingSegment as u64, core::mem::size_of::<XHCIRingSegment>() as u64, FlushType::Clean);
        }

        let crcr_pa = self.get_ptr::<XHCIRingSegment>(self.command_ring.as_ref().unwrap().segments[0].as_ref());

        let initial_crcr = self.op.command_ring_control.read();
        debug!("[XHCI] CRCR initial {:x}", initial_crcr);
        {
            if initial_crcr & OP_CRCR_CRR_MASK != 0 {
                return Err(Error::Str("CrCr is Running"));
            }

            let cyc_state = self.command_ring.as_ref().unwrap().cycle_state as u64;
            assert_eq!(crcr_pa & 0b111111, 0, "alignment");
            let val64 = (initial_crcr & OP_CRCR_RES_MASK) |
                (crcr_pa & OP_CRCR_CRPTR_MASK) |
                (cyc_state & OP_CRCR_CS_MASK);

            self.hal.memory_barrier();
            self.op.command_ring_control.write(val64);
            self.hal.memory_barrier();
            self.get_doorbell_register(0).reg.write(0);
            self.hal.memory_barrier();
            for i in 0..EVENT_RING_NUM_SEGMENTS {
                self.hal.flush_cache(self.command_ring.as_ref().unwrap().segments[i].as_ref() as *const XHCIRingSegment as u64, core::mem::size_of::<XHCIRingSegment>() as u64, FlushType::Clean);
            }

            let crcr = self.op.command_ring_control.read();
            info!("current crcr: {:#x}", crcr);
        }

        debug!("[XHCI] CRCR Setup complete");

        // Setup Event Ring
        self.event_ring = Some(XHCIRing::new_with_capacity(self.hal, EVENT_RING_NUM_SEGMENTS, false));
        self.event_ring_table = Some(Box::new(EventRingSegmentTable::default()));
        self.event_ring_table.as_mut().unwrap().segment_count = EVENT_RING_NUM_SEGMENTS;

        for idx in 0..EVENT_RING_NUM_SEGMENTS {
            let pa = self.get_ptr::<XHCIRingSegment>(self.event_ring.as_ref().unwrap().segments[idx].as_ref());

            let ent = self.event_ring_table.as_mut().unwrap();
            ent.segments[idx].segment_size = TRBS_PER_SEGMENT as u32;
            assert_eq!(pa & 0b11_1111, 0, "alignment");
            ent.segments[idx].addr = pa;
        }

        self.hal.flush_cache(self.event_ring_table.as_deref().unwrap() as *const EventRingSegmentTable as u64, core::mem::size_of::<EventRingSegmentTable>() as u64, FlushType::Clean);

        // Update Interrupter 0 Dequeu Pointer
        let dequeue_ptr_pa = self.get_ptr::<TRB>(&self.event_ring.as_ref().unwrap().segments[0].trbs[0]);
        self.get_runtime_interrupt_register(0).event_ring_deque_ptr.write(dequeue_ptr_pa & INT_ERDP_DEQUEUE_PTR_MASK);

        // set ERST table register with correct count
        let mut tmp = self.get_runtime_interrupt_register(0).event_ring_table_size.read();
        tmp &= !INT_ERSTSZ_TABLE_SIZE_MASK;
        tmp |= (EVENT_RING_NUM_SEGMENTS as u32) & INT_ERSTSZ_TABLE_SIZE_MASK;
        self.get_runtime_interrupt_register(0).event_ring_table_size.write(tmp);

        // Setup Event Ring Segment Table Pointer
        let erst_pa = self.get_ptr::<EventRingSegmentTable>(self.event_ring_table.as_ref().unwrap());
        self.get_runtime_interrupt_register(0).event_ring_seg_table_ptr.write(erst_pa);

        // Setup Scratchpad Registers
        let tmp = self.cap.hcs_params[2].read();
        let mut num_sp = (tmp & CAP_HCSPARAMS2_MAX_SCRATCH_H_MSAK)
            >> CAP_HCSPARAMS2_MAX_SCRATCH_H_SHIFT;
        num_sp <<= 5;
        num_sp |= (tmp & CAP_HCSPARAMS2_MAX_SCRATCH_L_MSAK) >> CAP_HCSPARAMS2_MAX_SCRATCH_L_SHIFT;
        if num_sp > 0 {
            self.scratchpads = Some(Box::new(
                ScratchPadBufferArray::new_with_capacity(self.hal, num_sp as usize, self.info.page_size as usize)
            ));
            let scratch_pa = self.get_ptr::<ScratchPadBufferArray>(self.scratchpads.as_ref().unwrap().as_ref());
            self.device_context_baa.as_mut().unwrap().entries[0] = scratch_pa;
        }

        // Zero device notification
        self.op.dnctlr.write(0x0);

        Ok(())
    }

    fn start(&mut self) -> Result<(), Error> {
        debug!("[XHCI] Starting the controller");

        self.op.command.update(|reg| {
            *reg |= OP_CMD_RUN_STOP_MASK | OP_CMD_HSERR_EN_MASK;
            // TODO maybe don't disable
            *reg &= !OP_CMD_INT_EN_MASK; // DISABLE INTERRUPT
        });

        self.wait_until(Error::Str("did not start!"), HALT_TIMEOUT, |this| {
            this.op.status.read() & OP_STS_HLT_MASK == 0
        })?;

        debug!("[XHCI] Started.");

        Ok(())
    }

    fn poll_event_ring_trb(&mut self) -> Option<TRBType> {
        let timeout = Duration::from_millis(1000) + self.hal.current_time();
        loop {
            let pop = self.event_ring.as_mut().expect("").pop(false);
            match pop {
                Some(trb) => {
                    let tmp = self.event_ring.as_ref().expect("").dequeue_pointer() |
                        INT_ERDP_BUSY_MASK | (INT_ERDP_DESI_MASK & self.event_ring.as_ref().expect("").dequeue.0 as u64);
                    self.get_runtime_interrupt_register(0).event_ring_deque_ptr.write(tmp);
                    let lol = TRBType::from(trb);
                    return Some(lol);
                }
                None => {}
            }
            if self.hal.current_time() > timeout {
                return None;
            }
            self.hal.sleep(Duration::from_millis(1));
        }
    }

    fn wait_command_complete(&mut self, ptr: u64) -> Option<CommandCompletionTRB> {
        self.hal.memory_barrier();
        // TODO update this code to use interrupt notification system
        self.get_doorbell_register(0).reg.write(0);
        loop {
            let trb = self.poll_event_ring_trb()?;
            match trb {
                TRBType::CommandCompletion(c) => {
                    if c.trb_pointer == ptr {
                        debug!("Got command completion: {:?}", c);
                        if c.code != 1 {
                            panic!("EHHHH WTF: {:?}", TRBCompletionCode::from(c.code));
                        }
                        return Some(c);
                    } else {
                        debug!("trb_pointer badddddd: {:?}", c);
                    }
                }
                e => {
                    debug!("Lol: {:?}", e);
                }
            }
        }
    }

    pub fn send_nop(&mut self) {
        let ptr = {
            let cmd_ring = self.command_ring.as_mut().expect("uninitialized");
            debug!("[XHCI] Sending NOOP on index {:?}", cmd_ring.enqueue);
            cmd_ring.push(CommandTRB::noop().into())
        };

        match self.wait_command_complete(ptr) {
            Some(_) => debug!("NoOP Complete at {:#x}", ptr),
            None => warn!("NoOP didnt return at {:#x}", ptr),
        }
    }

    fn is_port_connected(&self, port_id: u8) -> bool {
        let port_op = self.op.get_port_operational_register(port_id);
        let mut port_status = port_op.portsc.read();
        if port_status & OP_PORT_STATUS_POWER_MASK == 0 {
            debug!("[XHCI] Port {} not powered. Powering On", port_id);
            let tmp = (port_status & !OP_PORT_STATUS_PED_MASK) | OP_PORT_STATUS_POWER_MASK;
            port_op.portsc.write(tmp);
            while port_op.portsc.read() & OP_PORT_STATUS_POWER_MASK == 0 {}
            self.hal.sleep(Duration::from_millis(20));
            port_status = port_op.portsc.read();
            debug!("[XHCI] port {} powerup complete: status={:#x}", port_id, port_status);
        }
        let mut tmp = port_status & !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
        tmp |= OP_PORT_STATUS_OCC_MASK | OP_PORT_STATUS_CSC_MASK;
        port_op.portsc.write(tmp);

        // Check if a port is in it's ready state
        let ready_mask = OP_PORT_STATUS_CCS_MASK;
        port_op.portsc.read() & ready_mask == ready_mask
    }

    fn poll_port_status(&mut self, port_id: u8) {
        let ready = self.is_port_connected(port_id);

        info!("Port {}: connected={}", port_id, ready);
        if ready {
            let mut port = Port::new_from_root(port_id);
            if let Err(e) = self.setup_new_device(&mut port) {
                error!("setup_new_device() err: {:?}", e);
            }
        }
    }

    pub fn poll_ports(&mut self) {
        let max_port = self.info.max_port;
        for port in 1..=max_port {
            self.poll_port_status(port);
        }
    }

    pub fn extended_capability(&self) -> ExtendedCapabilityTags {
        // The higher 16bits specify the offset from base,
        // unit is **DWORD**
        let hcc1val = self.cap.hcc_param1.read();
        let cap_list_offset = (hcc1val >> 14) & !0b11u32; // LSH 2 so it's in bytes
        let cap_list_base = self.mmio_virt_base + cap_list_offset as u64;
        ExtendedCapabilityTags::get(cap_list_base as usize)
    }

    /// This function transfer the ownership of the controller from BIOS
    pub fn transfer_ownership(&mut self) -> Result<(), Error> {
        let tags = self.extended_capability().find(|t| {
            match t {
                ExtendedCapabilityTag::USBLegacySupport { head: _, bios_own: _, os_own: _ } => true,
                _ => false
            }
        });
        match tags {
            Some(t) => {
                if let ExtendedCapabilityTag::USBLegacySupport { head, bios_own, os_own } = t {
                    if os_own && bios_own {
                        return Err(Error::Str("XHCIError::UnexpectedOwnership"));
                    }
                    if os_own {
                        return Ok(());
                    }
                    let write_ptr = (head as *mut u8).wrapping_offset(3);
                    let read_ptr = (head as *const u8).wrapping_offset(2);
                    unsafe { write_ptr.write_volatile(0x1) }; // Claiming Ownership
                    // Now wait
                    // TODO implement timeout
                    while unsafe { read_ptr.read_volatile() } & 0x1 > 1 {}
                    return Ok(());
                }
                panic!("wrong tag type found");
            }
            _ => Ok(())
        }
    }

    fn into_type<T: Sized>(buf: &[u8]) -> T {
        let mut thing: T = unsafe { core::mem::zeroed() };
        {
            let tmp_slice = unsafe {
                core::slice::from_raw_parts_mut(
                    &mut thing as *mut T as *mut u8,
                    core::mem::size_of::<T>(),
                )
            };
            assert_eq!(tmp_slice.len(), buf.len(), "Unexpected size");
            tmp_slice.copy_from_slice(&buf);
        }
        thing
    }
}

#[derive(Clone)]
pub struct XhciWrapper(Arc<Mutex<Xhci<'static>>>);

struct USBDeviceMeta {
    pub is_root_hub: bool,
    pub slot: u8,
    pub port: Port,
}

impl USBMeta for USBDeviceMeta {}

impl USBHostController for XhciWrapper {
    fn register_root_hub(&self, device: &Arc<RwLock<USBDevice>>) {
        let device = device.write();
        device.prv = Some(Box::new(USBDeviceMeta {
            is_root_hub: true,
            slot: 0,
            port: Port::new_from_root(0), // fake port
        }));
    }

    fn pipe_open(&self, device: &Arc<RwLock<USBDevice>>, endpoint: Option<&USBEndpointDescriptor>) -> Result<Arc<RwLock<USBPipe>>, usb_host::items::Error> {
        let device = device.write();

        let endpoint_type = match endpoint {
            None => EndpointType::Control,
            Some(d) => match d.descriptor_type {
                0 => EndpointType::Control,
                1 => EndpointType::Isochronous,
                2 => EndpointType::Bulk,
                3 => EndpointType::Interrupt,
                d => panic!("bad descriptor_type: {}", d),
            }
        };

        let f = device.prv.as_ref().unwrap().downcast_ref::<USBDeviceMeta>().unwrap();
        if f.is_root_hub {
            assert!(matches!(endpoint, None));
            return Ok(Arc::new(RwLock::new(USBPipe { endpoint_type: EndpointType::Control })));
        }

        match endpoint_type {
            EndpointType::Control => {

                let mut x = self.0.lock();

                let slot = x.send_slot_enable()? as usize;

                let mut f = device.prv.as_mut().unwrap().downcast_mut::<USBDeviceMeta>().unwrap();
                f.port.slot_id = slot as u8;

                x.setup_slot(&f.port, device.speed, device.max_packet_size, true);


            },
            EndpointType::Isochronous => {},
            EndpointType::Bulk => {},
            EndpointType::Interrupt => {},
        }

        Err(usb_host::items::Error::Str("bad"))
    }

    fn set_address(&self, device: &Arc<RwLock<USBDevice>>) {

        let device = device.write();
        let f = device.prv.as_ref().unwrap().downcast_ref::<USBDeviceMeta>().unwrap();
        if f.is_root_hub {
            return;
        }

        let mut x = self.0.lock();
        x.setup_slot(&f.port, device.speed, device.max_packet_size, false);

    }

    fn reset_port(&self, device: &Arc<RwLock<USBDevice>>, port: u8) {
        unimplemented!()
    }

    fn control_transfer(&self, endpoint: Arc<RwLock<USBPipe>>) {
        unimplemented!()
    }
}

#[repr(C)]
struct Dwc3 {
    g_sbuscfg0: Volatile<u32>,
    g_sbuscfg1: Volatile<u32>,
    g_txthrcfg: Volatile<u32>,
    g_rxthrcfg: Volatile<u32>,
    g_ctl: Volatile<u32>,

    reserved1: Volatile<u32>,

    g_sts: Volatile<u32>,

    reserved2: Volatile<u32>,

    g_snpsid: Volatile<u32>,
    g_gpio: Volatile<u32>,
    g_uid: Volatile<u32>,
    g_uctl: Volatile<u32>,
    g_buserraddr: Volatile<u64>,
    g_prtbimap: Volatile<u64>,

    g_hwparams0: Volatile<u32>,
    g_hwparams1: Volatile<u32>,
    g_hwparams2: Volatile<u32>,
    g_hwparams3: Volatile<u32>,
    g_hwparams4: Volatile<u32>,
    g_hwparams5: Volatile<u32>,
    g_hwparams6: Volatile<u32>,
    g_hwparams7: Volatile<u32>,

    g_dbgfifospace: Volatile<u32>,
    g_dbgltssm: Volatile<u32>,
    g_dbglnmcc: Volatile<u32>,
    g_dbgbmu: Volatile<u32>,
    g_dbglspmux: Volatile<u32>,
    g_dbglsp: Volatile<u32>,
    g_dbgepinfo0: Volatile<u32>,
    g_dbgepinfo1: Volatile<u32>,
}

const DWC3_GSNPSID_MASK: u32 = 0xffff0000;
const DWC3_REVISION_MASK: u32 = 0xffff;
const DWC3_GCTL_SCALEDOWN_MASK: u32 = 3 << 4;
const DWC3_GCTL_DISSCRAMBLE: u32 = 1 << 3;
const DWC3_GCTL_U2RSTECN: u32 = 1 << 16;

const DWC3_GCTL_PRTCAP_HOST: u32 = 1;
const DWC3_GCTL_PRTCAP_DEVICE: u32 = 2;
const DWC3_GCTL_PRTCAP_OTG: u32 = 3;

pub fn init_dwc3(base_address: u64) {
    let dwc3 = unsafe { &mut *((base_address + 0xC100) as *mut Dwc3) };

    // ====== Core Init =====

    let revision = dwc3.g_snpsid.read();
    /* This should read as U3 followed by revision number */
    if (revision & DWC3_GSNPSID_MASK) != 0x55330000 {
        warn!("This is not a DesignWare USB3 DRD Core, id mask: {:#x}", revision);
    } else {
        debug!("init DesignWare USB3 DRD Core, id mask: {:#x}", revision);
    }

    // TODO issue a soft reset

    let mut reg = dwc3.g_ctl.read();
    reg &= !DWC3_GCTL_SCALEDOWN_MASK;
    reg |= DWC3_GCTL_DISSCRAMBLE;
    // TODO enable any power opt

    /*
   * WORKAROUND: DWC3 revisions <1.90a have a bug
   * where the device can fail to connect at SuperSpeed
   * and falls back to high-speed mode which causes
   * the device to enter a Connect/Disconnect loop
   */
    if (revision & DWC3_REVISION_MASK) < 0x190a {
        reg |= DWC3_GCTL_U2RSTECN;
    }

    dwc3.g_ctl.write(reg);

    // ===== Set Mode =====

    /* We are hard-coding DWC3 core to Host Mode */
    let mut reg = dwc3.g_ctl.read();
    // clear mask
    reg &= !((DWC3_GCTL_PRTCAP_OTG) << 12);
    reg |= ((DWC3_GCTL_PRTCAP_HOST) << 12);
    dwc3.g_ctl.write(reg);

    debug!("did dwc3_init()");
}


pub fn do_stuff(base_address: u64, hal: &dyn HAL) {
    match Xhci::new(base_address, hal).do_stuff() {
        Ok(()) => info!("did stuff successfully"),
        Err(e) => error!("Error failed to do stuff: {:?}", e),
    }
}


