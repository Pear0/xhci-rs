#![feature(allocator_api)]
#![feature(const_in_array_repeat_expressions)]
#![feature(global_asm)]
#![feature(llvm_asm)]
#![feature(track_caller)]

#![allow(dead_code, unused_imports, unused_parens, unused_variables)]

#![cfg_attr(not(test), no_std)]

// TODO: https://docs.rs/downcast-rs/1.2.0/downcast_rs/index.html for USB implementation specifics

extern crate alloc;
#[macro_use]
extern crate log;
#[macro_use]
extern crate usb_host;

use alloc::alloc::{AllocInit, AllocRef, Global, Layout};
use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::ops::Deref;
use core::ptr::NonNull;
use core::time::Duration;

use downcast_rs::Downcast;
use hashbrown::HashMap;
use spin::{Mutex, RwLock};
use volatile::*;

use usb_host::{USBErrorKind, USBResult};
use usb_host::consts::*;
use usb_host::descriptor::*;
use usb_host::items::{ControlCommand, EndpointType, PortStatus, TransferBuffer, TypeTriple};
use usb_host::items::TransferDirection::HostToDevice;
use usb_host::structs::{USBDevice, USBPipe};
use usb_host::traits::{USBHostController, USBMeta, USBAsyncReadFn};

use crate::consts::*;
use crate::extended_capability::{ExtendedCapabilityTag, ExtendedCapabilityTags};
use crate::quirks::XHCIQuirks;
use crate::registers::{DoorBellRegister, InterrupterRegisters};
use crate::structs::{DeviceContextArray, DeviceContextBaseAddressArray, EventRingSegmentTable, InputContext, ScratchPadBufferArray, SlotContext, XHCIRing, XHCIRingSegment};
use crate::trb::{CommandCompletionTRB, CommandTRB, DataStageTRB, EventDataTRB, NormalTRB, SetupStageTRB, StatusStageTRB, TransferEventTRB, TRB, TRBType, TransferEventTRBStatusWord};
use crate::trb::TRBType::TransferEvent;

pub mod quirks;
#[macro_use]
pub(crate) mod consts;
pub(crate) mod extended_capability;
pub(crate) mod registers;
pub(crate) mod structs;
pub(crate) mod trb;

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

fn as_mut_slice<T>(t: &mut T) -> &mut [u8] {
    unsafe { core::slice::from_raw_parts_mut(t as *mut T as *mut u8, core::mem::size_of::<T>()) }
}

fn as_slice<T>(t: &T) -> &[u8] {
    unsafe { core::slice::from_raw_parts(t as *const T as *const u8, core::mem::size_of::<T>()) }
}

fn copy_from_slice(dest: &mut [u8], src: &[u8]) {
    let amount = core::cmp::min(dest.len(), src.len());
    dest[..amount].copy_from_slice(&src[..amount]);
}

#[track_caller]
fn respond_slice(command: ControlCommand, src: &[u8]) {
    if let TransferBuffer::Read(slice) = command.buffer {
        copy_from_slice(slice, src);
    } else {
        panic!("expected a Read on command: {:?}", command);
    }
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

pub trait XhciHAL: usb_host::UsbHAL + Send + Sync {
    fn memory_barrier();
    fn translate_addr(addr: u64) -> u64;
    fn flush_cache(addr: u64, len: u64, flush: FlushType);

    fn alloc_noncached(layout: Layout) -> Option<u64> {
        Global.alloc(layout, AllocInit::Zeroed).ok().map(|x| x.ptr.as_ptr() as u64)
    }

    fn free_noncached(ptr: u64, layout: Layout) {
        unsafe {
            Global.dealloc(NonNull::new_unchecked(ptr as *mut u8), layout);
        }
    }
}

#[derive(Debug)]
struct PortContext {
    port: u8,
    root_port: u8,
    parent_slot: u8,
    parent_is_low_or_full_speed: bool,
    route_string: u32,
    speed: USBSpeed,
    max_packet_size: u16,
}

struct IntTransferContext {
    slot: u8,
    buf: Vec<u8>,
    callback: USBAsyncReadFn,
    packet_size: usize,
    response: Option<TransferEventTRBStatusWord>,
}

pub struct Xhci<H: XhciHAL> {
    mmio_virt_base: u64,
    cap: &'static mut XHCICapabilityRegisters,
    op: &'static mut XHCIOperationalRegisters,
    info: XHCIInfo,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    device_contexts: [Option<Box<DeviceContextArray>>; 255],

    // (slot, endpoint)
    transfer_rings: HashMap<(u8, u8), Arc<Mutex<XHCIRing<H>>>>,
    pending_interrupt_transfers: HashMap<u64, IntTransferContext>,

    command_ring: Option<XHCIRing<H>>,
    event_ring: Option<XHCIRing<H>>,
    event_ring_table: Option<Box<EventRingSegmentTable>>,
    scratchpads: Option<Box<ScratchPadBufferArray>>,
    pub quirks: XHCIQuirks,
    __phantom: PhantomData<H>,
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

impl<H: XhciHAL> Xhci<H> {
    pub fn new(base_address: u64) -> Self {
        let cap = get_registers::<XHCICapabilityRegisters>(base_address);
        let cap_size = (cap.length_and_ver.read() & 0xFF) as u64;

        let op_regs = get_registers::<XHCIOperationalRegisters>(base_address + cap_size);

        info!("Page Size: {}", op_regs.page_size.read());
        info!("Status: {:#x}", op_regs.status.read());

        let mut info = XHCIInfo::default();
        info.doorbell_offset = cap.doorbell_offset.read() & CAP_DBOFFSET_MASK;
        info.runtime_offset = cap.rts_offset.read() & CAP_RTSOFFSET_MASK;

        let mut this = Self {
            mmio_virt_base: base_address,
            cap,
            op: op_regs,
            info,
            device_context_baa: None,
            device_contexts: [None; 255],
            transfer_rings: HashMap::new(),
            pending_interrupt_transfers: HashMap::new(),
            command_ring: None,
            event_ring: None,
            event_ring_table: None,
            scratchpads: None,
            quirks: Default::default(),
            __phantom: PhantomData::default(),
        };

        this.do_stuff();
        this
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
        let wait_limit = H::current_time() + timeout;
        loop {
            if func(self) {
                return Ok(());
            }
            if H::current_time() > wait_limit {
                return Err(e);
            }
            H::sleep(Duration::from_millis(1));
        }
    }

    fn flush_trb(&self, ptr: u64, flush: FlushType) {
        H::flush_cache(ptr, 16, flush);
    }

    fn flush_struct<T>(&self, val: &T, flush: FlushType) {
        H::flush_cache(val as *const T as u64, core::mem::size_of_val(val) as u64, flush);
    }

    fn flush_slice<T>(&self, val: &[T], flush: FlushType) {
        H::flush_cache(val.as_ptr() as u64, core::mem::size_of_val(val) as u64, flush);
    }

    pub fn reset(&mut self) -> USBResult<()> {
        self.op.command.update(|x| *x |= OP_CMD_RESET_MASK);
        H::memory_barrier();

        H::wait_until(USBErrorKind::Timeout.msg("did not reset"), RESET_TIMEOUT, || {
            let cmd = self.op.command.read();
            let sts = self.op.status.read();
            (cmd & OP_CMD_RESET_MASK == 0) && (sts & OP_STS_CNR_MASK == 0)
        })?;

        Ok(())
    }

    fn get_ptr<T>(&self, t: &T) -> u64 {
        H::translate_addr(t as *const T as u64)
    }

    fn send_slot_enable(&mut self) -> USBResult<u8> {
        let cmd = CommandTRB::enable_slot();
        let ptr = self.command_ring.as_mut()
            .expect("no cmd ring found").push(cmd.into());
        trace!("[XHCI] Sending Slot EN");

        match self.wait_command_complete(ptr) {
            Some(trb) => Ok(trb.slot),
            _ => USBErrorKind::Timeout.err("timed out waiting for new slot")
        }
    }

    fn get_epctx_index(endpoint_address: u8) -> u8 {
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
        let transfer_ring = XHCIRing::<H>::new_with_capacity(1, true);
        let transfer_ring_ptr = self.get_ptr::<XHCIRingSegment>(transfer_ring.segments[0].as_ref());
        assert_eq!(transfer_ring_ptr & 0b1111, 0, "alignment");
        trace!("[XHCI] Setting Transfer Ring Pointer to {:#x}", transfer_ring_ptr);

        let index = Self::get_epctx_index(endpoint_address);

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

    fn create_slot_context(&mut self, input_ctx: &mut InputContext, port_ctx: &PortContext) {
        input_ctx.get_slot_mut().dword1.set_speed(speed_to_xhci(port_ctx.speed));
        input_ctx.get_slot_mut().dword1.set_context_entries(1); // TODO Maybe not hardcode 1?
        input_ctx.get_slot_mut().root_hub_port_number = port_ctx.root_port;
        input_ctx.get_slot_mut().dword1.set_route_string(port_ctx.route_string);
        input_ctx.get_slot_mut().slot_state = 0;

        if port_ctx.speed.is_low_or_full_speed() {
            // root hub slot id == 0
            if !port_ctx.parent_is_low_or_full_speed && port_ctx.parent_slot != 0 {
                input_ctx.get_slot_mut().parent_hub_slot_id = port_ctx.parent_slot;
                input_ctx.get_slot_mut().parent_port_number = port_ctx.port;
            }
        }
    }

    fn setup_slot(&mut self, slot: u8, port_ctx: &PortContext, block_cmd: bool) -> Box<InputContext> {
        debug!("setup_slot(slot: {}, port_ctx: {:?}, block_cmd: {})", slot, port_ctx, block_cmd);
        let slot = slot as usize;

        let mut input_ctx = Box::new(if self.info.big_context { InputContext::new_big() } else { InputContext::new_normal() });

        if let Some(output_ctx) = self.device_contexts[slot].as_ref() {
            self.flush_struct::<DeviceContextArray>(output_ctx.as_ref(), FlushType::Invalidate);
            // Clone slot context from Output -> Input
            *input_ctx.get_slot_mut() = output_ctx.get_slot().clone();

            // Clone CtrlEP0
            *input_ctx.get_endpoint_mut(0) = output_ctx.get_endpoint(0).clone();
        } else {
            let dev_ctx = Box::new(if self.info.big_context { DeviceContextArray::new_big() } else { DeviceContextArray::new_normal() });
            let ctx_ptr = self.get_ptr::<DeviceContextArray>(dev_ctx.as_ref());

            self.flush_struct::<DeviceContextArray>(dev_ctx.as_ref(), FlushType::Clean);

            // Activate Entry
            self.device_contexts[slot - 1] = Some(dev_ctx);
            self.device_context_baa.as_mut().expect("").entries[slot] = ctx_ptr;

            self.flush_struct::<DeviceContextBaseAddressArray>(self.device_context_baa.as_ref().unwrap(), FlushType::Clean);
        }

        self.create_slot_context(&mut input_ctx, port_ctx);
        self.configure_endpoint(slot as u8, input_ctx.as_mut(),
                                0, EP_TYPE_CONTROL_BIDIR, port_ctx.max_packet_size,
                                0, 0);

        input_ctx.get_input_mut()[1] = 0b11;

        H::flush_cache(input_ctx.get_ptr_va(), input_ctx.get_size() as u64, FlushType::Clean);

        let input_ctx_ptr = H::translate_addr(input_ctx.get_ptr_va());
        let ptr = self.command_ring.as_mut().expect("").push(
            TRB { command: CommandTRB::address_device(slot as u8, input_ctx_ptr, block_cmd) }
        );
        self.wait_command_complete(ptr).expect("command_complete");

        H::sleep(Duration::from_millis(10));

        input_ctx
    }

    fn send_control_command(&mut self, slot_id: u8, request_type: TypeTriple, request: u8,
                            value: u16, index: u16, length: u16,
                            write_to_usb: Option<&[u8]>, read_from_usb: Option<&mut [u8]>)
                            -> USBResult<usize>
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
            return USBErrorKind::InvalidArgument.err("cannot have write and read");
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
                H::translate_addr(write_to_usb.as_ref().unwrap().as_ptr() as u64)
            } else {
                H::translate_addr(read_from_usb.as_ref().unwrap().as_ptr() as u64)
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
                        if matches!(code, TRBCompletionCode::ShortPacket) {
                            warn!("ControlMessage Short Packet Detected, {:?}", t);
                        } else if !matches!(code, TRBCompletionCode::Success) {
                            return Err(code.into());
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
                return USBErrorKind::Timeout.err("xhci send_control_command timed out");
            }
        }
    }

    fn get_max_esti_payload(epdesc: &USBEndpointDescriptor) -> u32 {
        match epdesc.transfer_type() {
            EndpointType::Control | EndpointType::Bulk => 0,
            _ => {
                let maxp_mult = (epdesc.wMaxPacketSize >> 11) & 0x3;
                maxp_mult as u32 * epdesc.wMaxPacketSize as u32
            }
        }
    }

    fn with_input_context<R, F: FnOnce(&mut Self, &mut InputContext) -> USBResult<R>>(&mut self, slot: u8, func: F) -> USBResult<R> {
        let mut input_ctx = Box::new(if self.info.big_context { InputContext::new_big() } else { InputContext::new_normal() });

        let ctx = self.device_contexts[(slot - 1) as usize].as_ref().expect("No context for slot");
        H::flush_cache(ctx.get_ptr_va(), ctx.get_size() as u64, FlushType::Invalidate);

        *input_ctx.get_slot_mut() = ctx.get_slot().clone();
        // 31 is not a typo
        for i in 0..31 {
            *input_ctx.get_endpoint_mut(i) = ctx.get_endpoint(i).clone();
        }

        let result = func(self, input_ctx.as_mut())?;

        // always update slot context
        input_ctx.get_input_mut()[1] |= 1;

        H::flush_cache(input_ctx.get_ptr_va(), input_ctx.get_size() as u64, FlushType::Clean);
        let input_ctx_ptr = H::translate_addr(input_ctx.get_ptr_va());

        let ptr = self.command_ring.as_mut().expect("").push(
            TRB { command: CommandTRB::configure_endpoint(slot, input_ctx_ptr) }
        );
        self.wait_command_complete(ptr).ok_or(USBErrorKind::Timeout.msg("command did not complete"))?;
        Ok(result)
    }

    fn trigger_ring_doorbell(&mut self, ring: (u8, u8)) {
        self.get_doorbell_register(ring.0).reg.write(ring.1 as u32 + 1);
    }

    fn wait_for_transfer_event(&mut self, ptr: u64) -> Option<TransferEventTRB> {
        H::memory_barrier();
        loop {
            let trb = self.poll_event_ring_trb()?;
            match trb {
                TRBType::TransferEvent(c) => {
                    if let Some(ctx) = self.pending_interrupt_transfers.get_mut(&c.trb_pointer) {
                        ctx.response = Some(c.status);
                    }
                    if c.trb_pointer == ptr {
                        debug!("Got transfer event: {:?} -> bytes remain: {}", c, c.status.get_bytes_remain());
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

    fn transfer_single_bulk_ring(&mut self, ring: (u8, u8), max_packet_size: usize, mut transfer: TransferBuffer) -> USBResult<(usize, bool)> {
        debug!("transfer_single_bulk_ring(ring:{:?}, transfer:{:?})", ring, transfer);
        let mut my_read_buffer = Box::new([0u8; 512]);

        let packet_size = core::cmp::min(max_packet_size, transfer.len());
        let my_buffer = &mut my_read_buffer[..packet_size];

        if let TransferBuffer::Write(slice) = &transfer {
            my_buffer.copy_from_slice(&slice[..packet_size]);
            self.flush_slice(my_buffer.as_ref(), FlushType::Clean);
        }

        if let TransferBuffer::Read(_) = &mut transfer {
            self.flush_slice(my_buffer.as_ref(), FlushType::Invalidate);
        }

        let transfer_ring = self.transfer_rings.get(&ring).unwrap();
        let mut transfer_ring = transfer_ring.lock();

        let normal = NormalTRB::new::<H>(my_buffer.as_ref(), my_buffer.len());

        let ptr = transfer_ring.push(TRB { normal });

        core::mem::drop(transfer_ring);

        self.trigger_ring_doorbell(ring);

        let event = self.wait_for_transfer_event(ptr).ok_or(USBErrorKind::Timeout.msg("timed out waiting for transfer response"))?;
        let code = TRBCompletionCode::from(event.status.get_code());
        if !matches!(code, TRBCompletionCode::Success | TRBCompletionCode::ShortPacket) {
            return Err(code.into());
        }
        if matches!(code, TRBCompletionCode::ShortPacket) {
            warn!("got a short packet: {:?}", event);
        }

        let can_request_more = !matches!(code, TRBCompletionCode::ShortPacket);
        let amount_transferred = packet_size - event.status.get_bytes_remain() as usize;

        if let TransferBuffer::Read(slice) = &mut transfer {
            self.flush_slice(my_buffer.as_ref(), FlushType::Invalidate);
            (&mut slice[..amount_transferred]).copy_from_slice(&my_buffer[..amount_transferred]);
        }

        Ok((amount_transferred, can_request_more))
    }

    fn transfer_bulk_ring(&mut self, ring: (u8, u8), max_packet_size: usize, mut transfer: TransferBuffer) -> USBResult<usize> {
        let original_length = transfer.len();
        loop {
            if transfer.len() == 0 {
                return Ok(original_length);
            }

            let (amount, has_more) = self.transfer_single_bulk_ring(ring, max_packet_size, transfer.clone_mut())?;
            if amount == 0 || !has_more {
                return Ok((original_length - transfer.len()) + amount);
            }

            transfer = match transfer {
                TransferBuffer::Read(slice) => TransferBuffer::Read(&mut slice[amount..]),
                TransferBuffer::Write(slice) => TransferBuffer::Write(&slice[amount..]),
                TransferBuffer::None => TransferBuffer::None,
            };
        }
    }

    pub fn do_stuff(&mut self) -> USBResult<()> {
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
                return USBErrorKind::Other.err("PageSize > 4096 not supported");
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
        // self.poll_ports();
        let crcr = self.op.command_ring_control.read();
        info!("current crcr: {:#x}", crcr);

        Ok(())
    }

    fn initialize_memory_structures(&mut self) -> USBResult<()> {
        // Step 1: Setup Device Context Base Address Array
        if self.device_context_baa.is_none() {
            self.device_context_baa = Some(Box::new(DeviceContextBaseAddressArray::default()));
        }

        let dcbaa_pa = self.get_ptr::<DeviceContextBaseAddressArray>(self.device_context_baa.as_ref().unwrap());
        self.op.device_context_base_addr_array_ptr.write(dcbaa_pa);

        debug!("[XHCI] DCBAA Setup complete");

        // Step 2: Setup Command Ring (CRCR)
        if self.command_ring.is_none() {
            self.command_ring = Some(XHCIRing::<H>::new_with_capacity(1, true));
        }

        for i in 0..EVENT_RING_NUM_SEGMENTS {
            H::flush_cache(self.command_ring.as_ref().unwrap().segments[i].as_ref() as *const XHCIRingSegment as u64, core::mem::size_of::<XHCIRingSegment>() as u64, FlushType::Clean);
        }

        let crcr_pa = self.get_ptr::<XHCIRingSegment>(self.command_ring.as_ref().unwrap().segments[0].as_ref());

        let initial_crcr = self.op.command_ring_control.read();
        debug!("[XHCI] CRCR initial {:x}", initial_crcr);
        {
            if initial_crcr & OP_CRCR_CRR_MASK != 0 {
                return USBErrorKind::Other.err("CrCr is Running");
            }

            let cyc_state = self.command_ring.as_ref().unwrap().cycle_state as u64;
            assert_eq!(crcr_pa & 0b111111, 0, "alignment");
            let val64 = (initial_crcr & OP_CRCR_RES_MASK) |
                (crcr_pa & OP_CRCR_CRPTR_MASK) |
                (cyc_state & OP_CRCR_CS_MASK);

            H::memory_barrier();
            self.op.command_ring_control.write(val64);
            H::memory_barrier();
            self.get_doorbell_register(0).reg.write(0);
            H::memory_barrier();
            for i in 0..EVENT_RING_NUM_SEGMENTS {
                H::flush_cache(self.command_ring.as_ref().unwrap().segments[i].as_ref() as *const XHCIRingSegment as u64, core::mem::size_of::<XHCIRingSegment>() as u64, FlushType::Clean);
            }

            let crcr = self.op.command_ring_control.read();
            info!("current crcr: {:#x}", crcr);
        }

        debug!("[XHCI] CRCR Setup complete");

        // Setup Event Ring
        self.event_ring = Some(XHCIRing::<H>::new_with_capacity(EVENT_RING_NUM_SEGMENTS, false));
        self.event_ring_table = Some(Box::new(EventRingSegmentTable::default()));
        self.event_ring_table.as_mut().unwrap().segment_count = EVENT_RING_NUM_SEGMENTS;

        for idx in 0..EVENT_RING_NUM_SEGMENTS {
            let pa = self.get_ptr::<XHCIRingSegment>(self.event_ring.as_ref().unwrap().segments[idx].as_ref());

            let ent = self.event_ring_table.as_mut().unwrap();
            ent.segments[idx].segment_size = TRBS_PER_SEGMENT as u32;
            assert_eq!(pa & 0b11_1111, 0, "alignment");
            ent.segments[idx].addr = pa;
        }

        H::flush_cache(self.event_ring_table.as_deref().unwrap() as *const EventRingSegmentTable as u64, core::mem::size_of::<EventRingSegmentTable>() as u64, FlushType::Clean);

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
                ScratchPadBufferArray::new_with_capacity::<H>(num_sp as usize, self.info.page_size as usize)
            ));
            let scratch_pa = self.get_ptr::<ScratchPadBufferArray>(self.scratchpads.as_ref().unwrap().as_ref());
            self.device_context_baa.as_mut().unwrap().entries[0] = scratch_pa;
        }

        // Zero device notification
        self.op.dnctlr.write(0x0);

        Ok(())
    }

    fn start(&mut self) -> USBResult<()> {
        debug!("[XHCI] Starting the controller");

        self.op.command.update(|reg| {
            *reg |= OP_CMD_RUN_STOP_MASK | OP_CMD_HSERR_EN_MASK;
            // TODO maybe don't disable
            *reg &= !OP_CMD_INT_EN_MASK; // DISABLE INTERRUPT
        });

        self.wait_until(USBErrorKind::Timeout.msg("did not start!"), HALT_TIMEOUT, |this| {
            this.op.status.read() & OP_STS_HLT_MASK == 0
        })?;

        debug!("[XHCI] Started.");

        Ok(())
    }

    fn poll_once_event_ring_trb(&mut self) -> Option<TRBType> {
        let pop = self.event_ring.as_mut().expect("").pop(false);
        match pop {
            Some(trb) => {
                let tmp = self.event_ring.as_ref().expect("").dequeue_pointer() |
                    INT_ERDP_BUSY_MASK | (INT_ERDP_DESI_MASK & self.event_ring.as_ref().expect("").dequeue.0 as u64);
                self.get_runtime_interrupt_register(0).event_ring_deque_ptr.write(tmp);
                let lol = TRBType::from(trb);
                Some(lol)
            }
            None => None
        }
    }

    fn poll_event_ring_trb(&mut self) -> Option<TRBType> {
        let timeout = Duration::from_millis(1000) + H::current_time();
        loop {
            if let Some(trb) = self.poll_once_event_ring_trb() {
                return Some(trb);
            }
            if H::current_time() > timeout {
                return None;
            }
            H::sleep(Duration::from_millis(1));
        }
    }

    #[track_caller]
    fn wait_command_complete(&mut self, ptr: u64) -> Option<CommandCompletionTRB> {
        H::memory_barrier();
        // TODO update this code to use interrupt notification system
        self.get_doorbell_register(0).reg.write(0);
        loop {
            let trb = self.poll_event_ring_trb()?;
            match trb {
                TRBType::TransferEvent(c) => {
                    if let Some(ctx) = self.pending_interrupt_transfers.get_mut(&c.trb_pointer) {
                        ctx.response = Some(c.status);
                    }
                }
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

    fn is_hub_port_connected(&self, port_id: u8) -> bool {
        let port_op = self.op.get_port_operational_register(port_id);
        let mut port_status = port_op.portsc.read();
        if port_status & OP_PORT_STATUS_POWER_MASK == 0 {
            debug!("[XHCI] Port {} not powered. Powering On", port_id);
            let tmp = (port_status & !OP_PORT_STATUS_PED_MASK) | OP_PORT_STATUS_POWER_MASK;
            port_op.portsc.write(tmp);
            while port_op.portsc.read() & OP_PORT_STATUS_POWER_MASK == 0 {}
            H::sleep(Duration::from_millis(20));
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

    pub fn extended_capability(&self) -> ExtendedCapabilityTags {
        // The higher 16bits specify the offset from base,
        // unit is **DWORD**
        let hcc1val = self.cap.hcc_param1.read();
        let cap_list_offset = (hcc1val >> 14) & !0b11u32; // LSH 2 so it's in bytes
        let cap_list_base = self.mmio_virt_base + cap_list_offset as u64;
        ExtendedCapabilityTags::get(cap_list_base as usize)
    }

    /// This function transfer the ownership of the controller from BIOS
    pub fn transfer_ownership(&mut self) -> USBResult<()> {
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
                        return USBErrorKind::Other.err("UnexpectedOwnership");
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

    fn handle_root_hub_command(&mut self, command: ControlCommand) -> USBResult<()> {
        let port = command.index as u8;

        match (command.request_type, command.request) {
            (request_type!(HostToDevice, Class, Other), REQUEST_SET_FEATURE) => {
                match command.value as u8 {
                    FEATURE_PORT_POWER => {}
                    FEATURE_PORT_RESET => {
                        debug!("root hub reset port {}", port);
                        self.op.get_port_operational_register(port).portsc.update(|tmp| {
                            *tmp &= !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
                            *tmp |= OP_PORT_STATUS_RESET_MASK | OP_PORT_STATUS_PRC_MASK;
                        });
                    }
                    c => panic!("unexpected SET_FEATURE: {}", c),
                }
            }
            (request_type!(HostToDevice, Class, Other), REQUEST_CLEAR_FEATURE) => {
                match command.value as u8 {
                    FEATURE_C_PORT_CONNECTION => {}
                    FEATURE_C_PORT_RESET => {
                        debug!("root hub clear port did reset {}", port);
                        self.op.get_port_operational_register(port).portsc.update(|tmp| {
                            *tmp &= !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
                            *tmp |= OP_PORT_STATUS_PRC_MASK;
                        });
                    }
                    c => panic!("unexpected REQUEST_CLEAR_FEATURE: {}", c),
                }
            }
            (request_type!(DeviceToHost, Class, Other), REQUEST_GET_STATUS) => {
                let mut status = PortStatus::default();

                let val = self.op.get_port_operational_register(port).portsc.read();
                if (val & OP_PORT_STATUS_RESET_MASK == 0) && (OP_PORT_STATUS_PRC_MASK != 0) {
                    status.set_change_reset(true);
                }

                if self.is_hub_port_connected(port) {
                    status.set_device_connected(true);

                    let portsc = self.op.get_port_operational_register(port).portsc.read();
                    match ((portsc & OP_PORT_STATUS_SPEED_MASK) >> OP_PORT_STATUS_SPEED_SHIFT) as u8 {
                        OP_PORT_STATUS_SPEED_LOW => status.set_low_speed(true),
                        OP_PORT_STATUS_SPEED_HIGH => status.set_high_speed(true),
                        OP_PORT_STATUS_SPEED_FULL => {}
                        c => {
                            warn!("Unknown speed: {}", c)
                        }
                    }
                }

                respond_slice(command, as_slice(&status));
            }
            (request_type!(DeviceToHost, Standard, Device), REQUEST_GET_DESCRIPTOR) => {
                let (desc_type, desc_index) = ((command.value >> 8) as u8, command.value as u8);

                match desc_type {
                    DESCRIPTOR_TYPE_DEVICE => {
                        let mut desc = USBDeviceDescriptor::default();
                        desc.bLength = 18;
                        desc.bDescriptorType = DESCRIPTOR_TYPE_DEVICE;
                        desc.bcdUSB = 512;
                        desc.bDeviceClass = 9;
                        desc.bDeviceSubClass = 0;
                        desc.bDeviceProtocol = 1;
                        desc.bMaxPacketSize = 9;
                        desc.idVendor = 0x1337;
                        desc.idProduct = 0xcafe;
                        desc.bcdDevice = 256;
                        desc.bNumConfigurations = 1;

                        respond_slice(command, as_slice(&desc));
                    }
                    DESCRIPTOR_TYPE_CONFIGURATION => {
                        assert_eq!(core::mem::size_of::<USBConfigurationDescriptor>(), 9);

                        let total_length = core::mem::size_of::<USBConfigurationDescriptor>() + core::mem::size_of::<USBInterfaceDescriptor>();

                        let mut desc = USBConfigurationDescriptor::default();
                        desc.bLength = 9;
                        desc.bDescriptorType = DESCRIPTOR_TYPE_CONFIGURATION;
                        desc.wTotalLength = total_length as u16;
                        desc.bNumInterface = 1;
                        desc.bConfigurationValue = 1;

                        let mut interface = USBInterfaceDescriptor::default();
                        interface.bLength = 9;
                        interface.bDescriptorType = DESCRIPTOR_TYPE_INTERFACE;
                        interface.bInterfaceNumber = 1;
                        interface.bAlternateSetting = 0;
                        interface.bNumEndpoints = 0;
                        interface.bInterfaceClass = 9;
                        interface.bInterfaceSubClass = 0;
                        interface.bInterfaceProtocol = 1;
                        interface.iInterface = 0;

                        #[repr(C, packed)]
                        struct FullDescriptor(USBConfigurationDescriptor, USBInterfaceDescriptor);
                        assert_eq!(core::mem::size_of::<FullDescriptor>(), total_length);

                        respond_slice(command, as_slice(&FullDescriptor(desc, interface)));
                    }
                    DESCRIPTOR_TYPE_STRING => {
                        panic!("string? what is string?");
                    }
                    desc => {
                        panic!("got read for unknown descriptor {} on command {:?}", desc, command);
                    }
                }
            }
            (request_type!(DeviceToHost, Class, Device), REQUEST_GET_DESCRIPTOR) => {
                let (desc_type, desc_index) = ((command.value >> 8) as u8, command.value as u8);

                match desc_type {
                    DESCRIPTOR_TYPE_HUB => {
                        let mut desc = USBHubDescriptor::default();
                        desc.bLength = core::mem::size_of::<USBHubDescriptor>() as u8;
                        desc.bDescriptorType = DESCRIPTOR_TYPE_HUB;
                        desc.bNbrPorts = self.info.max_port;
                        desc.wHubCharacteristics = USBHubDescriptorHubCharacteristics::default();

                        respond_slice(command, as_slice(&desc));
                    }
                    desc => {
                        panic!("got read for unknown class descriptor {} on command {:?}", desc, command);
                    }
                }
            }
            (request_type!(HostToDevice, Standard, Device), REQUEST_SET_CONFIGURATION) => {
                // TODO not no-op? assertions?
            }
            _ => {
                panic!("got unknown command for hub: {:?}", command);
            }
        }

        Ok(())
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

fn speed_to_xhci(speed: USBSpeed) -> u8 {
    match speed {
        USBSpeed::Low => OP_PORT_STATUS_SPEED_LOW,
        USBSpeed::Full => OP_PORT_STATUS_SPEED_FULL,
        USBSpeed::High => OP_PORT_STATUS_SPEED_HIGH,
        USBSpeed::Super => OP_PORT_STATUS_SPEED_SUPER_G1,
        c => panic!("unknown speed: {:?}", c),
    }
}

pub struct XhciWrapper<H: XhciHAL>(pub Mutex<Xhci<H>>);

struct USBDeviceMeta {
    pub is_root_hub: bool,
    pub slot: u8,

    pub root_port: u8,
    pub parent_slot: u8,
    pub route_string: u32,
    pub parent_is_low_or_full_speed: bool,
}

impl USBMeta for USBDeviceMeta {}

impl<H: XhciHAL> USBHostController for XhciWrapper<H> {
    fn register_root_hub(&self, device: &Arc<RwLock<USBDevice>>) {
        let mut device = device.write();
        device.protocol_meta = Some(Box::new(USBDeviceMeta {
            is_root_hub: true,
            slot: 0,
            root_port: 0,
            parent_slot: 0,
            route_string: 0,
            parent_is_low_or_full_speed: false,
        }));
    }

    fn pipe_open(&self, device: &Arc<RwLock<USBDevice>>, endpoint: Option<&USBEndpointDescriptor>) -> USBResult<Arc<RwLock<USBPipe>>> {
        if endpoint.is_none() {
            let mut dev_lock = device.write();
            if dev_lock.protocol_meta.is_none() {
                let slot = dev_lock.addr as u8;
                assert_ne!(slot, 0);
                dev_lock.protocol_meta.replace(Box::new(USBDeviceMeta {
                    is_root_hub: false,
                    slot,
                    root_port: 0,
                    parent_slot: 0,
                    route_string: 0,
                    parent_is_low_or_full_speed: false,
                }));
            }
        }

        let cloned_device = device.clone();
        let mut dev_lock = device.upgradeable_read();

        assert!(dev_lock.protocol_meta.is_some());

        let cloned_controller = dev_lock.bus.controller.clone();

        let f = dev_lock.protocol_meta.as_ref().unwrap().downcast_ref::<USBDeviceMeta>().unwrap();
        if f.is_root_hub {
            assert!(matches!(endpoint, None));
            return Ok(Arc::new(RwLock::new(USBPipe {
                device: cloned_device,
                controller: cloned_controller,
                index: 0,
                endpoint_type: EndpointType::Control,
                max_packet_size: dev_lock.ddesc.get_max_packet_size() as usize,
                is_input: true,
            })));
        }

        assert_ne!(dev_lock.addr, 0);

        match endpoint {
            None => {
                let mut x = self.0.lock();

                let slot = dev_lock.addr as u8;

                let (speed, max_packet_size) = (dev_lock.speed, dev_lock.ddesc.get_max_packet_size() as u16);
                assert_ne!(speed, USBSpeed::Invalid);
                assert_ne!(max_packet_size, 0);

                let parent_slot: u8;
                let route_string: u32;
                let parent_is_low_or_full_speed: bool;
                let root_port: u8;

                {
                    let parent = dev_lock.parent.as_ref().expect("non-root hub has no parent");
                    let par_lock = parent.read();
                    let f = par_lock.protocol_meta.as_ref().unwrap().downcast_ref::<USBDeviceMeta>().unwrap();

                    if f.is_root_hub {
                        parent_slot = 0;
                        route_string = 0;
                        parent_is_low_or_full_speed = false;
                        root_port = dev_lock.port;
                    } else {
                        parent_slot = par_lock.addr as u8;
                        route_string = f.route_string | (dev_lock.port as u32) << (4 * (par_lock.depth - 1));
                        parent_is_low_or_full_speed = par_lock.speed.is_low_or_full_speed();
                        root_port = f.root_port;
                    }
                }

                let mut dev_lock = dev_lock.upgrade();
                let mut f = dev_lock.protocol_meta.as_mut().unwrap().downcast_mut::<USBDeviceMeta>().unwrap();

                f.parent_slot = parent_slot;
                f.route_string = route_string;
                f.parent_is_low_or_full_speed = parent_is_low_or_full_speed;
                f.root_port = root_port;

                let ctx = PortContext {
                    port: dev_lock.port,
                    parent_slot,
                    route_string,
                    parent_is_low_or_full_speed,
                    speed: dev_lock.speed,
                    root_port,
                    max_packet_size: dev_lock.ddesc.get_max_packet_size() as u16,
                };

                x.setup_slot(slot, &ctx, true);

                return Ok(Arc::new(RwLock::new(USBPipe {
                    device: cloned_device,
                    controller: cloned_controller,
                    index: 0,
                    endpoint_type: EndpointType::Control,
                    max_packet_size: dev_lock.ddesc.get_max_packet_size() as usize,
                    is_input: true,
                })));
            }
            Some(endpoint_desc) => {
                let slot = dev_lock.addr as u8;

                let endpoint_type = endpoint_desc.transfer_type();
                let endpoint_index = Xhci::<H>::get_epctx_index(endpoint_desc.bEndpointAddress);
                let endpoint_is_input = Xhci::<H>::is_ep_input(endpoint_desc.bEndpointAddress);

                let endpoint_type_value = match (&endpoint_type, endpoint_is_input) {
                    (EndpointType::Control, _) => EP_TYPE_CONTROL_BIDIR,
                    (EndpointType::Isochronous, true) => EP_TYPE_ISOCH_IN,
                    (EndpointType::Isochronous, false) => EP_TYPE_ISOCH_OUT,
                    (EndpointType::Bulk, true) => EP_TYPE_BULK_IN,
                    (EndpointType::Bulk, false) => EP_TYPE_BULK_OUT,
                    (EndpointType::Interrupt, true) => EP_TYPE_INTERRUPT_IN,
                    (EndpointType::Interrupt, false) => EP_TYPE_INTERRUPT_OUT,
                };

                let mut x = self.0.lock();
                x.with_input_context(slot, |this, input_ctx| {
                    this.configure_endpoint(slot, input_ctx,
                                            endpoint_desc.bEndpointAddress,
                                            endpoint_type_value, endpoint_desc.wMaxPacketSize,
                                            endpoint_desc.bInterval, Xhci::<H>::get_max_esti_payload(endpoint_desc));

                    // input_ctx.set_configure_ep_meta(configuration.config.config_val,
                    //                                 interface.interface.interface_number, interface.interface.alt_set);

                    Ok(())
                }).unwrap_or_else(|e| panic!("bigly oof: {:?}", e));

                return Ok(Arc::new(RwLock::new(USBPipe {
                    device: cloned_device,
                    controller: cloned_controller,
                    index: endpoint_index,
                    endpoint_type,
                    max_packet_size: endpoint_desc.wMaxPacketSize as usize,
                    is_input: endpoint_is_input,
                })));
            }
        }
    }

    fn set_address(&self, device: &Arc<RwLock<USBDevice>>, addr: u32) -> USBResult<()> {
        let dev_lock = device.read();
        let f = dev_lock.protocol_meta.as_ref().unwrap().downcast_ref::<USBDeviceMeta>().unwrap();
        if f.is_root_hub {
            return Ok(());
        }

        let ctx = PortContext {
            port: dev_lock.port,
            parent_slot: f.parent_slot,
            route_string: f.route_string,
            parent_is_low_or_full_speed: f.parent_is_low_or_full_speed,
            speed: dev_lock.speed,
            root_port: f.root_port,
            max_packet_size: dev_lock.ddesc.get_max_packet_size() as u16,
        };

        let mut x = self.0.lock();
        x.setup_slot(addr as u8, &ctx, false);
        Ok(())
    }

    fn configure_hub(&self, device: &Arc<RwLock<USBDevice>>, nbr_ports: u8, ttt: u8) -> USBResult<()> {

        // Nothing needs to be done for a root hub, since it is "fake"
        if device.read().parent.is_none() {
            return Ok(());
        }

        let slot = device.read().addr as u8;
        let mut x = self.0.lock();
        let mut input_ctx = Box::new(if x.info.big_context { InputContext::new_big() } else { InputContext::new_normal() });

        let ctx = x.device_contexts[(slot - 1) as usize].as_ref().expect("No context for slot");
        H::flush_cache(ctx.get_ptr_va(), ctx.get_size() as u64, FlushType::Invalidate);

        *input_ctx.get_slot_mut() = ctx.get_slot().clone();
        let slot_ctx = input_ctx.get_slot_mut();
        slot_ctx.numbr_ports = nbr_ports;
        slot_ctx.interrupter_ttt = ttt as u16;
        slot_ctx.dword1.set_hub(true);

        // update slot context
        input_ctx.get_input_mut()[1] = 0b1;

        H::flush_cache(input_ctx.get_ptr_va(), input_ctx.get_size() as u64, FlushType::Clean);
        let input_ctx_ptr = H::translate_addr(input_ctx.get_ptr_va());

        let ptr = x.command_ring.as_mut().expect("").push(
            TRB { command: CommandTRB::configure_endpoint(slot, input_ctx_ptr) }
        );
        return match x.wait_command_complete(ptr) {
            Some(trb) => {
                if trb.code == TRBCompletionCode::Success as u8 {
                    Ok(())
                } else {
                    let code = TRBCompletionCode::from(trb.code);
                    warn!("TRB Completion with non-success code: {:?} -> {:?}", code, trb);
                    Err(code.into())
                }
            }
            None => {
                Err(USBErrorKind::Timeout.msg("no response received"))
            }
        };
    }

    fn control_transfer(&self, endpoint: &USBPipe, command: ControlCommand) -> USBResult<()> {
        assert_eq!(endpoint.index, 0);
        let dev_lock = endpoint.device.read();

        let meta = dev_lock.protocol_meta.as_ref().unwrap().downcast_ref::<USBDeviceMeta>().unwrap();
        if meta.is_root_hub {
            let mut x = self.0.lock();
            return x.handle_root_hub_command(command);
        }

        let mut x = self.0.lock();
        match command.buffer {
            TransferBuffer::Read(slice) => x.send_control_command(meta.slot, command.request_type, command.request, command.value, command.index, command.length, None, Some(slice)),
            TransferBuffer::Write(slice) => x.send_control_command(meta.slot, command.request_type, command.request, command.value, command.index, command.length, Some(slice), None),
            TransferBuffer::None => x.send_control_command(meta.slot, command.request_type, command.request, command.value, command.index, command.length, None, None),
        }.map(|_| ())
    }

    fn bulk_transfer(&self, endpoint: &USBPipe, buffer: TransferBuffer) -> USBResult<usize> {
        assert!(matches!(endpoint.endpoint_type, EndpointType::Bulk));

        let dev_lock = endpoint.device.read();

        let ring = (dev_lock.addr as u8, endpoint.index);

        let mut x = self.0.lock();
        x.transfer_bulk_ring(ring, endpoint.max_packet_size, buffer)
    }

    fn async_read(&self, endpoint: &USBPipe, buf: Vec<u8>, int_callback: USBAsyncReadFn) -> USBResult<()> {

        // TODO this code does not ensure that buffers are cache aligned. may have problems on ARM
        H::flush_cache(buf.as_ptr() as u64, buf.len() as u64, FlushType::CleanAndInvalidate);

        assert!(matches!(endpoint.endpoint_type, EndpointType::Bulk | EndpointType::Interrupt));
        assert!(endpoint.is_input);

        let dev_lock = endpoint.device.read();
        let ring_addr = (dev_lock.addr as u8, endpoint.index);

        let packet_size = core::cmp::min(endpoint.max_packet_size, buf.len());

        let mut x = self.0.lock();

        let ring = x.transfer_rings.get(&ring_addr).ok_or(USBErrorKind::InvalidArgument.msg("invalid pipe has no ring"))?;
        let mut ring_lock = ring.lock();

        let normal = NormalTRB::new::<H>(buf.as_slice(), packet_size);
        let ptr = ring_lock.push(TRB { normal });

        core::mem::drop(ring_lock);

        x.pending_interrupt_transfers.insert(ptr, IntTransferContext {
            slot: ring_addr.0,
            buf,
            callback: int_callback,
            packet_size,
            response: None,
        });

        x.trigger_ring_doorbell(ring_addr);

        Ok(())
    }

    fn allocate_slot(&self) -> USBResult<u8> {
        let mut x = self.0.lock();
        x.send_slot_enable()
    }

    fn free_slot(&self, slot: u8) {
        debug!("free_slot(): leaking slot={}", slot);
    }

    fn process_interrupts(&self) {

        let mut x = self.0.lock();

        // process the full event ring...
        while let Some(trb) = x.poll_once_event_ring_trb() {
            match trb {
                TRBType::TransferEvent(c) => {
                    if let Some(ctx) = x.pending_interrupt_transfers.get_mut(&c.trb_pointer) {
                        ctx.response = Some(c.status);
                    }
                }
                e => {
                    warn!("interrupt ate trb: {:?}", e);
                }
            }
        }

        let mut done_trbs: Vec<u64> = Vec::new();
        for (ptr, ctx) in x.pending_interrupt_transfers.iter() {
            if ctx.response.is_some() {
                done_trbs.push(*ptr);
            }
        }

        let mut done_contexts: Vec<IntTransferContext> = Vec::new();
        for ptr in done_trbs.iter() {
            if let Some(ctx) = x.pending_interrupt_transfers.remove(ptr) {
                done_contexts.push(ctx);
            }
        }

        // don't hold the XHCI lock so that the callbacks can queue more interrupts.
        core::mem::drop(x);

        for mut ctx in done_contexts.drain(..) {
            let status = ctx.response.unwrap();

            let code = TRBCompletionCode::from(status.get_code());
            let result = if !matches!(code, TRBCompletionCode::Success | TRBCompletionCode::ShortPacket) {
                Err(code.into())
            } else {
                Ok(ctx.packet_size - status.get_bytes_remain() as usize)
            };

            // we need to take ownership of the buffer because calling callback will consume...
            let buf = core::mem::replace(&mut ctx.buf, Vec::new());

            H::flush_cache(buf.as_ptr() as u64, buf.len() as u64, FlushType::Invalidate);

            (ctx.callback)(buf, result);
        }

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

