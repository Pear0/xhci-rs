#![feature(llvm_asm)]
#![feature(global_asm)]
#![feature(allocator_api)]

#![cfg_attr(not(test), no_std)]

extern crate alloc;
#[macro_use]
extern crate log;

use alloc::boxed::Box;
use core::time::Duration;

use volatile::*;

use crate::consts::*;
use crate::structs::{DeviceContextBaseAddressArray, XHCIRing, EventRingSegmentTable, ScratchPadBufferArray, XHCIRingSegment};
use crate::trb::{TRB, CommandTRB, CommandCompletionTRB, TRBType};
use crate::registers::{InterrupterRegisters, DoorBellRegister};
use crate::extended_capability::{ExtendedCapabilityTags, ExtendedCapabilityTag};

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

fn get_registers<T>(base: u64) -> &'static mut T {
    unsafe { &mut *(base as *mut T) }
}

pub trait HAL {
    fn current_time(&self) -> Duration;
    fn sleep(&self, dur: Duration);
    fn memory_barrier(&self);
    fn translate_addr(&self, addr: u64) -> u64;
}

pub struct Xhci<'a> {
    mmio_virt_base: u64,
    cap: &'static mut XHCICapabilityRegisters,
    op: &'static mut XHCIOperationalRegisters,
    hal: &'a dyn HAL,
    info: XHCIInfo,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    command_ring: Option<XHCIRing<'a>>,
    event_ring: Option<XHCIRing<'a>>,
    event_ring_table: Option<Box<EventRingSegmentTable>>,
    scratchpads: Option<Box<ScratchPadBufferArray>>,
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
            command_ring: None,
            event_ring: None,
            event_ring_table: None,
            scratchpads: None,
        }
    }

    fn get_runtime_interrupt_register(&mut self, offset: u8) -> &'static mut InterrupterRegisters {
        let base_ptr = self.cap as *const XHCICapabilityRegisters as u64;
        unsafe {
            &mut *((base_ptr + (self.info.runtime_offset as u64)
                + 0x20 + (offset as u64) * 0x20) as *mut InterrupterRegisters)
        }
    }

    fn get_doorbell_regster(&mut self, offset: u8) -> &'static mut DoorBellRegister {
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

    pub fn reset(&mut self) -> Result<(), &'static str> {
        self.op.command.update(|x| *x |= OP_CMD_RESET_MASK);
        self.hal.memory_barrier();

        self.wait_until("did not reset", RESET_TIMEOUT, |this| {
            let cmd = this.op.command.read();
            let sts = this.op.status.read();
            (cmd & OP_CMD_RESET_MASK == 0) && (sts & OP_STS_CNR_MASK == 0)
        })?;

        Ok(())
    }

    fn get_ptr<T>(&self, t: &T) -> u64 {
        self.hal.translate_addr(t as *const T as u64)
    }

    pub fn do_stuff(&mut self) -> Result<(), &'static str> {
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
                return Err("PageSize > 4096 not supported");
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

        Ok(())
    }

    fn initialize_memory_structures(&mut self) -> Result<(), &'static str> {
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

        let crcr_pa = self.get_ptr::<XHCIRingSegment>(self.command_ring.as_ref().unwrap().segments[0].as_ref());

        let initial_crcr = self.op.command_ring_control.read();
        debug!("[XHCI] CRCR initial {:x}", initial_crcr);
        {
            if initial_crcr & OP_CRCR_CRR_MASK != 0 {
                return Err("CrCr is Running");
            }

            let cyc_state = self.command_ring.as_ref().unwrap().cycle_state as u64;
            assert_eq!(crcr_pa & 0b111111, 0, "alignment");
            let val64 = (initial_crcr & OP_CRCR_RES_MASK) |
                (crcr_pa & OP_CRCR_CRPTR_MASK) |
                (cyc_state & OP_CRCR_CS_MASK);
            self.op.command_ring_control.write(val64);
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

    fn start(&mut self) -> Result<(), &'static str> {
        debug!("[XHCI] Starting the controller");

        self.op.command.update(|reg| {
            *reg |= OP_CMD_RUN_STOP_MASK | OP_CMD_HSERR_EN_MASK;
            // TODO maybe don't disable
            *reg &= !OP_CMD_INT_EN_MASK; // DISABLE INTERRUPT
        });

        self.wait_until("did not start!", HALT_TIMEOUT, |this| {
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
            self.hal.sleep(Duration::from_millis(10));
        }
    }

    fn wait_command_complete(&mut self, ptr: u64) -> Option<CommandCompletionTRB> {
        // TODO update this code to use interrupt notification system
        self.get_doorbell_regster(0).reg.write(0);
        loop {
            let trb = self.poll_event_ring_trb()?;
            match trb {
                TRBType::CommandCompletion(c) => {
                    if c.trb_pointer == ptr {
                        return Some(c);
                    } else {
                        debug!("trb_pointer badddddd");
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
        self.wait_command_complete(ptr).expect("thing");
        debug!("NoOP Complete at {:#x}", ptr);
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
    pub fn transfer_ownership(&mut self) -> Result<(), &'static str> {
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
                        return Err("XHCIError::UnexpectedOwnership");
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

}

pub fn do_stuff(base_address: u64, hal: &dyn HAL) {
    match Xhci::new(base_address, hal).do_stuff() {
        Ok(()) => info!("did stuff successfully"),
        Err(e) => error!("Error failed to do stuff: {}", e),
    }
}


