#![feature(llvm_asm)]
#![feature(global_asm)]

#![cfg_attr(not(test), no_std)]

extern crate alloc;
#[macro_use]
extern crate log;

use alloc::boxed::Box;
use core::time::Duration;

use volatile::*;

use crate::consts::*;
use crate::structs::{DeviceContextBaseAddressArray, XHCIRing};

pub(crate) mod consts;
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
    cap: &'static mut XHCICapabilityRegisters,
    op: &'static mut XHCIOperationalRegisters,
    hal: &'a dyn HAL,
    info: XHCIInfo,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    command_ring: Option<XHCIRing<'a>>,
}

pub struct XHCIInfo {
    max_slot: u8,
    /// This field also double as the num of ports,
    /// since the port number starts from 1
    max_port: u8,
    page_size: u32,
    big_context: bool,
}

impl Default for XHCIInfo {
    fn default() -> Self {
        Self {
            max_slot: 0,
            max_port: 0,
            page_size: 0,
            big_context: false,
        }
    }
}


impl<'a> Xhci<'a> {
    pub fn new(base_address: u64, hal: &'a dyn HAL) -> Self {
        let cap = get_registers::<XHCICapabilityRegisters>(base_address);
        let cap_size = (cap.length_and_ver.read() & 0xFF) as u64;

        let op_regs = get_registers::<XHCIOperationalRegisters>(base_address + cap_size);

        info!("Page Size: {}", op_regs.page_size.read());
        info!("Status: {:#x}", op_regs.status.read());

        Self {
            cap,
            op: op_regs,
            hal,
            info: XHCIInfo::default(),
            device_context_baa: None,
            command_ring: None,
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

        self.initialize_memory_structures()?;


        Ok(())
    }

    fn initialize_memory_structures(&mut self) -> Result<(), &'static str> {
        // Step 1: Setup Device Context Base Address Array
        if self.device_context_baa.is_none() {
            self.device_context_baa = Some(Box::new(DeviceContextBaseAddressArray::default()));
        }

        let dcbaa_pa = self.get_ptr(&self.device_context_baa);
        self.op.device_context_base_addr_array_ptr.write(dcbaa_pa);

        debug!("[XHCI] DCBAA Setup complete");

        // Step 2: Setup Command Ring (CRCR)
        if self.command_ring.is_none() {
            self.command_ring = Some(XHCIRing::new_with_capacity(self.hal, 1, true));
        }

        let crcr_pa = self.get_ptr(self.command_ring.as_ref().unwrap().segments[0].as_ref());

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

        trace!("[XHCI] CRCR Setup complete");

        Ok(())
    }
}

pub fn do_stuff(base_address: u64, hal: &dyn HAL) {
    match Xhci::new(base_address, hal).do_stuff() {
        Ok(()) => info!("did stuff successfully"),
        Err(e) => error!("Error failed to do stuff: {}", e),
    }
}


