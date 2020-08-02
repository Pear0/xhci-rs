use alloc::alloc::{AllocInit, AllocRef, Global, Layout};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter};
use core::ops::Deref;
use core::ptr::NonNull;
use modular_bitfield::prelude::*;

use crate::consts::*;
use crate::HAL;
use crate::trb::{LinkTRB, TRB};

#[repr(C, align(2048))]
pub struct DeviceContextBaseAddressArray {
    pub entries: [u64; 256],
}

impl Default for DeviceContextBaseAddressArray {
    fn default() -> Self {
        Self { entries: [0; 256] }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct EventRingSegmentEntry {
    pub addr: u64,
    pub segment_size: u32,
    _res1: u32,
}

impl Default for EventRingSegmentEntry {
    fn default() -> Self {
        Self {
            addr: 0,
            segment_size: 0,
            _res1: 0,
        }
    }
}

impl EventRingSegmentEntry {
    pub fn new(ptr: u64, size: u32) -> EventRingSegmentEntry {
        EventRingSegmentEntry {
            addr: ptr,
            segment_size: size,
            _res1: 0,
        }
    }
}

#[repr(C, align(64))]
#[derive(Default)]
pub struct EventRingSegmentTable {
    pub segments: [EventRingSegmentEntry; 16],
    pub segment_count: usize,
}

/* ----------------------- XHCI Ring ------------------------ */
pub struct XHCIRing<'a> {
    pub segments: Vec<Box<XHCIRingSegment>>,
    pub enqueue: (usize, usize),
    pub dequeue: (usize, usize),
    pub cycle_state: u32,
    hal: &'a dyn HAL,
}

impl<'a> XHCIRing<'a> {
    pub fn new_with_capacity(hal: &'a dyn HAL, segments: usize, link_trbs: bool) -> XHCIRing {
        let mut ring = XHCIRing {
            segments: Vec::new(),
            enqueue: (0, 0),
            dequeue: (0, 0),
            /*
             * The ring is initialized to 0. The producer must write 1 to the
             * cycle bit to handover ownership of the TRB, so PCS = 1.
             * The consumer must compare CCS to the cycle bit to
             * check ownership, so CCS = 1.
             */
            cycle_state: 1, // Ring is initialized to 0, thus cycle state = 1
            hal,
        };
        for idx in 0..segments {

            ring.segments.push(Box::new(XHCIRingSegment::default()));
            if link_trbs {
                if idx > 0 {
                    let ptr_pa = hal.translate_addr(ring.segments[idx].deref() as *const XHCIRingSegment as u64);
                    ring.segments[idx - 1].link_segment(ptr_pa);
                }
                if idx == segments - 1 {
                    let ptr_pa = hal.translate_addr(ring.segments[0].deref() as *const XHCIRingSegment as u64);
                    ring.segments[idx].link_segment(ptr_pa);
                    unsafe { ring.segments[idx].trbs[TRBS_PER_SEGMENT - 1].link.link_flags |= TRB_LINK_TOGGLE_MASK };
                }
            }
        }

        for seg in ring.segments.iter() {
            hal.flush_cache(seg.as_ref() as *const XHCIRingSegment as u64, core::mem::size_of::<XHCIRingSegment>() as u64);
        }

        ring
    }

    pub fn push(&mut self, mut trb: TRB) -> u64 {
        assert_ne!(self.segments.len(), 0, "no segments");
        trb.set_cycle_state(self.cycle_state as u8);
        self.segments[self.enqueue.0].trbs[self.enqueue.1] = trb;
        let ptr_va = &self.segments[self.enqueue.0].trbs[self.enqueue.1] as *const TRB as u64;
        let ptr_pa = self.hal.translate_addr(ptr_va);
        if self.enqueue.1 < (TRBS_PER_SEGMENT - 2) { // Last element is Link
            self.enqueue.1 += 1;
        } else {
            if self.enqueue.0 < self.segments.len() - 1 {
                self.enqueue.0 += 1;
            } else {
                // Toggle Cycle State
                debug!("[XHCI] Toggling State");
                self.segments[self.enqueue.0].trbs[TRBS_PER_SEGMENT - 1].set_cycle_state(self.cycle_state as u8);
                self.cycle_state = if self.cycle_state == 0 { 1 } else { 0 };
                self.enqueue.0 = 0;
            }
            self.enqueue.1 = 0;
        }
        self.hal.flush_cache(ptr_va, core::mem::size_of::<TRB>() as u64);
        ptr_pa
    }

    pub fn pop(&mut self, has_link: bool) -> Option<TRB> {
        self.hal.flush_cache((&self.segments[self.dequeue.0].trbs[self.dequeue.1]) as *const TRB as u64, 16);
        let trb = self.segments[self.dequeue.0].trbs[self.dequeue.1].clone();
        if trb.get_cycle_state() != self.cycle_state as u8 {
            return None;
        }
        if self.dequeue.1 < (TRBS_PER_SEGMENT - (if has_link { 2 } else { 1 })) {
            self.dequeue.1 += 1;
        } else {
            self.dequeue.1 = 0;
            if self.dequeue.0 < self.segments.len() - 1 {
                self.dequeue.0 += 1;
            } else {
                self.dequeue.0 = 0;
                self.cycle_state = if self.cycle_state == 0 { 1 } else { 0 };
            }
        }
        Some(trb)
    }

    pub fn dequeue_pointer(&self) -> u64 {
        self.hal.translate_addr(&self.segments[self.dequeue.0].trbs[self.dequeue.1] as *const TRB as u64)
    }
}

impl Debug for XHCIRing<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "XHCIRing {{ enq: {:?}, deq: {:?}, cycle_state: {}, num_segs: {} }}",
               self.enqueue, self.dequeue, self.cycle_state, self.segments.len())
    }
}

/* --------------------- XHCI Ring Segment ---------------- */
#[repr(C, align(4096))]
pub struct XHCIRingSegment {
    pub trbs: [TRB; TRBS_PER_SEGMENT],
}

impl Default for XHCIRingSegment {
    fn default() -> Self {
        XHCIRingSegment {
            trbs: [Default::default(); TRBS_PER_SEGMENT],
        }
    }
}

impl XHCIRingSegment {
    /// Make the prev segment point to the next segment.
    /// Change the last TRB in the prev segment to be a Link TRB which points to the
    /// address of the next segment.  The caller needs to set any Link TRB
    /// related flags, such as End TRB, Toggle Cycle, and no snoop.
    pub fn link_segment(&mut self, other: u64) {
        let link_trb = LinkTRB::new(other);
        self.trbs[TRBS_PER_SEGMENT - 1] = TRB { link: link_trb };
    }
}


/* ------------- Device Context ------------- */

#[repr(C, align(2048))]
pub struct InputContext {
    pub input: [u32; 8],
    pub slot: SlotContext,
    pub endpoint: [EndpointContext; 31],
}

#[repr(C, align(2048))]
#[derive(Default, Clone)]
pub struct DeviceContextArray {
    pub slot: SlotContext,
    pub endpoint: [EndpointContext; 31],
}

macro_rules! set_field {
    ($var: expr, $shift: expr, $mask: expr, $val: expr) => {{
        let tmp = $var & (!$mask);
        $var = tmp | (($val << $shift) & $mask);
    }};
}

#[repr(C)]
#[derive(Default, Clone)]
pub struct EndpointContext {
    dword1: u32,
    pub flags1: u8,
    pub max_burst_size: u8,
    pub max_packet_size: u16,
    pub dequeu_pointer: u64,
    pub average_trb_len: u16,
    max_esit_payload_lo: u16,
    _res0: [u32; 3],
}

impl EndpointContext {
    pub fn set_lsa_bit(&mut self) {
        self.dword1 |= EP_CTX_LSA_MASK;
    }

    pub fn set_cerr(&mut self, val: u8) {
        set_field!(self.flags1,
            EP_CTX_CERR_SHIFT, EP_CTX_CERR_MASK,
            val
        );
    }

    pub fn set_ep_type(&mut self, val: u8) {
        set_field!(self.flags1,
            EP_CTX_EPTYPE_SHIFT, EP_CTX_EPTYPE_MASK,
            val
        );
    }
}

#[bitfield]
#[derive(Debug, Copy, Clone, Default)]
pub struct SlotContextDW1 {
    route_string: B20,
    speed: B4,
    resz: B1,
    mtt: bool,
    hub: bool,
    context_entries: B5
}

#[repr(C)]
#[derive(Default, Clone, Debug)]
pub struct SlotContext {
    // DWORD1
    pub dword1: SlotContextDW1,
    // DWORD2
    pub max_exit_latency: u16,
    pub root_hub_port_number: u8,
    pub numbr_ports: u8,
    // DWORD3
    pub hub_slot_id: u8,
    pub tt_port_number: u8,
    pub interrupter_ttt: u16,
    // DWORD 4
    pub device_addr: u8,
    _res0: [u8; 2],
    pub slot_state: u8,
    // DWORD 5-8
    _res1: [u32; 4],
}


/* ------------- Scratchpad ----------------- */
#[repr(C, align(4096))]
pub struct ScratchPadBufferArray {
    scratchpads: [u64; 1024],
    scratchpad_vas: [u64; 1024],
    page_size: usize,
}

impl ScratchPadBufferArray {
    pub fn new_with_capacity<'a>(hal: &'a dyn HAL, num: usize, page_size: usize) -> Self {
        assert!(num <= 1024, "unsupported count > 1024");
        let mut thing = Self {
            scratchpads: [0; 1024],
            scratchpad_vas: [0; 1024],
            page_size,
        };
        for i in 0..num {
            let ptr = Global.alloc(Layout::from_size_align(page_size, page_size).expect("alignment"),
                                   AllocInit::Zeroed).expect("alloc failed").ptr.as_ptr() as u64;
            thing.scratchpad_vas[i] = ptr;
            thing.scratchpads[i] = hal.translate_addr(ptr);
        }
        thing
    }
}

impl Drop for ScratchPadBufferArray {
    fn drop(&mut self) {
        debug!("[XHCI] Freeing scratchpad buffers");
        for pad in self.scratchpad_vas.iter() {
            if *pad != 0 {
                unsafe {
                    Global.dealloc(NonNull::<u8>::new_unchecked((*pad) as *mut u8),
                                   Layout::from_size_align(self.page_size, self.page_size)
                                       .expect("align"))
                };
            }
        }
    }
}





