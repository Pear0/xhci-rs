use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter};
use core::ops::Deref;

use crate::consts::{TRB_LINK_TOGGLE_MASK, TRBS_PER_SEGMENT};
use crate::HAL;
use crate::trb::{TRB, LinkTRB};

#[repr(C, align(2048))]
pub struct DeviceContextBaseAddressArray {
    pub entries: [u64; 256],
}

impl Default for DeviceContextBaseAddressArray {
    fn default() -> Self {
        Self { entries: [0; 256] }
    }
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
        ring
    }

    pub fn push(&mut self, mut trb: TRB) -> u64 {
        assert_ne!(self.segments.len(), 0, "no segments");
        trb.set_cycle_state(self.cycle_state as u8);
        self.segments[self.enqueue.0].trbs[self.enqueue.1] = trb;
        let ptr_pa = self.hal.translate_addr(&self.segments[self.enqueue.0].trbs[self.enqueue.1] as *const TRB as u64);
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
        ptr_pa
    }

    pub fn pop(&mut self, has_link: bool) -> Option<TRB> {
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