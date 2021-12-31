#![cfg_attr(not(test), no_std)]

use core::ops::Range;

#[derive(Clone, Copy, Debug)]
pub struct BufferPointer<const N: usize> {
    pos: usize,
}

impl<const N: usize> BufferPointer<N> {
    pub const ZERO: Self = Self::new();

    pub const fn new() -> Self {
        Self {
            pos: 0,
        }
    }

    pub fn advance(&mut self, amount: usize) -> usize {
        let new_pos = self.pos + amount;
        self.pos = if new_pos >= N * 2 { new_pos - N * 2 } else { new_pos };
        self.pos
    }

    pub fn phase(&self) -> bool {
        self.pos >= N
    }

    pub fn index(&self) -> usize {
        if self.pos >= N { self.pos - N } else { self.pos }
    }

    pub fn distance(from: Self, to: Self) -> usize {
        let to_pos = if to.pos < from.pos { to.pos + N * 2} else { to.pos };
        let distance = to_pos - from.pos;
        if distance > N { distance - N } else { distance }
    }

    pub fn continuous_blocks<F: FnMut(Range<usize>) -> bool>(from: Self, to: Self, mut f: F) {
        let distance_to_top = Self::distance(from, Self::ZERO);
        let distance = Self::distance(from, to);
        if distance > distance_to_top {
            if f(from.index()..N) {
                f(0..to.index());
            }
        } else {
            f(from.index()..to.index());
        }
    }
}



pub struct BlockBuffer<const N: usize> {
    buffer: [u8; N],
    rpos: BufferPointer<N>,
    wpos: BufferPointer<N>,
}

impl<const N: usize> BlockBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; N],
            rpos: BufferPointer::new(),
            wpos: BufferPointer::new(),
        }
    }

    pub fn clear(&mut self) {
        self.rpos = BufferPointer::new();
        self.wpos = BufferPointer::new();
    }
    pub fn available(&self) -> usize {
        BufferPointer::distance(self.rpos, self.wpos)
    }
    pub fn space(&self) -> usize {
        BufferPointer::distance(self.wpos, self.rpos)
    }
    
    pub fn read<F: FnMut(&[u8]) -> usize>(&mut self, mut f: F)  {
        if self.available() > 0 {
            BufferPointer::continuous_blocks(self.rpos, self.wpos, |range| {
                let len = range.len();
                let read_count = f(&self.buffer[range]);
                self.rpos.advance(read_count);
                read_count == len   // Return if we can process the next block or not
            })
        }
    }
    pub fn write<F: FnMut(&mut [u8]) -> usize>(&mut self, mut f: F)  {
        if self.space() > 0 {
            BufferPointer::continuous_blocks(self.wpos, self.rpos, |range| {
                let len = range.len();
                let write_count = f(&mut self.buffer[range]);
                self.wpos.advance(write_count);
                write_count == len  // Return if we can process the next block or not
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{BufferPointer, BlockBuffer};
    #[test]
    fn buffer_pointer_zero() {
        let p = BufferPointer::<1>::ZERO;
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), false);
    }
    #[test]
    fn buffer_pointer_advance_one() {
        let mut p = BufferPointer::<2>::new();
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), false);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 0);
        p.advance(1);
        assert_eq!(p.index(), 1);
        assert_eq!(p.phase(), false);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 1);
        p.advance(1);
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), true);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 2);
        p.advance(1);
        assert_eq!(p.index(), 1);
        assert_eq!(p.phase(), true);
        p.advance(1);
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), false);
    }
    #[test]
    fn buffer_pointer_advance_two() {
        let mut p = BufferPointer::<2>::new();
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), false);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 0);
        p.advance(2);
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), true);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 2);
        p.advance(2);
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), false);
    }
    #[test]
    fn buffer_pointer_4_advance_two() {
        let mut p = BufferPointer::<4>::new();
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), false);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 0);
        p.advance(2);
        assert_eq!(p.index(), 2);
        assert_eq!(p.phase(), false);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 2);
        p.advance(2);
        assert_eq!(p.index(), 0);
        assert_eq!(p.phase(), true);
        assert_eq!(BufferPointer::distance(BufferPointer::ZERO, p), 4);

    }
}