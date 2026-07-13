// Copyright 2021 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#![no_std]

// Transitional compatibility: the boot2 block is the board's responsibility
// (it depends on the flash chip), so boards should provide their own
// BOOT2_FIRMWARE static. These features keep the old library-provided
// behavior available during migration.
#[cfg(feature = "boot2-ram-memcpy")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;

#[cfg(feature = "boot2-w25q080")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub mod bitbang;
pub mod bridge;
pub mod line_coding;
#[cfg(not(feature = "bitbang"))]
pub mod pio;
pub mod util;

/// Releases all SIO spinlocks. `#[rp2040_hal::entry]` does this on startup,
/// but applications built on `#[rtic::app]` bypass that entry point, so they
/// must call this from a `#[pre_init]` handler instead.
///
/// # Safety
/// Must only be called before interrupts are enabled and before any spinlock
/// is in use (i.e. from `#[pre_init]`).
pub unsafe fn clear_spinlocks() {
    const SIO_BASE: u32 = 0xd0000000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    const SPINLOCK_COUNT: usize = 32;
    for i in 0..SPINLOCK_COUNT {
        SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
    }
}
