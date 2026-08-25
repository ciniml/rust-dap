// Copyright 2026 Hidekazu Kobayashi
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

//! Board-unique serial number, compatible with raspberrypi/debugprobe
//! (pico-sdk `pico_get_unique_board_id_string`): the 64-bit unique board ID
//! formatted as 16 uppercase hex digits. On RP2040 the ID is the QSPI flash
//! unique ID (command 0x4B); on RP2350 it is the chip id from OTP.

use core::sync::atomic::{AtomicBool, Ordering};

const ID_BYTES: usize = 8;

static INITIALIZED: AtomicBool = AtomicBool::new(false);
static mut SERIAL: [u8; ID_BYTES * 2] = [0; ID_BYTES * 2];

/// Returns the board-unique serial number string for the USB serial number
/// descriptor and `DAP_Info`.
///
/// The ID is read once on the first call and cached. On RP2040 the read
/// issues QSPI commands with XIP disabled, so the first call must happen
/// during init: before core1 is started and before any DMA from flash.
pub fn serial_number() -> &'static str {
    if !INITIALIZED.load(Ordering::Acquire) {
        let id = read_unique_id();
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(SERIAL) };
        for (i, b) in id.iter().enumerate() {
            buf[i * 2] = HEX[(b >> 4) as usize];
            buf[i * 2 + 1] = HEX[(b & 0xf) as usize];
        }
        INITIALIZED.store(true, Ordering::Release);
    }
    unsafe { core::str::from_utf8_unchecked(&*core::ptr::addr_of!(SERIAL)) }
}

/// Host builds (e.g. `cargo clippy` on the workspace) cannot assemble the
/// ARM flash-read code; return the pico-sdk-style dummy ID instead.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn read_unique_id() -> [u8; ID_BYTES] {
    [0xEE; ID_BYTES]
}

#[cfg(all(feature = "rp2040", target_arch = "arm", target_os = "none"))]
fn read_unique_id() -> [u8; ID_BYTES] {
    let mut id = [0u8; ID_BYTES];
    // Restore XIP by re-running the boot2 blob, like pico-sdk's flash_do_cmd,
    // so flash keeps running at full speed afterwards. With `ram-exec` the
    // program runs from SRAM: XIP speed does not matter and re-running
    // BOOT_LOADER_RAM_MEMCPY would copy the image over itself, so use the
    // generic ROM routine instead.
    let use_boot2 = !cfg!(feature = "ram-exec");
    cortex_m::interrupt::free(|_| unsafe { flash::flash_unique_id(&mut id, use_boot2) });
    id
}

#[cfg(all(feature = "rp2350", target_arch = "arm", target_os = "none"))]
fn read_unique_id() -> [u8; ID_BYTES] {
    use crate::hal::rom_data::sys_info_api::chip_info;
    // Same 8 bytes as pico-sdk's pico_unique_id on RP2350: the OTP chip id,
    // wafer id then device id, big-endian.
    match chip_info() {
        Ok(Some(info)) => {
            let mut id = [0u8; ID_BYTES];
            id[..4].copy_from_slice(&info.wafer_id.to_be_bytes());
            id[4..].copy_from_slice(&info.device_id.to_be_bytes());
            id
        }
        // Same well-defined-but-obviously-wrong fallback as pico-sdk.
        _ => [0xEE; ID_BYTES],
    }
}

/// QSPI flash "Read Unique ID" support, vendored from the rp2040-flash crate
/// v0.6.0 (https://github.com/jannic/rp2040-flash, MIT OR Apache-2.0) with
/// the erase/program paths dropped, to avoid pulling a second rp2040-hal
/// version into the tree.
#[cfg(all(feature = "rp2040", target_arch = "arm", target_os = "none"))]
mod flash {
    use crate::hal::rom_data;
    use core::marker::PhantomData;

    #[repr(C)]
    struct FlashFunctionPointers<'a> {
        connect_internal_flash: unsafe extern "C" fn() -> (),
        flash_exit_xip: unsafe extern "C" fn() -> (),
        flash_range_erase: Option<
            unsafe extern "C" fn(addr: u32, count: usize, block_size: u32, block_cmd: u8) -> (),
        >,
        flash_range_program:
            Option<unsafe extern "C" fn(addr: u32, data: *const u8, count: usize) -> ()>,
        flash_flush_cache: unsafe extern "C" fn() -> (),
        flash_enter_cmd_xip: unsafe extern "C" fn() -> (),
        phantom: PhantomData<&'a ()>,
    }

    fn flash_function_pointers() -> FlashFunctionPointers<'static> {
        FlashFunctionPointers {
            connect_internal_flash: rom_data::connect_internal_flash::ptr(),
            flash_exit_xip: rom_data::flash_exit_xip::ptr(),
            flash_range_erase: None,
            flash_range_program: None,
            flash_flush_cache: rom_data::flash_flush_cache::ptr(),
            flash_enter_cmd_xip: rom_data::flash_enter_cmd_xip::ptr(),
            phantom: PhantomData,
        }
    }

    /// # Safety
    ///
    /// `boot2` must contain a valid 2nd stage boot loader which can be called to re-initialize XIP mode
    unsafe fn flash_function_pointers_with_boot2(boot2: &[u32; 64]) -> FlashFunctionPointers<'_> {
        let boot2_fn_ptr = (boot2 as *const u32 as *const u8).offset(1);
        let boot2_fn: unsafe extern "C" fn() -> () = core::mem::transmute(boot2_fn_ptr);
        FlashFunctionPointers {
            connect_internal_flash: rom_data::connect_internal_flash::ptr(),
            flash_exit_xip: rom_data::flash_exit_xip::ptr(),
            flash_range_erase: None,
            flash_range_program: None,
            flash_flush_cache: rom_data::flash_flush_cache::ptr(),
            flash_enter_cmd_xip: boot2_fn,
            phantom: PhantomData,
        }
    }

    #[repr(C)]
    struct FlashCommand {
        cmd_addr: *const u8,
        cmd_addr_len: u32,
        dummy_len: u32,
        data: *mut u8,
        data_len: u32,
    }

    /// Return SPI flash unique ID
    ///
    /// Not all SPI flashes implement this command, so check the JEDEC
    /// ID before relying on it. The Winbond parts commonly seen on
    /// RP2040 devboards (JEDEC=0xEF7015) support an 8-byte unique ID.
    ///
    /// # Safety
    ///
    /// Nothing must access flash while this is running.
    /// Usually this means:
    ///   - interrupts must be disabled
    ///   - 2nd core must be running code from RAM or ROM with interrupts disabled
    ///   - DMA must not access flash memory
    pub unsafe fn flash_unique_id(out: &mut [u8], use_boot2: bool) {
        let mut boot2 = [0u32; 256 / 4];
        let ptrs = if use_boot2 {
            rom_data::memcpy44(&mut boot2 as *mut _, 0x10000000 as *const _, 256);
            flash_function_pointers_with_boot2(&boot2)
        } else {
            flash_function_pointers()
        };
        // 4B - read unique ID
        let cmd = [0x4B];
        read_flash(&cmd[..], 4, out, &ptrs as *const FlashFunctionPointers);
    }

    unsafe fn read_flash(
        cmd_addr: &[u8],
        dummy_len: u32,
        out: &mut [u8],
        ptrs: *const FlashFunctionPointers,
    ) {
        read_flash_inner(
            FlashCommand {
                cmd_addr: cmd_addr.as_ptr(),
                cmd_addr_len: cmd_addr.len() as u32,
                dummy_len,
                data: out.as_mut_ptr(),
                data_len: out.len() as u32,
            },
            ptrs,
        );
    }

    /// Issue a generic SPI flash read command
    ///
    /// # Arguments
    ///
    /// * `cmd` - `FlashCommand` structure
    /// * `ptrs` - Flash function pointers
    #[inline(never)]
    #[link_section = ".data.ram_func"]
    unsafe fn read_flash_inner(cmd: FlashCommand, ptrs: *const FlashFunctionPointers) {
        core::arch::asm!(
            // r6, r7 are LLVM-reserved and can't be marked as a clobber, so save/restore them manually
            // (r6 is not actually used, but we need to push two words to maintain stack alignment)
            "push {{r6, r7}}",

            "mov r7, r0", // cmd
            "mov r5, r1", // ptrs

            "ldr r4, [r5, #0]",
            "blx r4", // connect_internal_flash()

            "ldr r4, [r5, #4]",
            "blx r4", // flash_exit_xip()

            "movs r4, #0x18",
            "lsls r4, r4, #24", // 0x18000000, SSI, RP2040 datasheet 4.10.13

            // Disable, write 0 to SSIENR
            "movs r0, #0",
            "str r0, [r4, #8]", // SSIENR

            // Write ctrlr0
            "movs r0, #0x3",
            "lsls r0, r0, #8", // TMOD=0x300
            "ldr r1, [r4, #0]", // CTRLR0
            "orrs r1, r0",
            "str r1, [r4, #0]",

            // Write ctrlr1 with len-1
            "ldr r0, [r7, #8]", // dummy_len
            "ldr r1, [r7, #16]", // data_len
            "add r0, r1",
            "subs r0, #1",
            "str r0, [r4, #0x04]", // CTRLR1

            // Enable, write 1 to ssienr
            "movs r0, #1",
            "str r0, [r4, #8]", // SSIENR

            // Write cmd/addr phase to DR
            "mov r2, r4",
            "adds r2, 0x60", // &DR
            "ldr r0, [r7, #0]", // cmd_addr
            "ldr r1, [r7, #4]", // cmd_addr_len
            "10:",
            "ldrb r3, [r0]",
            "strb r3, [r2]", // DR
            "adds r0, #1",
            "subs r1, #1",
            "bne 10b",

            // Skip any dummy cycles
            "ldr r1, [r7, #8]", // dummy_len
            "cmp r1, #0",
            "beq 9f",
            "4:",
            "ldr r3, [r4, #0x28]", // SR
            "movs r2, #0x8",
            "tst r3, r2", // SR.RFNE
            "beq 4b",

            "mov r2, r4",
            "adds r2, 0x60", // &DR
            "ldrb r3, [r2]", // DR
            "subs r1, #1",
            "bne 4b",

            // Read RX fifo
            "9:",
            "ldr r0, [r7, #12]", // data
            "ldr r1, [r7, #16]", // data_len

            "2:",
            "ldr r3, [r4, #0x28]", // SR
            "movs r2, #0x8",
            "tst r3, r2", // SR.RFNE
            "beq 2b",

            "mov r2, r4",
            "adds r2, 0x60", // &DR
            "ldrb r3, [r2]", // DR
            "strb r3, [r0]",
            "adds r0, #1",
            "subs r1, #1",
            "bne 2b",

            // Disable, write 0 to ssienr
            "movs r0, #0",
            "str r0, [r4, #8]", // SSIENR

            // Write 0 to CTRLR1 (returning to its default value)
            //
            // flash_enter_cmd_xip does NOT do this, and everything goes
            // wrong unless we do it here
            "str r0, [r4, #4]", // CTRLR1

            "ldr r4, [r5, #20]",
            "blx r4", // flash_enter_cmd_xip();

            "pop {{r6, r7}}",

            in("r0") &cmd as *const FlashCommand,
            in("r1") ptrs,
            out("r2") _,
            out("r3") _,
            out("r4") _,
            out("r5") _,
            clobber_abi("C"),
        );
    }
}
