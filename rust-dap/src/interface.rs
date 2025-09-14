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

use num_enum::{IntoPrimitive, TryFromPrimitive};
use usb_device::class_prelude::*;
use usb_device::control::RequestType;
use usb_device::device::DEFAULT_ALTERNATE_SETTING;
use usb_device::Result;

const USB_IF_CLASS_VENDOR: u8 = 0xff;
const USB_IF_SUBCLASS_VENDOR: u8 = 0x00;
const USB_IF_PROTOCOL_NONE: u8 = 0x00;

pub const USB_CLASS_MISCELLANEOUS: u8 = 0xef;
pub const USB_SUBCLASS_COMMON: u8 = 0x02;
pub const USB_PROTOCOL_IAD: u8 = 0x01;

const BOS_CAPABILITY_TYPE_PLATFORM: u8 = 0x05;
const MS_OS_20_SET_HEADER_DESCRIPTOR: u16 = 0x0000;
const MS_OS_20_SUBSET_HEADER_CONFIGURATION: u16 = 0x0001;
const MS_OS_20_SUBSET_HEADER_FUNCTION: u16 = 0x0002;
const MS_OS_20_FEATURE_COMPATIBLE_ID: u16 = 0x0003;
const MS_OS_20_FEATURE_REG_PROPERTY: u16 = 0x0004;

#[repr(u16)]
#[derive(IntoPrimitive, TryFromPrimitive)]
enum RegPropertyType {
    Reserved,
    String,
    ExpandString,
    Binary,
    DwordLittleEndian,
    DwordBigEndian,
    Link,
    MultiString,
}

const MS_VENDOR_CODE: u8 = 0x01;

fn fill_utf16<const N: usize>(buf: &mut [u8], b: &[u8; N]) {
    for i in 0..N {
        buf[i * 2] = b[i];
        buf[i * 2 + 1] = 0;
    }
}

fn write_descriptor_set(
    buffer: &mut [u8],
    windows_version: u32,
    f: impl FnOnce(&mut [u8]) -> Result<usize>,
) -> Result<usize> {
    let length = 10usize;
    if buffer.len() < length {
        return Err(UsbError::BufferOverflow);
    }
    buffer[0] = u16_lo(length as u16);
    buffer[1] = u16_hi(length as u16);
    buffer[2] = u16_lo(MS_OS_20_SET_HEADER_DESCRIPTOR);
    buffer[3] = u16_hi(MS_OS_20_SET_HEADER_DESCRIPTOR);
    buffer[4] = u16_lo(u32_lo(windows_version));
    buffer[5] = u16_hi(u32_lo(windows_version));
    buffer[6] = u16_lo(u32_hi(windows_version));
    buffer[7] = u16_hi(u32_hi(windows_version));
    let total_length = f(&mut buffer[length..])? + length;
    buffer[8] = u16_lo(total_length as u16);
    buffer[9] = u16_hi(total_length as u16);
    Ok(total_length)
}

fn write_configuration_subset(
    buffer: &mut [u8],
    f: impl FnOnce(&mut [u8]) -> Result<usize>,
) -> Result<usize> {
    let length = 8usize;
    if buffer.len() < length {
        return Err(UsbError::BufferOverflow);
    }
    buffer[0] = u16_lo(length as u16);
    buffer[1] = u16_hi(length as u16);
    buffer[2] = u16_lo(MS_OS_20_SUBSET_HEADER_CONFIGURATION);
    buffer[3] = u16_hi(MS_OS_20_SUBSET_HEADER_CONFIGURATION);
    buffer[4] = 0; // Currently usb_device supports one configuration.
    buffer[5] = 0; // reserved
    let total_length = f(&mut buffer[length..])? + length;
    buffer[6] = u16_lo(total_length as u16);
    buffer[7] = u16_hi(total_length as u16);
    Ok(total_length)
}

fn write_function_subset(
    buffer: &mut [u8],
    first_interface_number: InterfaceNumber,
    f: impl FnOnce(&mut [u8]) -> Result<usize>,
) -> Result<usize> {
    let length = 8usize;
    if buffer.len() < length {
        return Err(UsbError::BufferOverflow);
    }
    buffer[0] = u16_lo(length as u16);
    buffer[1] = u16_hi(length as u16);
    buffer[2] = u16_lo(MS_OS_20_SUBSET_HEADER_FUNCTION);
    buffer[3] = u16_hi(MS_OS_20_SUBSET_HEADER_FUNCTION);
    buffer[4] = first_interface_number.into();
    buffer[5] = 0; // reserved
    let total_length = f(&mut buffer[length..])? + length;
    buffer[6] = u16_lo(total_length as u16);
    buffer[7] = u16_hi(total_length as u16);
    Ok(total_length)
}

fn write_compatible_id(
    buffer: &mut [u8],
    compatible_id: &[u8; 8],
    sub_compatible_id: &[u8; 8],
) -> Result<usize> {
    let length = 20usize;
    if buffer.len() < length {
        return Err(UsbError::BufferOverflow);
    }
    buffer[0] = u16_lo(length as u16);
    buffer[1] = u16_hi(length as u16);
    buffer[2] = u16_lo(MS_OS_20_FEATURE_COMPATIBLE_ID);
    buffer[3] = u16_hi(MS_OS_20_FEATURE_COMPATIBLE_ID);
    buffer[4..12].copy_from_slice(compatible_id);
    buffer[12..20].copy_from_slice(sub_compatible_id);
    Ok(length)
}

fn write_registry_property(
    buffer: &mut [u8],
    property_name: &[u8],
    property_type: RegPropertyType,
    property_data: &[u8],
) -> Result<usize> {
    let name_len = property_name.len();
    let data_len = property_data.len();
    let length = name_len + data_len + 10;
    if buffer.len() < length {
        return Err(UsbError::BufferOverflow);
    }
    let property_type: u16 = property_type.into();
    buffer[0] = u16_lo(length as u16);
    buffer[1] = u16_hi(length as u16);
    buffer[2] = u16_lo(MS_OS_20_FEATURE_REG_PROPERTY);
    buffer[3] = u16_hi(MS_OS_20_FEATURE_REG_PROPERTY);
    buffer[4..6].copy_from_slice(&property_type.to_le_bytes());
    buffer[6..8].copy_from_slice(&(name_len as u16).to_le_bytes());
    buffer[8..8 + name_len].copy_from_slice(property_name);
    buffer[8 + name_len..8 + name_len + 2].copy_from_slice(&(data_len as u16).to_le_bytes());
    buffer[8 + name_len + 2..8 + name_len + 2 + data_len].copy_from_slice(property_data);

    Ok(length)
}

const fn u16_lo(v: u16) -> u8 {
    (v & 0xff) as u8
}
const fn u16_hi(v: u16) -> u8 {
    (v >> 8) as u8
}
const fn u32_lo(v: u32) -> u16 {
    (v & 0xfffff) as u16
}
const fn u32_hi(v: u32) -> u16 {
    (v >> 16) as u16
}

pub struct CmsisDapInterface<'a, B: UsbBus> {
    interface: InterfaceNumber,
    serial_string: StringIndex,
    out_ep: EndpointOut<'a, B>,
    in_ep: EndpointIn<'a, B>,
}

impl<B: UsbBus> CmsisDapInterface<'_, B> {
    pub fn new(alloc: &UsbBusAllocator<B>, max_packet_size: u16) -> CmsisDapInterface<'_, B> {
        CmsisDapInterface {
            interface: alloc.interface(),
            serial_string: alloc.string(),
            out_ep: alloc.bulk(max_packet_size),
            in_ep: alloc.bulk(max_packet_size),
        }
    }

    pub fn max_packet_size(&self) -> u16 {
        self.out_ep.max_packet_size()
    }

    pub fn write_packet(&mut self, data: &[u8]) -> Result<usize> {
        self.in_ep.write(data)
    }
    pub fn read_packet(&mut self, data: &mut [u8]) -> Result<usize> {
        self.out_ep.read(data)
    }

    pub(crate) fn in_ep_address(&self) -> EndpointAddress {
        self.in_ep.address()
    }
    pub(crate) fn out_ep_address(&self) -> EndpointAddress {
        self.out_ep.address()
    }
}

impl<B: UsbBus> UsbClass<B> for CmsisDapInterface<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        //writer.iad(self.interface, 1, 0xff, 0xff, 0x00)?;
        writer.interface_alt(
            self.interface,
            DEFAULT_ALTERNATE_SETTING,
            USB_IF_CLASS_VENDOR,
            USB_IF_SUBCLASS_VENDOR,
            USB_IF_PROTOCOL_NONE,
            Some(self.serial_string),
        )?;
        writer.endpoint(&self.out_ep)?;
        writer.endpoint(&self.in_ep)?;

        Ok(())
    }

    fn get_bos_descriptors(&self, writer: &mut BosWriter) -> Result<()> {
        #[rustfmt::skip]
        writer.capability(
            BOS_CAPABILITY_TYPE_PLATFORM,
            &[
                0,  // Reserved
                0xdf, 0x60, 0xdd, 0xd8,  // MS_OS_20_Platform_Capability_ID
                0x89, 0x45, 0xc7, 0x4c,  // {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
                0x9c, 0xd2, 0x65, 0x9d,  // 
                0x9e, 0x64, 0x8a, 0x9f,  //
                0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion – 0x06030000 (Win8.1 or later)
                174, 0,                  // wLength = MS OS 2.0 descriptor set
                MS_VENDOR_CODE,          // bMS_VendorCode
                0x00,                    // bAltEnumCmd - does not support alternate enum.
            ]
        )?;
        Ok(())
    }

    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        let _ = lang_id;
        if index == self.serial_string {
            Some("CMSIS-DAP interface")
        } else {
            None
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let request = xfer.request();
        if request.request_type == RequestType::Vendor
            && request.request == MS_VENDOR_CODE
            && request.value == 0
            && request.index == 7
        {
            // Request to retrieve MS OS 2.0 Descriptor Set.
            let interface_number = self.interface;
            xfer.accept(|buffer| {
                write_descriptor_set(buffer, 0x06030000, |buffer| {
                    write_configuration_subset(buffer, |buffer| {
                        write_function_subset(buffer, interface_number, |buffer| {
                            let mut offset = 0;
                            // Registry property name and value must be null-terminated UTF-16 string so we have to allocate double size of the original string and fill it with 0 padded on MSB.
                            let mut property_name = [0u8; b"DeviceInterfaceGUID\0".len() * 2];
                            fill_utf16(&mut property_name, b"DeviceInterfaceGUID\0");
                            let mut property_data =
                                [0u8; b"{A5DCBF10-6530-11D2-901F-00C04FB951ED}\0".len() * 2];
                            fill_utf16(
                                &mut property_data,
                                b"{A5DCBF10-6530-11D2-901F-00C04FB951ED}\0",
                            );

                            // Set Compatible ID to WINUSB in order to be WinUSB driver is loaded for this interface.
                            offset += write_compatible_id(
                                &mut buffer[offset..],
                                b"WINUSB\0\0",
                                &[0u8; 8],
                            )?;
                            // Set GUID_DEVINTERFACE_USB_DEVICE({A5DCBF10-6530-11D2-901F-00C04FB951ED}) Device Interface Class to be enumerated by libusb.
                            offset += write_registry_property(
                                &mut buffer[offset..],
                                &property_name,
                                RegPropertyType::String,
                                &property_data,
                            )?;
                            Ok(offset)
                        })
                    })
                })
            })
            .unwrap();
        }
    }
}
