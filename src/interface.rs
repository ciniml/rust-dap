use usb_device::class_prelude::*;
use usb_device::Result;

pub const USB_CLASS_VENDOR: u8 = 0xff;
pub const USB_SUBCLASS_VENDOR: u8 = 0x00;
pub const USB_PROTOCOL_NONE: u8 = 0x00;

pub struct CmsisDapInterface<'a, B: UsbBus> {
    interface: InterfaceNumber,
    out_ep: EndpointOut<'a, B>,
    in_ep: EndpointIn<'a, B>,
}

impl<B: UsbBus> CmsisDapInterface<'_, B> {
    pub fn new(alloc: &UsbBusAllocator<B>, max_packet_size: u16) -> CmsisDapInterface<'_, B> {
        CmsisDapInterface {
            interface: alloc.interface(),
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

impl<B: UsbBus> UsbClass<B> for CmsisDapInterface <'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(
            self.interface,
            USB_CLASS_VENDOR,
            USB_SUBCLASS_VENDOR,
            USB_PROTOCOL_NONE
        )?;
        writer.endpoint(&self.out_ep)?;
        writer.endpoint(&self.in_ep)?;
        

        Ok(())
    }

    fn reset(&mut self) {
        
    }

    fn control_in(&mut self, _xfer: ControlIn<B>) {
    }

    fn control_out(&mut self, _xfer: ControlOut<B>) {
    }


}