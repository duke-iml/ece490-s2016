import usb

dev = usb.core.find(idVendor=0x0922, idProduct=0x8003, find_all = False)


reattach = False
if dev.is_kernel_driver_active(0):
    reattach = True
    dev.detach_kernel_driver(0)

dev.set_configuration()
cfg = dev.get_active_configuration()

interface_number = cfg[(0,0)].bInterfaceNumber
alternate_settting = usb.control.get_interface(dev, interface_number)
intf = usb.util.find_descriptor(cfg, bInterfaceNumber = interface_number, bAlternateSetting = alternate_settting)

# ep = usb.util.find_descriptor(intf,custom_match = lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
# ep.write("test\n\r")

# This is needed to release interface, otherwise attach_kernel_driver fails
# due to "Resource busy"
usb.util.dispose_resources(dev)

# It may raise USBError if there's e.g. no kernel driver loaded at all
if reattach:
    dev.attach_kernel_driver(0)
