import usb.core
import usb.util
import math
import sys
import cPickle as pickle
import time, os

import re
import time
from serial import Serial
from serial import SerialException
import cPickle as pickle
import random
import serial.tools.list_ports


class Scale_Measurement:
    def __init__(self):
        self.VENDOR_ID = 0x0922
        self.PRODUCT_ID = 0x8003
        # ^ these IDs are unique to the product being used
        # if we switch products from the DYMO M!0, these may need to change
        # might want to change them to constants in the future


        self.devices = []


        # v find the USB device
        self.deviceList = usb.core.find(idVendor = self.VENDOR_ID, idProduct = self.PRODUCT_ID, find_all = True)


        for cfg in self.deviceList:
            sys.stdout.write('Decimal VendorID=' + str(cfg.idVendor) + ' & ProductID=' + str(cfg.idProduct) + '\n')
            sys.stdout.write('Hexadecimal VendorID=' + hex(cfg.idVendor) + ' & ProductID=' + hex(cfg.idProduct) + '\n\n')
            self.devices.append(cfg)



        if self.devices == []:
            raise IOError('No device found')

        print ('Connected ', len(self.devices), ' scales')


        for cfg in self.devices:
            if cfg.is_kernel_driver_active(0):
                try:
                    cfg.detach_kernel_driver(0)
                    print "kernel driver detached"
                except usb.core.USBError:
                    raise IOError("Could not detach kernel driver")

            #use the initial configuration





        #first endpoint
        self.endPointList = []

        for cfg in self.devices:
            self.endPointList.append(cfg[0][(0,0)][0])

        #[device number][configuration][? idk]

    #read a data packet
    def readData(self, attempts):

        DATA_MODE_GRAMS = 2
        DATA_MODE_OUNCES = 11

        if(not attempts>0):
            attempts = 10
            #default attempts are 10

        totalWeight = []

        for index in range(len(self.devices)):
            data = None
            weight = None
            while data is None and attempts >0:

                try:
                    data = self.devices[index].read(self.endPointList[index].bEndpointAddress, self.endPointList[index].wMaxPacketSize)
                except usb.core.USBError as e:
                    print e
                    print data
                    data = None
                    if e.args == ('Operation timed out',):
                        attempts -=1
                        continue

            # ^ reads from usb until it gets data or uses max attempts
            #print data

            if data is None:
                raise IOError('Received no data')

            scaling_factor = data[3]
            if scaling_factor > 128:
                scaling_factor = scaling_factor - 256

            scaling_factor = math.pow(10, scaling_factor)


            raw_weight = data[4] + (256*data[5])

            if data[2] == DATA_MODE_OUNCES:
                ounces = raw_weight * scaling_factor
                weight = "%s" % ounces
            elif data[2] == DATA_MODE_GRAMS:
                grams = raw_weight
                weight = "%s" % grams

            totalWeight.append(weight)

        return totalWeight

if __name__ == "__main__":
    sm = Scale_Measurement()
    while True:

        with open("weight.pkl", "wb") as f:
            pickle.dump(sm.readData(10), f)
            # pickle.dump("1", f)
        time.sleep(1)
