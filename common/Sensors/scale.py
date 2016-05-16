#code copied from stentsnyder.com/reading-a-dymo-usb-scale-using-python/
import usb.core
import usb.util
import math

class Scale:
	def __init__(self):
		self.VENDOR_ID = 0x0922
		self.PRODUCT_ID = 0x8003
		# ^ these IDs are unique to the product being used
		# if we switch products from the DYMO M!0, these may need to change
		# might want to change them to constants in the future


		# v find the USB device
		self.device = usb.core.find(idVendor = self.VENDOR_ID, idProduct = self.PRODUCT_ID)

		if self.device == None:
			raise IOError('Device not found')
		
		#use the initial configuration

		#first endpoint
		self.endpoint = self.device[0][(0,0)][0]
		#[device number][configuration][? idk]

	#read a data packet
	def readData(self, attempts):

		DATA_MODE_GRAMS = 2
		DATA_MODE_OUNCES = 11

		if(not attempts>0):
			attempts = 10
			#default attempts are 10

		data = None
		while data is None and attempts >0:

			print attempts

			try:
				data = self.device.read(self.endpoint.bEndpointAddress, self.endpoint.wMaxPacketSize)
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

		scaling_factor = math.py(10, scaling_factor) 


		raw_weight = data[4] + (256*data[5])

		if data[2] == DATA_MODE_OUNCES:
			ounces = raw_weight * scaling_factor
			weight = "%s oz" % ounces
		elif data[2] == DATA_MODE_GRAMS:
			grams = raw_weight
			weight = "%s g" % grams

		return weight

		# for our device:
		# index 0 is always 3...
		# index 1 indicates stability of measured value - might actually indicate +/-  (2/5)
		# index 2 is 2 or 11 for metric/english respectively
		# index 3 is scaling in ounces
		# index 4 is lower bits of weight
		# index 5 is upper bits of weight



