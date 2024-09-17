import smbus2 as smbus

class I2C_ID():

	def __init__(self):

		self.start_addr = 0x60
		self.end_addr = 0x77
	
	def getId(self, start_bus_number):
		bus_number = start_bus_number
		
		for bus_number in range(1, 3):
			bus = smbus.SMBus(bus_number)
			for addr in range(self.start_addr, self.end_addr + 1):
				try:
					bus.read_byte(addr)
					if(hex(addr) == '0x68'):
						bus_connected = bus_number
				except IOError:
					pass
		return bus_connected

