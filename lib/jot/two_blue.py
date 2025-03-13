from . import tasks
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement

# If all services are advertised in a single large advertisement, it's too big for CircuitPython clients to connect to, so split them into two advertisements:
# one for services for clients that pair with this device such as laptops, and one for services for other clients such as the other half of a keyboard.
class TwoBlue:
	def __init__(self, ble, led=None, *, unpaired_services, unpaired_appearance, paired_services, paired_appearance):
		self.ble = ble
		ble.stop_advertising()
		self.unpaired_advertisement = ProvideServicesAdvertisement(*unpaired_services)
		self.unpaired_advertisement.appearance = unpaired_appearance
		self.unpaired_advertisement.complete_name = ble.name
		self.paired_advertisement = ProvideServicesAdvertisement(*paired_services)
		self.paired_advertisement.appearance = paired_appearance
		self.paired_advertisement.complete_name = ble.name
		self.led = led
		self._led_tick = 0
		self._advertising = None
		self.task = tasks.create()(self.loop)

	def _flash(self, ms, now):
		if not self.led:
			return
		if tasks.ticks_diff(now, self._led_tick) < ms:
			return
		self._led_tick = now
		self.led.value = not self.led.value
	
	def loop(self, now):
		num_paired = num_unpaired = 0
		for c in self.ble.connections:
			if c.paired:
				num_paired += 1
			else:
				num_unpaired += 1
		if num_paired > 1:
			for c in self.ble.connections:
				if c.paired:
					c.disconnect()
			return # wait for the next task loop
		elif num_unpaired > 1:
			for c in self.ble.connections:
				if not c.paired:
					c.disconnect()
			return # wait for the next task loop
		elif num_unpaired < 1:
			if self._advertising != self.unpaired_advertisement:
				self.ble.stop_advertising()
				self.ble.start_advertising(self.unpaired_advertisement)
				self._advertising = self.unpaired_advertisement
				print('advertising unpaired services')
			self._flash(250, now)
		elif num_paired < 1:
			if self._advertising != self.paired_advertisement:
				self.ble.stop_advertising()
				self.ble.start_advertising(self.paired_advertisement)
				self._advertising = self.paired_advertisement
				print('advertising paired services')
			self._flash(500, now)
		elif self._advertising != None:
			# done!
			self.ble.stop_advertising()
			self._advertising = None
			if self.led:
				self.led.value = True
			print('stopped advertising')
