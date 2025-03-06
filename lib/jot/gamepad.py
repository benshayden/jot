import time
from adafruit_hid import find_device

REPORT_DESCRIPTOR = bytes((
  0x05, 0x01,  # Usage Page (Generic Desktop Ctrls)
  0x09, 0x05,  # Usage (Game Pad)
  0xA1, 0x01,  # Collection (Application)
  0x85, 0x04,  #   Report ID (4)
  0x05, 0x09,  #   Usage Page (Button)
  0x19, 0x01,  #   Usage Minimum (Button 1)
  0x29, 0x18,  #   Usage Maximum (Button 24)
  0x15, 0x00,  #   Logical Minimum (0)
  0x25, 0x01,  #   Logical Maximum (1)
  0x75, 0x01,  #   Report Size (1)
  0x95, 0x18,  #   Report Count (24)
  0x81, 0x02,  #   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x05, 0x01,  #   Usage Page (Generic Desktop Ctrls)
  0x15, 0x81,  #   Logical Minimum (-127)
  0x25, 0x7F,  #   Logical Maximum (127)
  0x09, 0x30,  #   Usage (X)
  0x09, 0x31,  #   Usage (Y)
  0x09, 0x32,  #   Usage (Z)
  0x09, 0x35,  #   Usage (Rz)
  0x75, 0x08,  #   Report Size (8)
  0x95, 0x04,  #   Report Count (4)
  0x81, 0x02,  #   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,        # End Collection
))

class Gamepad:
	def __init__(self, devices, joystick0, joystick1, name='gamepad', ms=50):
		self._device = find_device(devices, usage_page=1, usage=5)
		self._report = bytearray(7)
		self.joystick0 = joystick0
		self.joystick1 = joystick1
		self.ms = ms
	
	def wait(self, attempts=100, seconds=0.01):
		for attempt in range(attempts):
			try:
				self._send()
				break
			except OSError:
				time.sleep(seconds)
	
	def press(self, code):
		self._report[int(code / 8)] |= 1 << (code % 8)
		self._send()
	
	def release(self, code):
		self._report[int(code / 8)] &= 0xff ^ (1 << (code % 8))
		self._send()
	
	def loop(self):
		prevjoys = tuple(self._report[3:7])
		nextjoys = (
			self._round(self.joystick0.x),
			self._round(self.joystick0.y),
			self._round(self.joystick1.x),
			self._round(self.joystick1.y),
		)
		if nextjoys != prevjoys:
			self._report[3:7] = nextjoys
			self._send()
	
	def _round(self, n):
		return min(127, max(-127, round(127.0 * n / self.samples))) & 0xff
	
	def _send(self):
		self._device.send_report(self._report)
