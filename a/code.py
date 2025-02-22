import board
import gc
import keypad
import math
import neopixel
import os
import supervisor
import sys
import time
import usb_hid

from adafruit_hid import find_device
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode
from adafruit_hid.mouse import Mouse
from analogio import AnalogIn

class ConsumerControlWrapper(ConsumerControl):
	def release(self, *unused):
		super().release()

def por(device, event, code):
	# press or release
	if event.pressed:
		device.press(code)
	else:
		device.release(code)

LAYER_CACHE = {}
def get_layer(name):
	if name in LAYER_CACHE:
		return LAYER_CACHE[name]
	filename = '/layers/' + name + '.txt'
	try:
		LAYER_CACHE[name] = open(filename).read().split('\n')
		return LAYER_CACHE[name]
	except Exception as e:
		print('exception reading ' + filename)
		print(e)

KEYMAP = []
def set_layer(name):
	KEYMAP[:] = get_layer(name)

SCRIPT_CACHE = {}
def run(name):
	if not name:
		return
	if name not in SCRIPT_CACHE:
		filename = '/scripts/' + name + '.py'
		try:
			SCRIPT_CACHE[name] = compile(open(filename).read(), filename, 'exec')
		except Exception as e:
			print('exception loading ' + filename)
			print(e)
	if name not in SCRIPT_CACHE:
		return
	try:
		exec(SCRIPT_CACHE[name], globals(), globals())
	except Exception as e:
		print(e)


SERIAL_BLOCK_CALLBACK = None
SERIAL_BLOCK = ''
def read_block(callback):
	global SERIAL_BLOCK_CALLBACK
	SERIAL_BLOCK_CALLBACK = callback

_TICKS_PERIOD = const(1<<29)
_TICKS_MAX = const(_TICKS_PERIOD-1)
_TICKS_HALFPERIOD = const(_TICKS_PERIOD//2)

def ticks_add(ticks, delta):
    "Add a delta to a base number of ticks, performing wraparound at 2**29ms."
    return (ticks + delta) % _TICKS_PERIOD

def ticks_diff(ticks1, ticks2):
    "Compute the signed difference between two ticks values, assuming that they are within 2**28 ticks"
    diff = (ticks1 - ticks2) & _TICKS_MAX
    diff = ((diff + _TICKS_HALFPERIOD) & _TICKS_MAX) - _TICKS_HALFPERIOD
    return diff


EVERY_PREV_MS = {}
def everyms(ms, usage):
	now = supervisor.ticks_ms()
	if usage not in EVERY_PREV_MS or ticks_diff(now, EVERY_PREV_MS[usage]) >= ms:
		EVERY_PREV_MS[usage] = now
		return True
	return False


class JoyStick:
	scale = float(1 << 15) # half of the max value

	def __init__(self, xpin, ypin, ms=10):
		self.xin = xpin
		self.yin = ypin
		self.ms = ms
		self.deadzone = 0.01
		self.speed = 0.1
		self.xcenter = self.scale
		self.ycenter = self.scale

		# temporary state
		self.start = self.now()
		self.xbuf = 0.0
		self.ybuf = 0.0
		self.samples = 0
		self.prevsample = self.start - 1
		self.prevdebug = 0

		self.debug = False # set to true to enable regularly printing debug info
		self.debugms = 1000 # period of debug printing

	def calibrate(self, samples=100):
		self.xcenter = 0
		self.ycenter = 0
		for _ in range(samples):
			self.xcenter += float(self.xin.value) / samples
			self.ycenter += float(self.yin.value) / samples
			time.sleep(0.001)
		self.deadzone = 0
		for _ in range(samples):
			x = float(self.xin.value - self.xcenter) / self.scale
			y = float(self.yin.value - self.ycenter) / self.scale
			self.deadzone = max(self.deadzone, math.sqrt(x * x + y * y))
			time.sleep(0.001)
		if self.debug:
			print('xcenter ' + str(self.xcenter) + ' ycenter ' + str(self.ycenter) + ' deadzone ' + str(self.deadzone))

	def now(self):
		return supervisor.ticks_ms()

	def loop(self):
		now = self.now()
		if now == self.prevsample:
			return
		self.prevsample = now
		xnew = self.xin.value
		ynew = self.yin.value
		# scale from [0,2*self.scale] to [-1.0,1.0]
		self.xbuf += (xnew - self.xcenter) / self.scale
		self.ybuf += (ynew - self.ycenter) / self.scale
		self.samples += 1

		if ticks_diff(now, self.start) >= self.ms:
			# self.samples should equal self.ms but might be less if the main loop was stuck on something big for over 1ms.
			self.start = now
			if (math.sqrt(self.xbuf * self.xbuf + self.ybuf * self.ybuf) / self.samples) < self.deadzone:
				self.xbuf = self.ybuf = 0
			x, newxbuf = self.clean(self.xbuf)
			y, newybuf = self.clean(self.ybuf)

			if self.debug and ticks_diff(now, self.prevdebug) >= self.debugms:
				print('joystick ' + str(self.samples) + ' ' + str(self.xbuf) + ' ' + str(self.ybuf) + ' ' + str(x) + ' ' + str(y))
				self.prevdebug = now

			self.xbuf = newxbuf
			self.ybuf = newybuf
			self.samples = 0
			return (x, y)

	def clean(self, n):
		if n < 0:
			n = min(0, n + self.deadzone)
		else:
			n = max(0, n - self.deadzone)
		n /= (1.0 - self.deadzone)
		# instead of rounding, truncate and carry the fractional part over to the next move().
		whole, frac = divmod(self.speed * n * 127.0 / self.samples, 1.0)
		return max(-127, min(127, int(whole))), frac * self.samples / 127.0
	

class JoyMouse:
	def __init__(self, joystick):
		self._joystick = joystick

	def loop(self):
		xy = self._joystick.loop()
		if xy:
			mouse.move(*xy)


class Gamepad:
	def __init__(self, devices, joystick0, joystick1):
		self._device = find_device(devices, usage_page=1, usage=5)
		self._report = bytearray(6)
		self._joystick0 = joystick0
		self._joystick1 = joystick1
		self._wait()

	def _wait(self):
		for attempt in range(50):
			try:
				self._send()
				break
			except OSError:
				time.sleep(0.01)

	def press(self, code):
		self._report[int(code / 8)] |= 1 << (code % 8)
		self._send()

	def release(self, code):
		self._report[int(code / 8)] &= 0xff ^ (1 << (code % 8))
		self._send()

	def loop(self):
		j0 = self._joystick0.loop()
		j1 = self._joystick1.loop()
		if j0:
			self._report[2] = j0[0] & 0xff
			self._report[3] = j0[1] & 0xff
		if j1:
			self._report[4] = j1[0] & 0xff
			self._report[5] = j1[1] & 0xff
		if j0 or j1:
			self._send()

	def _send(self):
		self._device.send_report(self._report)

LOOPS = {}
def addloop(name, script):
	LOOPS[name] = (lambda _=compile(script, name, 'exec'): exec(_, globals, globals)) if isinstance(script, str) else script

def removeloop(name):
	del LOOPS[name]

def runloops():
	bad = []
	for name in LOOPS:
		try:
			LOOPS[name]()
		except Exception as e:
			print(e)
			bad.append(name)
	for name in bad:
		del LOOPS[name]

addloop('loop', lambda: run('loop'))

neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

A0 = AnalogIn(board.A0)
A1 = AnalogIn(board.A1)
A2 = AnalogIn(board.A2)
A3 = AnalogIn(board.A3)
JOYSTICK0 = JoyStick(A0, A1)
JOYSTICK1 = JoyStick(A2, A3)

KEYMATRIX = keypad.KeyMatrix(
  columns_to_anodes=True,
	column_pins=(board.D5, board.D6, board.D7, board.D8, board.D9, board.D10),
	row_pins=(board.D2, board.D3, board.D4),
	debounce_threshold=2,
)
print('key_count=' + str(KEYMATRIX.key_count))
pressed_scripts = set()

for d in usb_hid.devices:
	print('device ' + str(d.usage_page) + ' ' + str(d.usage))

keyboard = Keyboard(usb_hid.devices)
consumer = ConsumerControlWrapper(usb_hid.devices)
mouse = Mouse(usb_hid.devices)
gamepad = Gamepad(usb_hid.devices, JOYSTICK0, JOYSTICK1)
joymouse = JoyMouse(JOYSTICK0)

set_layer('default')
run('setup')

while True:
	event = KEYMATRIX.events.get()
	if event and 0 <= event.key_number < len(KEYMAP):
		if event.pressed:
			pressed_scripts.add(KEYMAP[event.key_number])
		elif KEYMAP[event.key_number] in pressed_scripts:
			pressed_scripts.remove(KEYMAP[event.key_number])
		run(KEYMAP[event.key_number])
	# If a command is entered on Serial, exec it.
	# https://webserial.io/
	if supervisor.runtime.serial_bytes_available:
		serial_bytes = sys.stdin.read(supervisor.runtime.serial_bytes_available)
		if SERIAL_BLOCK_CALLBACK:
			SERIAL_BLOCK += serial_bytes
			if SERIAL_BLOCK.endswith('\n\n') or SERIAL_BLOCK.endswith('\r\n\r\n'):
				try:
					SERIAL_BLOCK_CALLBACK(SERIAL_BLOCK)
				except Exception as e:
					print(e)
				SERIAL_BLOCK = ''
				SERIAL_BLOCK_CALLBACK = None
		else:
			# todo buffer, split lines
			ARGS = serial_bytes.strip().split()
			if ARGS:
				run(ARGS[0])
	runloops()
