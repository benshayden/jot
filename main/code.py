import board
import collections
import gc
import keypad
import math
import microcontroller
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

def por(device, code):
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
		LAYER_CACHE[name] = [tuple(line.split(' ')) for line in open(filename).read().split('\n')]
		return LAYER_CACHE[name]
	except Exception as e:
		print('exception reading ' + filename)
		print(e)

KEYMAP = []
LAYER_NAME = 'default'
def set_layer(name):
	global LAYER_NAME
	LAYER_NAME = name
	KEYMAP[:] = get_layer(name)

SCRIPT_CACHE = {}
def switch(ARGS, _dir='switches'):
	globals()['ARGS'] = ARGS
	if not ARGS or not ARGS[0]:
		return
	filename = '/' + _dir + '/' + ARGS[0] + '.py'
	if filename not in SCRIPT_CACHE:
		try:
			SCRIPT_CACHE[filename] = compile(open(filename).read(), filename, 'exec')
		except Exception as e:
			print('exception loading ' + filename)
			print(e)
	if filename not in SCRIPT_CACHE:
		return
	try:
		exec(SCRIPT_CACHE[filename], globals(), globals())
	except Exception as e:
		print(e)

def command(ARGS):
	switch(ARGS, _dir='commands')


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

Interval = collections.namedtuple('Interval', ('min', 'max'))

class JoyStick:
	def __init__(self, xpin, xneg, xpos, ypin, yneg, ypos):
		self.xin = xpin
		self.xneg = xneg
		self.xpos = xpos
		self.yin = ypin
		self.yneg = yneg
		self.ypos = ypos

	def _norm(self, val, neg, pos):
		if neg.min <= val <= neg.max:
			return (val - neg.max) / (neg.max - neg.min)
		elif pos.min <= val <= pos.max:
			return (val - pos.min) / (pos.max - pos.min)
		return 0.0
	
	def loop(self):
		# returns x,y where x and y are in [-1.0, 1.0]
		return self._norm(self.xin.value, self.xneg, self.xpos), self._norm(self.yin.value, self.yneg, self.ypos)
	

class JoyMouse:
	def __init__(self, joystick, ms=50, speed=0.1):
		self._joystick = joystick
		self.ms = ms
		self.speed = speed
		self._x = 0.0
		self._y = 0.0
		self._samples = 0
	
	def loop(self):
		if everyms(1, id(self) - 1):
			xnorm, ynorm = self._joystick.loop()
			self._x += xnorm
			self._y += ynorm
			self._samples += 1
			if everyms(self.ms, id(self)):
				dx, self._x = self._buf(self._x)
				dy, self._y = self._buf(self._y)
				self._samples = 0
				mouse.move(dx, dy)
	
	def _buf(self, n):
		whole, frac = divmod(self.speed * n * 127.0 / self._samples, 1.0)
		return min(127, max(-127, int(whole))) & 0xff, frac * self._samples / 127.0


class Gamepad:
	def __init__(self, devices, joystick0, joystick1, ms=50):
		self._device = find_device(devices, usage_page=1, usage=5)
		self._report = bytearray(7)
		self._joystick0 = joystick0
		self._joystick1 = joystick1
		self.ms = ms
		self._x0 = 0.0
		self._y0 = 0.0
		self._x1 = 0.0
		self._y1 = 0.0
		self._samples = 0
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
		if everyms(1, id(self) - 1):
			x0, y0 = self._joystick0.loop()
			self._x0 += x0
			self._y0 += y0
			x1, y1 = self._joystick1.loop()
			self._x1 += x1
			self._y1 += y1
			self._samples += 1
			if everyms(self.ms, id(self)):
				self._report[3] = self._round(self._x0)
				self._report[4] = self._round(self._y0)
				self._report[5] = self._round(self._x1)
				self._report[6] = self._round(self._y1)
				self._send()
				self._x0 = 0.0
				self._y0 = 0.0
				self._x1 = 0.0
				self._y1 = 0.0
				self._samples = 0

	def _round(self, n):
		return min(127, max(-127, round(127.0 * n / self.samples))) & 0xff
	
	def _send(self):
		self._device.send_report(self._report)

class Log:
	def __init__(self, capacity=100):
		self.capacity = capacity
		self._events = [Log.Event(0, False, ()) for i in range(self.capacity)]
		self._i = -1
		self._len = 0

	def __iter__(self):
		for i in range(self._len):
			yield self[i]

	def __call__(self, timestamp, pressed, switch):
		# Ensure len(self._events) = self.capacity in case capacity changed
		if len(self._events) > self.capacity:
			del self._events[(self._i + 1):(self._i + 1 + len(self._events) - self.capacity)]
			print('a',[e.timestamp for e in self._events])
			if len(self._events) > self.capacity:
				self._i -= (len(self._events) - self.capacity)
				del self._events[:(len(self._events) - self.capacity)]
		while len(self._events) < self.capacity:
			self._events.insert(self._i + 1, Log.Event(0, False, ()))
		
		self._i = (self._i + 1) % self.capacity
		self._len = min(self.capacity, 1 + self._len)
		self._events[self._i].timestamp = timestamp
		self._events[self._i].pressed = pressed
		self._events[self._i].switch = switch

	def __len__(self):
		return self._len

	def __getitem__(self, i):
		# [0] is the previous event, [1] is the one before that
		return self._events[self._i - i]

	class Event:
		def __init__(self, timestamp, pressed, s):
			self.timestamp = timestamp
			self.pressed = pressed
			self.switch = s

log = Log()

LOOPS = {}
def addloop(name, script=None):
	if script is None:
		def wrapper(fun):
			LOOPS[name] = fun
		return wrapper
	if isinstance(script, str):
		script = lambda _=compile(script, name, 'exec'): exec(_, globals(), globals())
	LOOPS[name] = script

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
		removeloop(name)

addloop('loop', lambda: command(['loop']))

neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

A0 = AnalogIn(board.A0)
A1 = AnalogIn(board.A1)
A2 = AnalogIn(board.A2)
A3 = AnalogIn(board.A3)
JOYSTICK0 = JoyStick(A0, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A1, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))
JOYSTICK1 = JoyStick(A2, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A3, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))

_event = keypad.Event()
event = None
KEYMATRIX = keypad.KeyMatrix(
	columns_to_anodes=True,
	column_pins=(board.D5, board.D6, board.D7, board.D8, board.D9, board.D10),
	row_pins=(board.D2, board.D3, board.D4, board.SCK, board.MISO, board.MOSI),
	debounce_threshold=2,
)
print('key_count=' + str(KEYMATRIX.key_count))
pressed_switches = set()

_switch_hist = {}
_switch_hist_count = 0
try:
	for line in open('/hist.txt'):
		line = line.strip()
		comma = line.find(',')
		if comma < 1 or not line[:comma].isdigit() or comma >= len(line):
			continue
		_switch_hist[line[comma + 1:]] = int(line[:comma])
except Exception as e:
	print(e)

def flush_switch_hist():
	if _switch_hist_count < 100:
		return
	with open('/hist.txt', 'w') as f:
		for s, n in _switch_hist.items():
			f.write('{n},{s}\n'.format(s=' '.join(s), n=n))
	_switch_hist_count = 0

@addloop('switch_hist')
def _():
	global _switch_hist_count
	if event and event.pressed:
		_switch_hist[KEYMAP[event.key_number]] = _switch_hist.get(KEYMAP[event.key_number], 0) + 1
		_switch_hist_count += 1
	if everyms(1000 * 60 * 10, 'flush_switch_hist'):
		flush_switch_hist()

FakeEvent = collections.namedtuple('FakeEvent', ('pressed',))
PRESS = FakeEvent(True)
RELEASE = FakeEvent(False)

#for d in usb_hid.devices: print('device ' + str(d.usage_page) + ' ' + str(d.usage))

keyboard = Keyboard(usb_hid.devices)
consumer = ConsumerControlWrapper(usb_hid.devices)
mouse = Mouse(usb_hid.devices)
gamepad = Gamepad(usb_hid.devices, JOYSTICK0, JOYSTICK1)
joymouse = JoyMouse(JOYSTICK0)

set_layer('default')
command(['setup'])

while True:
	event = None
	if KEYMATRIX.events.get_into(_event) and 0 <= _event.key_number < len(KEYMAP):
		event = _event
		if event.pressed:
			pressed_switches.add(KEYMAP[event.key_number])
		elif KEYMAP[event.key_number] in pressed_switches:
			pressed_switches.remove(KEYMAP[event.key_number])
		switch(KEYMAP[event.key_number])
		log(event.timestamp, event.pressed, KEYMAP[event.key_number])
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
			command(serial_bytes.strip().split())
	runloops()
