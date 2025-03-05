import board
import collections
import gc
import keypad
import math
import microcontroller
import neopixel
import os
import struct
import supervisor
import sys
import time
import traceback
import usb_hid
from analogio import AnalogIn
import digitalio
from binascii import hexlify, unhexlify

from adafruit_hid import find_device
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.keycode import Keycode
from adafruit_hid.mouse import Mouse

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
	filename = f'/layers/{name}.txt'
	try:
		LAYER_CACHE[name] = [tuple(line.split(' ')) for line in open(filename).read().split('\n')]
		return LAYER_CACHE[name]
	except Exception as e:
		print(f'exception reading {filename}')
		print('\n'.join(traceback.format_exception(e)))

KEYMAP = []
LAYER_NAME = 'default'
def set_layer(name):
	global LAYER_NAME
	LAYER_NAME = name
	keymap = get_layer(name)
	if keymap:
		KEYMAP[:] = keymap

SCRIPT_CACHE = {}
__script_locals = {}
def run_switch(ARGS, _dir='switches'):
	globals()['ARGS'] = ARGS
	if not ARGS or not ARGS[0]:
		return
	filename = f'/{_dir}/{ARGS[0]}.py'
	if filename not in SCRIPT_CACHE:
		try:
			SCRIPT_CACHE[filename] = compile(open(filename).read(), filename, 'exec')
		except Exception as e:
			print(f'exception loading {filename}')
			print('\n'.join(traceback.format_exception(e)))
	if filename not in SCRIPT_CACHE:
		return
	if filename not in __script_locals:
		__script_locals[filename] = {}
	try:
		exec(SCRIPT_CACHE[filename], globals(), __script_locals[filename])
	except Exception as e:
		print('\n'.join(traceback.format_exception(e)))

def run_command(ARGS):
	run_switch(ARGS, _dir='commands')

class CommandLineInterface:
	def __init__(self):
		self._block = ''
		self._block_callback = None
	
	def read_block(self, callback):
		self._block_callback = callback

	def loop(self):
		# If a command is entered on Serial, exec it.
		# https://webserial.io/
		if supervisor.runtime.serial_bytes_available:
			serial_bytes = sys.stdin.read(supervisor.runtime.serial_bytes_available)
			if self._block_callback:
				self._block += serial_bytes
				if self._block.endswith('\n\n') or self._block.endswith('\r\n\r\n'):
					try:
						self._block_callback(self._block)
					except Exception as e:
						print('\n'.join(traceback.format_exception(e)))
					self._block = ''
					self._block_callback = None
			else:
				run_command(serial_bytes.strip().split())

cli = CommandLineInterface()

_TICKS_PERIOD = const(1<<29)
_TICKS_MAX = const(_TICKS_PERIOD-1)
_TICKS_HALFPERIOD = const(_TICKS_PERIOD//2)

def ticks_add(ticks, delta):
    # Add a delta to a base number of ticks, performing wraparound at 2**29ms.
    return (ticks + delta) % _TICKS_PERIOD

def ticks_diff(ticks1, ticks2):
    # Compute the signed difference between two ticks values, assuming that they are within 2**28 ticks
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
	def __init__(self, xpin, xneg, xpos, ypin, yneg, ypos, window_ms=50):
		self._xpin = xpin
		self._xneg = xneg
		self._xpos = xpos
		self._ypin = ypin
		self._yneg = yneg
		self._ypos = ypos
		self._tick = 0
		self._x = RingBuffer(capacity=window_ms, init=0.0)
		self._y = RingBuffer(capacity=window_ms, init=0.0)

	@property
	def x(self) -> float:
		return sum(self._x) / len(self._x) if len(self._x) else 0.0

	@property
	def y(self) -> float:
		return sum(self._y) / len(self._y) if len(self._y) else 0.0

	def _norm(self, val, neg, pos):
		if neg.min <= val <= neg.max:
			return (val - neg.max) / (neg.max - neg.min)
		elif pos.min <= val <= pos.max:
			return (val - pos.min) / (pos.max - pos.min)
		return 0.0
	
	def loop(self):
		now = supervisor.ticks_ms()
		if ticks_diff(now, self._tick) > 0:
			self._tick = now
			self._x(self._norm(self._xpin.value, self._xneg, self._xpos))
			self._y(self._norm(self._ypin.value, self._yneg, self._ypos))

class JoyMouse:
	def __init__(self, joystick, ms=50, speed=0.1):
		self._joystick = joystick
		self.ms = ms
		self.speed = speed
		self._x = 0.0
		self._y = 0.0
	
	def loop(self):
		self._joystick.loop()
		if everyms(self.ms, id(self)):
			dx, self._x = self._buf(self._x + self._joystick.x)
			dy, self._y = self._buf(self._y + self._joystick.y)
			mouse.move(dx, dy)
	
	def _buf(self, n):
		whole, frac = divmod(self.speed * n * 127.0, 1.0)
		return min(127, max(-127, int(whole))) & 0xff, (frac / 127.0) / self.speed


class Gamepad:
	def __init__(self, devices, joystick0, joystick1, ms=50):
		self._device = find_device(devices, usage_page=1, usage=5)
		self._report = bytearray(7)
		self._joystick0 = joystick0
		self._joystick1 = joystick1
		self.ms = ms
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
		self._joystick0.loop()
		self._joystick1.loop()
		if everyms(self.ms, id(self)):
			prevjoys = tuple(self._report[3:7])
			nextjoys = (
				self._round(self._joystick0.x),
				self._round(self._joystick0.y),
				self._round(self._joystick1.x),
				self._round(self._joystick1.y),
			)
			if nextjoys != prevjoys:
				self._report[3:7] = nextjoys
				self._send()

	def _round(self, n):
		return min(127, max(-127, round(127.0 * n / self.samples))) & 0xff
	
	def _send(self):
		self._device.send_report(self._report)

class RingBuffer:
	def __init__(self, capacity=100, init=None):
		self.capacity = capacity
		self._init = init if callable(init) else lambda: init
		self._events = [self._init() for i in range(capacity)]
		self._i = -1
		self._len = 0

	def _update(self, *params):
		self._events[self._i] = params[0]

	def __iter__(self):
		for i in range(self._len):
			yield self[i]

	def __call__(self, *params):
		# Ensure len(self._events) = self.capacity in case capacity changed
		if len(self._events) > self.capacity:
			del self._events[(self._i + 1):(self._i + 1 + len(self._events) - self.capacity)]
			if len(self._events) > self.capacity:
				self._i -= (len(self._events) - self.capacity)
				del self._events[:(len(self._events) - self.capacity)]
		while len(self._events) < self.capacity:
			self._events.insert(self._i + 1, self._init())
		
		self._i = (self._i + 1) % self.capacity
		self._len = min(self.capacity, 1 + self._len)
		self._update(*params)

	def __len__(self):
		return self._len

	def __getitem__(self, i):
		# [0] is the previous event, [1] is the one before that
		return self._events[self._i - i]

class Log(RingBuffer):	
	class Event:
		def __init__(self, timestamp=0, pressed=False, swtch=()):
			self.timestamp = timestamp
			self.pressed = pressed
			self.switch = swtch
		
		def __repr__(self):
			return f'Event({self.timestamp}, {self.pressed}, {self.switch})'

	def __init__(self, capacity=100):
		super().__init__(capacity=capacity, init=lambda: Log.Event())

	def _update(self, timestamp, pressed, swtch):
		self._events[self._i].timestamp = timestamp
		self._events[self._i].pressed = pressed
		self._events[self._i].switch = swtch

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
			print('\n'.join(traceback.format_exception(e)))
			bad.append(name)
	for name in bad:
		removeloop(name)

addloop('loop', lambda: run_command(['loop']))
addloop('cli', lambda: cli.loop())

neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

A0 = AnalogIn(board.A0)
A1 = AnalogIn(board.A1)
A2 = AnalogIn(board.A2)
A3 = AnalogIn(board.A3)
JOYSTICK0 = JoyStick(A0, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A1, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))
JOYSTICK1 = JoyStick(A2, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A3, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))

keypad_event = keypad.Event()
event = None
key_matrix = keypad.KeyMatrix(
	columns_to_anodes=True,
	column_pins=(board.D12, board.D11, board.D10, board.D9, board.D6, board.D5),
	row_pins=(board.D2, board.TX, board.RX),
	debounce_threshold=2,
)
print(f'key_count={key_matrix.key_count}')
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
	print('\n'.join(traceback.format_exception(e)))

def flush_switch_hist():
	global _switch_hist_count
	if _switch_hist_count < 100:
		return
	with open('/hist.txt', 'w') as f:
		for s, n in sorted(_switch_hist.items(), key=lambda item: -item[1]):
			f.write(f'{n},{' '.join(s)}\n')
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

if hasattr(board, 'BLUE_LED'):
	blue_led = digitalio.DigitalInOut(board.BLUE_LED)
	blue_led.direction = digitalio.Direction.OUTPUT
else:
	blue_led = None
if hasattr(board, 'RED_LED'):
	red_led = digitalio.DigitalInOut(board.RED_LED)
	red_led.direction = digitalio.Direction.OUTPUT
else:
	red_led = None

i2c = board.I2C() if hasattr(board, 'I2C') else None

keyboard = usb_keyboard = Keyboard(usb_hid.devices)
keyboard_layout = usb_keyboard_layout = KeyboardLayoutUS(usb_keyboard)
consumer = usb_consumer = ConsumerControlWrapper(usb_hid.devices)
mouse = usb_mouse = Mouse(usb_hid.devices)
gamepad = usb_gamepad = Gamepad(usb_hid.devices, JOYSTICK0, JOYSTICK1)

try:
	from adafruit_ble import BLERadio
	from adafruit_ble.advertising import Advertisement
	from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
	from adafruit_ble.services.nordic import UARTService
	from adafruit_ble.services.standard.hid import HIDService, DEFAULT_HID_DESCRIPTOR
	from adafruit_ble.services.standard.device_info import DeviceInfoService
	from adafruit_ble.services.standard import BatteryService
	from adafruit_bluefruit_connect.packet import Packet
	from adafruit_bluefruit_connect.button_packet import ButtonPacket
	from adafruit_bluefruit_connect.color_packet import ColorPacket
	from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
	from adafruit_bluefruit_connect.magnetometer_packet import MagnetometerPacket
	from adafruit_bluefruit_connect.gyro_packet import GyroPacket
	from adafruit_bluefruit_connect._xyz_packet import _XYZPacket
	
	def reprButtonPacket(self):
		return f'ButtonPacket({self.button}, {self.pressed})'
	ButtonPacket.__repr__ = reprButtonPacket
	
	def reprAccelerometerPacket(self):
		return f'AccelerometerPacket({self._x}, {self._y}, {self._z})'
	AccelerometerPacket.__repr__ = reprAccelerometerPacket
	
	def reprMagnetometerPacket(self):
		return f'MagnetometerPacket({self._x}, {self._y}, {self._z})'
	MagnetometerPacket.__repr__ = reprMagnetometerPacket
	
	def reprColorPacket(self):
		return f'ColorPacket({self.color})'
	ColorPacket.__repr__ = reprColorPacket

	def reprGyroPacket(self):
		return f'GyroPacket({self._x}, {self._y}, {self._z})'
	GyroPacket.__repr__ = reprGyroPacket
	
	class JoystickPacket(Packet):
		_FMT_PARSE: str = '<xxffx'
		PACKET_LENGTH: int = struct.calcsize(_FMT_PARSE)
		# _FMT_CONSTRUCT doesn't include the trailing checksum byte.
		_FMT_CONSTRUCT: str = '<2sff'
		_TYPE_HEADER: bytes = b'!J'
	
		def __init__(self, x: float, y: float):
			self._x = x
			self._y = y
	
		def to_bytes(self) -> bytes:
			partial_packet = struct.pack(
					self._FMT_CONSTRUCT,
					self._TYPE_HEADER,
					self._x,
					self._y,
			)
			return self.add_checksum(partial_packet)
	
		def __repr__(self):
			return f'JoystickPacket({self._x}, {self._y})'
	JoystickPacket.register_packet_type()
	
	class ProximityPacket(Packet):
		_FMT_PARSE: str = '<xxfx'
		PACKET_LENGTH: int = struct.calcsize(_FMT_PARSE)
		_FMT_CONSTRUCT: str = '<2sf'
		_TYPE_HEADER: bytes = b'!P'
		
		def __init__(self, proximity: float):
			self.proximity = proximity
		
		def to_bytes(self) -> bytes:
			partial_packet = struct.pack(
					self._FMT_CONSTRUCT,
					self._TYPE_HEADER,
					self.proximity,
			)
			return self.add_checksum(partial_packet)
		
		def __repr__(self):
			return f'ProximityPacket({self.proximity})'
	ProximityPacket.register_packet_type()
	
	gamepad_descriptor = b'' # todo try to read this from usb_gamepad
	
	ble = BLERadio()
	ble.name = 'jot'
	ble_hid = HIDService(DEFAULT_HID_DESCRIPTOR + gamepad_descriptor)
	ble_keyboard = Keyboard(ble_hid.devices)
	ble_keyboard_layout = KeyboardLayoutUS(ble_keyboard)
	ble_consumer = ConsumerControlWrapper(ble_hid.devices)
	ble_mouse = Mouse(ble_hid.devices)
	ble_gamepad = None  # todo Gamepad(ble_hid.devices, JOYSTICK0, JOYSTICK1)
	ble_device_info_service = DeviceInfoService(
		software_revision='2025-03-03',
		manufacturer='bsh',
		model_number=ble.name,
	)
	ble_battery_service = BatteryService()
	ble_uart_service = UARTService()
	ble_advertisement = ProvideServicesAdvertisement(ble_hid, ble_device_info_service, ble_battery_service)
	ble_advertisement.appearance = 961 # Keyboard
	ble_advertisement.complete_name = ble.name
	ble.start_advertising(ble_advertisement)
	print('advertising', hexlify(ble.address_bytes))
	voltage_monitor = AnalogIn(board.VOLTAGE_MONITOR) if hasattr(board, 'VOLTAGE_MONITOR') else None
	
	@addloop('battery')
	def _():
		if everyms(1000 * 60, 'battery'):
			# todo average RingBuffer?
			# todo record times between percentages?
			ble_battery_service.level = min(100, max(0, int(100.0 * voltage_monitor.value / 65536.0)))
			print(f'battery adc={voltage_monitor.value} percent={ble_battery_service.level}')
			red_led = (ble_battery_service.level < 10)

	class AuxJoystick:
		def __init__(self):
			self.x = 0
			self.y = 0

		def loop(self):
			pass

	ble_aux_joystick = AuxJoystick()

	class AuxAPDS9960:
		def __init__(self):
			self.proximity = 0
			self.color = (0, 0, 0)
	ble_aux_apds9960 = AuxAPDS9960()

	ble_aux_address = open('/aux.txt').read()
	aux_connection = None
	@addloop('aux')
	def _():
		global aux_connection
		if aux_connection is None and everyms(100, 'ble_aux'):
			for ad in ble.start_scan(ProvideServicesAdvertisement):
				if hexlify(ad.address.address_bytes) == ble_aux_address:
					aux_connection = ble.connect(ad)
					aux_connection.pair()
					print('Connected')
					break
			ble.stop_scan()
		if aux_connection and everyms(10, 'ble_aux'):
			try:
				received = aux_connection[UARTService].in_waiting
			except Exception as e:
				received = False
				aux_connection = None
			if received:
				packet = Packet.from_stream(aux_connection[UARTService])
				print('received', packet)
				if isinstance(packet, ButtonPacket):
					# todo keymap etc
					neopixel.fill((0, 0, 0) if packet.pressed else (10, 10, 10))
				elif isinstance(packet, JoystickPacket):
					ble_aux_joystick.x = packet.x
					ble_aux_joystick.y = packet.y
				elif isinstance(packet, ProximityPacket):
					ble_aux_apds9960.proximity = packet.proximity
				# todo proxy key_matrix, apds9960, etc from aux for scripts to access

	@addloop('ble')
	def _():
		pass
		# todo blue_led.value = True when host not connected
		# todo ble.stop_advertising() when a client is paired
		# todo ble.start_advertising(ble_advertisement, ble_scan_response) when either a host or aux are not paired
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))
	ble = None
	ble_keyboard = None
	ble_mouse = None
	ble_consumer = None
	ble_gamepad = None
	ble_aux_joystick = None

try:
	from adafruit_apds9960.apds9960 import APDS9960
	apds9960 = APDS9960(i2c)
	apds9960.enable_proximity = True
	apds9960.enable_color = True
	
	print('proximity', apds9960.proximity)
	print('color', apds9960.color_data)
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))
	apds9960 = None

try:
	from adafruit_bmp280 import Adafruit_BMP280_I2C
	bmp280 = Adafruit_BMP280_I2C(i2c)
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))
	bmp280 = None

try:
	from adafruit_lis3mdl import LIS3MDL
	lis3mdl = LIS3MDL(i2c)
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))
	lis3mdl = None

try:
	from adafruit_sht31d import SHT31D
	sht31d = SHT31D(i2c)
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))
	sht31d = None

joymouse = JoyMouse(JOYSTICK0)

set_layer('default')
run_command(['setup'])

while True:
	event = None
	if key_matrix.events.get_into(keypad_event) and 0 <= keypad_event.key_number < len(KEYMAP):
		event = keypad_event
		switch = KEYMAP[event.key_number]
		if event.pressed:
			pressed_switches.add(switch)
		elif switch in pressed_switches:
			pressed_switches.remove(switch)
		run_switch(switch)
		log(event.timestamp, event.pressed, switch)
	runloops()
