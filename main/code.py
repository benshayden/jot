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
from jot import run_script, uncache_script, CommandLineInterface, Gamepad, RingBuffer, AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket, Interval, JoyStick, SwitchEvent, PRESS, RELEASE
from adafruit_debouncer import Debouncer
from adafruit_hid import find_device
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.keycode import Keycode
from adafruit_hid.mouse import Mouse

__keymap_cache = {}
def get_layer(name):
	if name in __keymap_cache:
		return __keymap_cache[name]
	filename = f'/layers/{name}.txt'
	try:
		__keymap_cache[name] = [tuple(line.split(' ')) for line in open(filename).read().split('\n')]
		return __keymap_cache[name]
	except Exception as e:
		print(f'exception reading {filename}')
		print('\n'.join(traceback.format_exception(e)))

keymap_name = 'default'
def set_layer(name):
	global keymap_name
	keymap_name = name
	SwitchEvent.keymap = get_layer(name) or SwitchEvent.keymap

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

SwitchEvent.run = lambda args: run_script(args, 'switches', globals())

class ConsumerControlWrapper(ConsumerControl):
	def release(self, *unused):
		super().release()

def por(device, code):
	# press or release
	if SwitchEvent.current.pressed:
		device.press(code)
	else:
		device.release(code)

class JoyMouse:
	def __init__(self, joystick, ms=50, speed=0.1):
		self.joystick = joystick
		self.ms = ms
		self.speed = speed
		self._x = 0.0
		self._y = 0.0
	
	def loop(self):
		self.joystick.loop()
		if everyms(self.ms, id(self)):
			dx, self._x = self._buf(self._x + self.joystick.x)
			dy, self._y = self._buf(self._y + self.joystick.y)
			mouse.move(dx, dy)
	
	def _buf(self, n):
		whole, frac = divmod(self.speed * n * 127.0, 1.0)
		return min(127, max(-127, int(whole))) & 0xff, (frac / 127.0) / self.speed

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

cli = CommandLineInterface('commands', globals())
neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
A0 = AnalogIn(board.A0)
A1 = AnalogIn(board.A1)
joystick = JoyStick(A0, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A1, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))
joymouse = JoyMouse(joystick)

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

_switch = digitalio.DigitalInOut(board.SWITCH)
_switch.direction = digitalio.Direction.INPUT
_switch.pull = digitalio.Pull.UP
switch = Debouncer(_switch)
SwitchEvent.source(switch, 1)

keypad_event = keypad.Event()
key_matrix = keypad.KeyMatrix(
	columns_to_anodes=True,
	column_pins=(board.D12, board.D11, board.D10, board.D9, board.D6, board.D5),
	row_pins=(board.D2, board.TX, board.RX),
	debounce_threshold=2,
)
SwitchEvent.source(key_matrix, key_matrix.key_count)
i2c = board.I2C() if hasattr(board, 'I2C') else None

class AuxJoystick:
	def __init__(self):
		self.x = 0
		self.y = 0

	def loop(self):
		pass

aux_joystick = AuxJoystick()

class AuxAPDS9960:
	def __init__(self):
		self.proximity = 0
		self.color = (0, 0, 0)
aux_apds9960 = AuxAPDS9960()

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

bmp280 = None
if False:
	try:
		from adafruit_bmp280 import Adafruit_BMP280_I2C
		bmp280 = Adafruit_BMP280_I2C(i2c)
	except Exception as e:
		print('\n'.join(traceback.format_exception(e)))

lis3mdl = None
if False:
	try:
		from adafruit_lis3mdl import LIS3MDL
		lis3mdl = LIS3MDL(i2c)
	except Exception as e:
		print('\n'.join(traceback.format_exception(e)))

sht31d = None
if False:
	try:
		from adafruit_sht31d import SHT31D
		sht31d = SHT31D(i2c)
	except Exception as e:
		print('\n'.join(traceback.format_exception(e)))

try:
	from adafruit_lsm6ds.lsm6ds33 import LSM6DS33 as LSM6DS
	lsm6ds = LSM6DS(i2c)
except RuntimeError:
	from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
	lsm6ds = LSM6DS(i2c)
print('acceleration', lsm6ds.acceleration)
print('gyro', lsm6ds.gyro)

for d in usb_hid.devices: print('usb device ' + str(d.usage_page) + ' ' + str(d.usage))

keyboard = usb_keyboard = Keyboard(usb_hid.devices)
keyboard_layout = usb_keyboard_layout = KeyboardLayoutUS(usb_keyboard)
consumer = usb_consumer = ConsumerControlWrapper(usb_hid.devices)
mouse = usb_mouse = Mouse(usb_hid.devices)
gamepad = usb_gamepad = Gamepad(usb_hid.devices, joystick, aux_joystick)

addloop('loop', lambda: cli.run(['loop']))
addloop('cli', lambda: cli.loop())

@addloop('switch_hist')
def _():
	if everyms(1000 * 60 * 10, 'flush_switch_hist'):
		SwitchEvent.flush_histogram()

try:
	from adafruit_ble import BLERadio
	from adafruit_ble.advertising import Advertisement
	from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
	from adafruit_ble.services.nordic import UARTService
	from adafruit_ble.services.standard.hid import HIDService, DEFAULT_HID_DESCRIPTOR
	from adafruit_ble.services.standard.device_info import DeviceInfoService
	from adafruit_ble.services.standard import BatteryService
	
	ble = BLERadio()
	ble.stop_advertising()
	ble.name = 'jot'
	
	ble_hid = HIDService(DEFAULT_HID_DESCRIPTOR + Gamepad.DESCRIPTOR)
	
	for d in ble_hid.devices: print(f'ble device {d.usage_page} {d.usage}')
	
	ble_uart_service = UARTService()
	ble_device_info_service = DeviceInfoService(
		software_revision='2025-03-03',
		manufacturer='bsh',
		model_number=ble.name,
	)
	ble_battery_service = BatteryService()
	ble_hid_advertisement = ProvideServicesAdvertisement(ble_hid, ble_device_info_service, ble_battery_service)
	ble_hid_advertisement.appearance = 961 # Keyboard
	ble_hid_advertisement.complete_name = ble.name
	SwitchEvent.source(ble_uart_service, 19)
	
	_hid_mode_tick = 0
	@addloop('hid mode')
	def _():
		global keyboard, keyboard_layout, mouse, gamepad, consumer, _hid_mode_tick
		now = supervisor.ticks_ms()
		if ticks_diff(now, _hid_mode_tick) < 1000:
			return
		_hid_mode_tick = now
		use_usb = (len([c for c in ble.connections if c.paired]) < 1)
		#use_usb = supervisor.runtime.usb_connected
		if use_usb and keyboard != usb_keyboard:
			keyboard = usb_keyboard
			keyboard_layout = usb_keyboard_layout
			mouse = usb_mouse
			gamepad = usb_gamepad
			consumer = usb_consumer
			print('using usb hid')
		elif not use_usb and keyboard == usb_keyboard:
			keyboard = Keyboard(ble_hid.devices)
			keyboard_layout = KeyboardLayoutUS(keyboard)
			consumer = ConsumerControlWrapper(ble_hid.devices)
			mouse = Mouse(ble_hid.devices)
			# todo gamepad = Gamepad(ble_hid.devices, JOYSTICK0, JOYSTICK1)
			print('using ble hid')
	
	voltage_monitor = AnalogIn(board.VOLTAGE_MONITOR) if hasattr(board, 'VOLTAGE_MONITOR') else None
	if voltage_monitor:
		_battery_voltage = RingBuffer(100)
		_battery_raw_tick = 0
		@addloop('raw battery')
		def _():
			global _battery_raw_tick
			now = supervisor.ticks_ms()
			if ticks_diff(now, _battery_raw_tick) < 600:
				return
			_battery_raw_tick = now
			_battery_voltage(voltage_monitor.value * 3.6 * 2.0 / 65536.0)

		_battery_tick = 0
		@addloop('battery')
		def _():
			global _battery_tick
			now = supervisor.ticks_ms()
			if ticks_diff(now, _battery_tick) < 60000:
				return
			_battery_tick = now

			if len(_battery_voltage) < 10:
				return
			avg_v = sum(_battery_voltage) / len(_battery_voltage)
			# lipo batteries max out at 4.2V, and the regulator shuts off at 3.2V.
			norm_v = (avg_v - 3.2) / (4.2 - 3.2)
			# todo record times between percentages?
			ble_battery_service.level = min(100, max(0, int(100.0 * norm_v)))
			print(f'battery voltage={avg_v} percent={ble_battery_service.level}')
			if red_led:
				red_led.value = (ble_battery_service.level < 10)

	@addloop('aux')
	def _():
		# todo wait for aux to say the magic word before trusting anything else it says.
		if ble_uart_service.in_waiting:
			packet = Packet.from_stream(ble_uart_service)
			print('received', packet)
			if isinstance(packet, ButtonPacket):
				SwitchEvent.dispatch(packet.pressed, packet.index, ble_uart_service)
			elif isinstance(packet, JoystickPacket):
				aux_joystick.x = packet.x
				aux_joystick.y = packet.y
			elif isinstance(packet, ProximityPacket):
				aux_apds9960.proximity = packet.proximity
	
	# CircuitPython in aux cannot connect to large advertisements such as hid+uart,
	# so advertise uart for aux separately from advertising hid.
	ble_uart_advertisement = ProvideServicesAdvertisement(ble_uart_service)
	ble_uart_advertisement.appearance = 960 # generic hid
	ble_uart_advertisement.complete_name = ble.name

	_ble_advertising = None
	_blue_led_tick = 0
	@addloop('ble')
	def _():
		global _ble_advertising, _blue_led_tick
		now = supervisor.ticks_ms()
		if _ble_advertising == ble_uart_advertisement:
			if blue_led and ticks_diff(now, _blue_led_tick) >= 250:
				_blue_led_tick = now
				blue_led.value = not blue_led.value
			if len([c for c in ble.connections if not c.paired]) == 1:
				ble.stop_advertising()
				ble.start_advertising(ble_hid_advertisement)
				_ble_advertising = ble_hid_advertisement
				print('advertising hid from', hexlify(ble.address_bytes))
		elif _ble_advertising == ble_hid_advertisement:
			if blue_led and ticks_diff(now, _blue_led_tick) >= 500:
				_blue_led_tick = now
				blue_led.value = not blue_led.value
			if len([c for c in ble.connections if c.paired]) == 1:
				ble.stop_advertising()
				_ble_advertising = None
				print('stopped advertising')
		elif _ble_advertising is None:
			if blue_led:
				blue_led.value = True
			if len(ble.connections) != 2:
				# todo if there's 1 paired connection then assume it's the hid client, keep it, just advertise uart.
				# todo if there's 1 unpaired connection then assume it's aux, keep it, just advertise hid.
				for c in ble.connections:
					c.disconnect()
				ble.start_advertising(ble_uart_advertisement)
				_ble_advertising = ble_uart_advertisement
				print('advertising uart from', hexlify(ble.address_bytes))
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))

set_layer('default')
cli.run(['setup'])
gc.collect()

while True:
	SwitchEvent.current = None
	switch.update()
	if switch.rose or switch.fell:
		SwitchEvent.dispatch(switch.fell, 0, switch)
	if SwitchEvent.current is None and key_matrix.events.get_into(keypad_event):
		SwitchEvent.dispatch(keypad_event.pressed, keypad_event.key_number, key_matrix)
	runloops()
