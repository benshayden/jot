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
import types
import usb_hid
from analogio import AnalogIn
from binascii import hexlify, unhexlify
from jot import run_script, uncache_script, CommandLineInterface, Gamepad, RingBuffer, AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket, Interval, JoyStick, SwitchEvent, PRESS, RELEASE, tasks, DigitalIn, DigitalOut
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
		self.speed = speed
		self._x = 0.0
		self._y = 0.0
		self.task = tasks.create(ms=ms)(self.loop)
	
	def loop(self, now):
		dx, self._x = self._buf(self._x + self.joystick.x)
		dy, self._y = self._buf(self._y + self.joystick.y)
		mouse.move(dx, dy)
	
	def _buf(self, n):
		whole, frac = divmod(self.speed * n * 127.0, 1.0)
		return min(127, max(-127, int(whole))) & 0xff, (frac / 127.0) / self.speed

cli = CommandLineInterface('commands', globals())
neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
A0 = AnalogIn(board.A0)
A1 = AnalogIn(board.A1)
joystick = JoyStick(A0, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A1, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))
joymouse = JoyMouse(joystick)
joymouse.task.enabled = False # todo remove

blue_led = DigitalOut(board.BLUE_LED) if hasattr(board, 'BLUE_LED') else None
red_led = DigitalOut(board.RED_LED) if hasattr(board, 'RED_LED') else None
switch = Debouncer(DigitalIn(board.SWITCH)) if hasattr(board, 'SWITCH') else None
if switch: SwitchEvent.source(switch, 1)

keypad_event = keypad.Event()
key_matrix = keypad.KeyMatrix(
	columns_to_anodes=True,
	column_pins=(board.D12, board.D11, board.D10, board.D9, board.D6, board.D5),
	row_pins=(board.D2, board.TX, board.RX),
	debounce_threshold=2,
)
SwitchEvent.source(key_matrix, key_matrix.key_count)

set_layer('default')
@tasks.create()
def switch_event_task(now):
	if switch:
		switch.update()
		if switch.rose or switch.fell:
			SwitchEvent.dispatch(switch.fell, 0, switch)
	if key_matrix.events.get_into(keypad_event):
		SwitchEvent.dispatch(keypad_event.pressed, keypad_event.key_number, key_matrix)

tasks.create(ms=1000 * 60 * 10)(lambda now: SwitchEvent.flush_histogram())

i2c = board.I2C() if hasattr(board, 'I2C') else None

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
		print('magnetic', lis3mdl.magnetic)
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

for d in usb_hid.devices:
	print(f'usb device f{d.usage_page} {d.usage}')
keyboard = usb_keyboard = Keyboard(usb_hid.devices)
keyboard_layout = usb_keyboard_layout = KeyboardLayoutUS(usb_keyboard)
consumer = usb_consumer = ConsumerControlWrapper(usb_hid.devices)
mouse = usb_mouse = Mouse(usb_hid.devices)
gamepad = usb_gamepad = Gamepad(usb_hid.devices, *joysticks)

aux_joystick = types.SimpleNamespace()
aux_joystick.x = aux_joystick.y = 0.0
joysticks = (aux_joystick, joystick)

aux_apds9960 = types.SimpleNamespace()
aux_apds9960.proximity = 0
aux_apds9960.color_data = (0, 0, 0)

aux_lsm6ds = types.SimpleNamespace()
aux_lsm6ds.gyro = (0.0, 0.0, 0.0)
aux_lsm6ds.acceleration = (0.0, 0.0, 0.0)

aux_lis3mdl = types.SimpleNamespace()
aux_lis3mdl.magnetic = (0.0, 0.0, 0.0)

try:
	import _bleio
	from adafruit_ble import BLERadio
	from adafruit_ble.services.nordic import UARTService
	from adafruit_ble.services.standard.hid import HIDService, DEFAULT_HID_DESCRIPTOR
	from adafruit_ble.services.standard.device_info import DeviceInfoService
	from adafruit_ble.services.standard import BatteryService
	
	ble = BLERadio()
	ble.name = 'jot'
	
	ble_uart_service = UARTService()
	SwitchEvent.source(ble_uart_service, 19)
	@tasks.create()
	def ble_uart_receive_task(now):
		# todo wait for aux to say the magic word before trusting anything else it says.
		if not ble_uart_service.in_waiting:
			return
		packet = Packet.from_stream(ble_uart_service)
		print('received', packet)
		if isinstance(packet, ButtonPacket):
			SwitchEvent.dispatch(packet.pressed, packet.index, ble_uart_service)
		elif isinstance(packet, JoystickPacket):
			aux_joystick.x = packet.x
			aux_joystick.y = packet.y
		elif isinstance(packet, ProximityPacket):
			aux_apds9960.proximity = packet.proximity
		elif isinstance(packet, GyroPacket):
			aux_lsm6ds.gyro = packet.x, packet.y, packet.z
		elif isinstance(packet, AccelerometerPacket):
			aux_lsm6ds.acceleration = packet.x, packet.y, packet.z
		elif isinstance(packet, MagnetometerPacket):
			aux_lis3mdl.magnetic = packet.x, packet.y, packet.z
		elif isinstance(packet, ColorPacket):
			aux_apds9960.color_data = color
	
	ble_hid = HIDService(DEFAULT_HID_DESCRIPTOR + Gamepad.DESCRIPTOR)
	@tasks.create(ms=1000)
	def hid_mode_task(now):
		global keyboard, keyboard_layout, mouse, gamepad, consumer
		use_usb = (len([c for c in ble.connections if c.paired]) < 1) # prefer ble_hid if available
		#use_usb = supervisor.runtime.usb_connected # prefer usb_hid if available
		if use_usb and keyboard != usb_keyboard:
			keyboard = usb_keyboard
			keyboard_layout = usb_keyboard_layout
			mouse = usb_mouse
			gamepad = usb_gamepad
			consumer = usb_consumer
			print('using usb hid')
		elif not use_usb and keyboard == usb_keyboard:
			for d in ble_hid.devices:
				print(f'ble device {d.usage_page} {d.usage}')
			keyboard = Keyboard(ble_hid.devices)
			keyboard_layout = KeyboardLayoutUS(keyboard)
			consumer = ConsumerControlWrapper(ble_hid.devices)
			mouse = Mouse(ble_hid.devices)
			gamepad = Gamepad(ble_hid.devices, *joysticks)
			print('using ble hid')
	
	ble_battery_service = BatteryService()
	voltage_monitor = AnalogIn(board.VOLTAGE_MONITOR) if hasattr(board, 'VOLTAGE_MONITOR') else None
	if voltage_monitor:
		_battery_voltage = RingBuffer(100)
		@tasks.create(ms=600)
		def measure_battery_task(now):
			_battery_voltage(voltage_monitor.value * 3.6 * 2.0 / 65536.0)

		@tasks.create(ms=60000)
		def ble_battery_service_task(now):
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
	
	ble_device_info_service = DeviceInfoService(
		software_revision='2025-03-03',
		manufacturer='bsh',
		model_number=ble.name,
	)

	from jot.two_blue import TwoBlue
	two_blue = TwoBlue(
		ble, blue_led,
		unpaired_services=(ble_uart_service,), unpaired_appearance=960,
		paired_services=(ble_hid, ble_device_info_service, ble_battery_service), paired_appearance=961)
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))

cli.run(['setup'])
cli_loop_task = tasks.create()(lambda now, args=('loop',): cli.run(args))

tasks.start()
