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
from jot import run_script, uncache_script, CommandLineInterface, Gamepad, RingBuffer, AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket, Interval, JoyStick, SwitchEvent, PRESS, RELEASE, tasks
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

set_layer('default')
@tasks.create()
def switch_event_task(now):
	SwitchEvent.current = None
	switch.update()
	if switch.rose or switch.fell:
		SwitchEvent.dispatch(switch.fell, 0, switch)
		# SwitchEvent.dispatch sets SwitchEvent.current. Let other tasks observe SwitchEvent.current in this task loop.
		# If there's also an event in key_matrix's queue, it can stay there until next task loop.
		return
	if key_matrix.events.get_into(keypad_event):
		SwitchEvent.dispatch(keypad_event.pressed, keypad_event.key_number, key_matrix)

tasks.create(ms=1000 * 60 * 10)(lambda now: SwitchEvent.flush_histogram())

i2c = board.I2C() if hasattr(board, 'I2C') else None

class AuxJoystick:
	def __init__(self):
		self.x = 0
		self.y = 0
aux_joystick = AuxJoystick()
joysticks = (aux_joystick, joystick)

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
gamepad = usb_gamepad = Gamepad(usb_hid.devices, *joysticks)

try:
	import _bleio
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
	
	@tasks.create(ms=1000)
	def hid_mode_task(now):
		global keyboard, keyboard_layout, mouse, gamepad, consumer, _hid_mode_tick
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
	
	ble_device_info_service = DeviceInfoService(
		software_revision='2025-03-03',
		manufacturer='bsh',
		model_number=ble.name,
	)
	
	# CircuitPython in aux cannot connect to large advertisements such as hid+uart,
	# so advertise uart for aux separately from advertising hid.
	ble_uart_advertisement = ProvideServicesAdvertisement(ble_uart_service)
	ble_uart_advertisement.appearance = 960 # generic hid
	ble_uart_advertisement.complete_name = ble.name
	ble_hid_advertisement = ProvideServicesAdvertisement(ble_hid, ble_device_info_service, ble_battery_service)
	ble_hid_advertisement.appearance = 961 # Keyboard
	ble_hid_advertisement.complete_name = ble.name

	_ble_advertising = None
	_blue_led_tick = 0
	@tasks.create()
	def ble_advertise_task(now):
		global _ble_advertising, _blue_led_tick
		num_paired = 0
		num_unpaired = 0
		for c in ble.connections:
			if c.paired:
				num_paired += 1
			else:
				num_unpaired += 1
		if num_paired > 1 or num_unpaired > 1:
			for c in ble.connections:
				c.disconnect()
			return # wait for the next task loop
		if num_unpaired < 1:
			if _ble_advertising != ble_uart_advertisement:
				ble.stop_advertising()
				ble.start_advertising(ble_uart_advertisement)
				_ble_advertising = ble_uart_advertisement
				print('advertising uart from', hexlify(ble.address_bytes))
			if blue_led and tasks.ticks_diff(now, _blue_led_tick) >= 250:
				_blue_led_tick = now
				blue_led.value = not blue_led.value
		elif num_paired < 1:
			if _ble_advertising != ble_hid_advertisement:
				ble.stop_advertising()
				ble.start_advertising(ble_hid_advertisement)
				_ble_advertising = ble_hid_advertisement
				print('advertising hid from', hexlify(ble.address_bytes))
			if blue_led and tasks.ticks_diff(now, _blue_led_tick) >= 500:
				_blue_led_tick = now
				blue_led.value = not blue_led.value
		else:
			if _ble_advertising != None:
				# done!
				ble.stop_advertising()
				_ble_advertising = None
				if blue_led:
					blue_led.value = True
				print('stopped advertising')
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))

cli.run(['setup'])
cli_loop_task = tasks.create()(lambda now, args=('loop',): cli.run(args))

tasks.start()
