from jot import run_script, uncache_script, CommandLineInterface, Gamepad, RingBuffer, AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket, JoyStick, Interval
import board
import collections
import neopixel
import keypad
import struct
import supervisor
import traceback
from digitalio import DigitalInOut, Direction, Pull
from analogio import AnalogIn
from binascii import hexlify, unhexlify
from time import sleep
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_debouncer import Debouncer
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_bmp280 import Adafruit_BMP280_I2C
from adafruit_lis3mdl import LIS3MDL
from adafruit_sht31d import SHT31D

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

neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
ble = BLERadio()
ble.name = '_jot'
if ble.advertising:
	ble.stop_advertising()
main_address = open('/main.txt', 'rb').read()

_switch = DigitalInOut(board.SWITCH)
_switch.direction = Direction.INPUT
_switch.pull = Pull.UP
switch = Debouncer(_switch)
key_event = keypad.Event()
key_matrix = keypad.KeyMatrix(
	columns_to_anodes=True,
	column_pins=(board.D12, board.D11, board.D10, board.D9, board.D6, board.D5),
	row_pins=(board.D2, board.RX, board.TX),
	debounce_threshold=2,
)
button_packet = ButtonPacket('1', True)
A0 = AnalogIn(board.A0)
A1 = AnalogIn(board.A1)
A2 = AnalogIn(board.A2)
A3 = AnalogIn(board.A3)
joystick = JoyStick(A0, Interval(0.0, 32750.0), Interval(32780.0, 65535.0), A1, Interval(0.0, 32750.0), Interval(32780.0, 65535.0))
joystick_packet = JoystickPacket(joystick.x, joystick.y)
i2c = board.I2C()
apds9960 = APDS9960(i2c)
apds9960.enable_proximity = True
apds9960.enable_color = True
proximity_packet = ProximityPacket(apds9960.proximity)
color_packet = ColorPacket(apds9960.color_data[:3])
bmp280 = Adafruit_BMP280_I2C(i2c)
lis3mdl = LIS3MDL(i2c)
magnetometer_packet = MagnetometerPacket(lis3mdl.magnetic[0], lis3mdl.magnetic[1], lis3mdl.magnetic[2])
sht31d = SHT31D(i2c)
try:
	from adafruit_lsm6ds.lsm6ds33 import LSM6DS33 as LSM6DS
	lsm6ds = LSM6DS(i2c)
except RuntimeError:
	from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
	lsm6ds = LSM6DS(i2c)
accelerometer_packet = AccelerometerPacket(lsm6ds.acceleration[0], lsm6ds.acceleration[1], lsm6ds.acceleration[2])
gyro_packet = GyroPacket(lsm6ds.gyro[0], lsm6ds.gyro[1], lsm6ds.gyro[2])

while True:
	uart_service = received = None
	try:
		uart_service = ble.connections[0][UARTService]
		received = uart_service.in_waiting
	except Exception as e:
		uart_service = None
		try:
			printed_scanned = set()
			for ad in ble.start_scan(ProvideServicesAdvertisement):
				ad_address = hexlify(ad.address.address_bytes)
				if ad_address == main_address:
					print('found', ad_address, ad.short_name, ad.complete_name, ad.appearance, ad.flags)
					try:
						connection = ble.connect(ad.address)
						uart_service = ble.connections[0][UARTService]
						ble.stop_scan()
						print('connected to main')
					except Exception as e:
						print('\n'.join(traceback.format_exception(e)))
				elif ad_address not in printed_scanned:
					print('scanned', ad_address)
					printed_scanned.add(ad_address)
		except Exception as e:
			print('\n'.join(traceback.format_exception(e)))
	if uart_service:
		if received:
			packet = Packet.from_stream(uart_service)
			print('received', packet)
			if isinstance(packet, ColorPacket):
				neopixel.fill(packet.color)
		switch.update()
		if switch.rose or switch.fell: # todo remove
			button_packet.index = key_matrix.key_count
			button_packet._pressed = not switch.value
			uart_service.write(button_packet.to_bytes())
			print('wrote', button_packet)
		if key_matrix.events.get_into(key_event):
			button_packet.index = key_event.key_number
			button_packet.pressed = key_event.pressed
			uart_service.write(button_packet.to_bytes())
			print('wrote', button_packet)
		# todo uart_service.write(joystick_packet.to_bytes())
		# todo uart_service.write(proximity_packet.to_bytes())
		# todo uart_service.write(color_packet.to_bytes())
		# todo uart_service.write(magnetometer_packet.to_bytes())
		# todo uart_service.write(accelerometer_packet.to_bytes())
		# todo uart_service.write(gyro_packet.to_bytes())
