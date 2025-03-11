from jot import RingBuffer, AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket, JoyStick, Interval, tasks
import board
import collections
import gc
import neopixel
import keypad
import struct
import supervisor
import traceback
import digitalio
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

neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
ble = BLERadio()
ble.name = '_jot'
if ble.advertising:
	ble.stop_advertising()
main_address = open('/main.txt', 'rb').read()

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
gc.collect()

@tasks.create()
def reconnect(now):
	try:
		unused = ble.connections[0][UARTService].in_waiting
	except Exception as e:
		if blue_led:
			blue_led.value = False
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
						if blue_led:
							blue_led.value = True
					except Exception as e:
						print('\n'.join(traceback.format_exception(e)))
				elif ad_address not in printed_scanned:
					print('scanned', ad_address)
					printed_scanned.add(ad_address)
		except Exception as e:
			print('\n'.join(traceback.format_exception(e)))

def safe_get_uart_service():
	try:
		# in_waiting may also throw so safe that also.
		uart_service = ble.connections[0][UARTService]
		return uart_service, uart_service.in_waiting
	except Exception as e:
		# reconnect will do so soon, it's ok if the caller has no effect.
		return None, False

@tasks.create()
def uart_receive(now):
	uart_service, received = safe_get_uart_service()
	if not uart_service:
		return
	if received:
		packet = Packet.from_stream(uart_service)
		print('received', packet)
		if isinstance(packet, ColorPacket):
			neopixel.fill(packet.color)

@tasks.create(ms=7)
def uart_send_buttons(now):
	uart_service, received = safe_get_uart_service()
	if not uart_service:
		return
	switch.update()
	if switch.rose or switch.fell: # todo remove
		button_packet.index = key_matrix.key_count
		button_packet.pressed = not switch.value
		uart_service.write(button_packet.to_bytes())
		print('wrote', button_packet)
	if key_matrix.events.get_into(key_event):
		button_packet.index = key_event.key_number
		button_packet.pressed = key_event.pressed
		uart_service.write(button_packet.to_bytes())
		print('wrote', button_packet)

@tasks.create(ms=40)
def uart_send_joystick(now):
	uart_service, received = safe_get_uart_service()
	if not uart_service:
		return
	newjoy = joystick.x, joystick.y
	if abs(newjoy[0] - joystick_packet.x) > 1 or abs(newjoy[1] - joystick_packet.y) > 1:
		joystick_packet.x = newjoy[0]
		joystick_packet.y = newjoy[1]
		uart_service.write(joystick_packet.to_bytes())

@tasks.create(ms=10)
def uart_send_proximity(now):
	uart_service, received = safe_get_uart_service()
	if not uart_service:
		return
	if abs(proximity_packet.proximity - apds9960.proximity) > 1:
		proximity_packet.proximity = apds9960.proximity
		uart_service.write(proximity_packet.to_bytes())

# todo uart_service.write(color_packet.to_bytes())
# todo uart_service.write(magnetometer_packet.to_bytes())
# todo uart_service.write(accelerometer_packet.to_bytes())
# todo uart_service.write(gyro_packet.to_bytes())

tasks.start()
