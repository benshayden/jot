from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.color_packet import ColorPacket
from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
from adafruit_bluefruit_connect.accelerometer_packet import MagnetometerPacket
from adafruit_bluefruit_connect.gyro_packet import GyroPacket
from adafruit_bluefruit_connect._xyz_packet import _XYZPacket
import board
import neopixel
import keypad
from adafruit_debouncer import Debouncer
from digitalio import DigitalInOut, Direction, Pull
from binascii import hexlify, unhexlify
from time import sleep
from adafruit_ble.services.standard.device_info import DeviceInfoService, BatteryService
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_bmp280 import Adafruit_BMP280_I2C
from adafruit_lis3mdl import LIS3MDL
from adafruit_sht31d import SHT31D

class RingBuffer:
	def __init__(self, capacity):
		self._capacity = capacity

class JoyStick:
	def __init__(self, xpin, xneg, xpos, ypin, yneg, ypos, ms=1, capacity=50):
		self._xpin = xpin
		self._xneg = xneg
		self._xpos = xpos
		self._ypin = ypin
		self._yneg = yneg
		self._ypos = ypos
		self._ms = ms
		self._x = RingBuffer(capacity)
		self._y = RingBuffer(capacity)
	
	@property
	def x(self) -> float:
		return sum(self._x) / len(self._x)

	@property
	def y(self) -> float:
		return sum(self._y) / len(self._y)

	def _norm(self, val, neg, pos):
		if neg.min <= val <= neg.max:
			return (val - neg.max) / (neg.max - neg.min)
		elif pos.min <= val <= pos.max:
			return (val - pos.min) / (pos.max - pos.min)
		return 0.0
	
	def loop(self):
		self._x.add(self._norm(self.xin.value, self.xneg, self.xpos))
		self._y.add(self._norm(self.yin.value, self.yneg, self.ypos))

def reprButtonPacket(self):
	return f'ButtonPacket({self.button}, {self.pressed})'
ButtonPacket.__repr__ = reprButtonPacket

def reprAccelerometerPacket(self):
	return f'AccelerometerPacket({self._x}, {self._y}, {self._z})'
AccelerometerPacket.__repr__ = reprAccelerometerPacket

class JoystickPacket(Packet):
	_FMT_PARSE: str = "<xxffx"
	PACKET_LENGTH: int = struct.calcsize(_FMT_PARSE)
	# _FMT_CONSTRUCT doesn't include the trailing checksum byte.
	_FMT_CONSTRUCT: str = "<2sff"
	_TYPE_HEADER: bytes = b"!J"

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
		return f'JoystickPacket()'
JoystickPacket.register_packet_type()

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

device_info_service = DeviceInfoService(
	software_revision='2025-03-02',
	manufacturer='bsh',
	model_number='_jot',
)
battery_service = BatteryService()
neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
ble = BLERadio()
print('my address', hexlify(ble.address_bytes))
main_address = open('/mainaddr.txt').read()
ble.name = '_jot'
uart_service = UARTService()
advertisement = ProvideServicesAdvertisement(uart_service, device_info_service, battery_service)
advertisement.complete_name = ble.name
print('advertising', hexlify(ble.address_bytes))
ble.start_advertising(advertisement)
paired = False
_switch = DigitalInOut(board.SWITCH)
_switch.direction = Direction.INPUT
_switch.pull = Pull.UP
switch = Debouncer(_switch)
switchprev = switch.value
key_event = keypad.Event()
key_matrix = keypad.KeyMatrix(
	columns_to_anodes=True,
	column_pins=(board.D13, board.D12, board.D11, board.D10, board.D9, board.D6),
	row_pins=(board.D5, board.RX, board.TX),
	debounce_threshold=2,
)
button_packet = ButtonPacket('1', True)
i2c = board.I2C()
apds9960 = APDS9960(i2c)
apds9960.enable_proximity = True
apds9960.enable_color = True
proximity_packet = ProximityPacket(0.0)
bmp280 = Adafruit_BMP280_I2C(i2c)
lis3mdl = LIS3MDL(i2c)
sht31d = SHT31D(i2c)
try:
	from adafruit_lsm6ds.lsm6ds33 import LSM6DS33 as LSM6DS
	lsm6ds = LSM6DS(i2c)
except RuntimeError:
	from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
	lsm6ds = LSM6DS(i2c)
magnetometer_packet = MagnetometerPacket(lis3mdl.magnetic[0], lis3mdl.magnetic[1], lis3mdl.magnetic[2])
accelerometer_packet = AccelerometerPacket(lsm6ds.acceleration[0], lsm6ds.acceleration[1], lsm6ds.acceleration[2])
gyro_packet = GyroPacket(lsm6ds.gyro[0], lsm6ds.gyro[1], lsm6ds.gyro[2])

while True:
	if ble.connected:
		# todo confirm client address is main_address, pair
		for connection in ble.connections:
			if not connection:
				continue
			connection_address = hexlify(_bleio_connection.address)
			if connection_address == main_address:
				connection.pair()
				print('paired with main')
				paired = True
				ble.stop_advertising()
			else:
				print('disconnecting', connection_address)
				connection.disconnect()
	if ble.connected and paired:
		switch.update()
		if switch.value != switchprev:
			print('sending switch', switch.value)
			button_packet._pressed = switch.value
			uart_service.write(button_packet.to_bytes())
			switchprev = switch.value
			print('sent switch', switch.value)
		if uart_service.in_waiting:
			packet = Packet.from_stream(uart_service)
			print('received', packet)
			if isinstance(packet, ButtonPacket):
				neopixel.fill((0, 0, 0) if packet.pressed else (10, 10, 10))
		if key_matrix.events.get_into(key_event):
			button_packet._button = ord(0x30 + key_event.key_number)
			button_packet._pressed = key_event.pressed
			uart_service.write(button_packet.to_bytes())
			print('sent', button_packet)
	elif not ble.advertising:
		print('advertising', hexlify(ble.address_bytes))
		ble.start_advertising(advertisement)
	sleep(0.01)
