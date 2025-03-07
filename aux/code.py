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
from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.color_packet import ColorPacket
from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
from adafruit_bluefruit_connect.magnetometer_packet import MagnetometerPacket
from adafruit_bluefruit_connect.gyro_packet import GyroPacket
from adafruit_debouncer import Debouncer
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_bmp280 import Adafruit_BMP280_I2C
from adafruit_lis3mdl import LIS3MDL
from adafruit_sht31d import SHT31D

Interval = collections.namedtuple('Interval', ('min', 'max'))

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
	# _FMT_CONSTRUCT doesn't include the trailing checksum byte.
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
	if ble.connections and ble.connections[0] and ble.connections[0].connected and ble.connections[0].paired and ble.connections[0][UARTService]:
		uart_service = ble.connections[0][UARTService]
		switch.update()
		if switch.rose or switch.fell: # todo remove
			button_packet._pressed = switch.value
			uart_service.write(button_packet.to_bytes())
		try:
			received = uart_service.in_waiting
		except Exception as e:
			received = False
		if received:
			packet = Packet.from_stream(uart_service)
			print('received', packet)
			if isinstance(packet, ColorPacket):
				neopixel.fill(packet.color)
		if key_matrix.events.get_into(key_event):
			button_packet._button = ord(0x80 + key_event.key_number)
			button_packet._pressed = key_event.pressed
			uart_service.write(button_packet.to_bytes())
		# todo uart_service.write(joystick_packet.to_bytes())
		# todo uart_service.write(proximity_packet.to_bytes())
		# todo uart_service.write(color_packet.to_bytes())
		# todo uart_service.write(magnetometer_packet.to_bytes())
		# todo uart_service.write(accelerometer_packet.to_bytes())
		# todo uart_service.write(gyro_packet.to_bytes())
	else:
		for ad in ble.start_scan(ProvideServicesAdvertisement):
			ad_address = hexlify(ad.address.address_bytes)
			if ad_address == main_address:
				try:
					connection = ble.connect(ad.address)
					if not connection.paired:
						connection.pair()
					ble.stop_scan()
				except Exception as e:
					print('\n'.join(traceback.format_exception(e)))
		print('connected to main')
