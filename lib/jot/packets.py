from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.color_packet import ColorPacket
from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
from adafruit_bluefruit_connect.magnetometer_packet import MagnetometerPacket
from adafruit_bluefruit_connect.gyro_packet import GyroPacket

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
		self.x = x
		self.y = y

	def to_bytes(self) -> bytes:
		partial_packet = struct.pack(
				self._FMT_CONSTRUCT,
				self._TYPE_HEADER,
				self.x,
				self.y,
		)
		return self.add_checksum(partial_packet)

	def __repr__(self):
		return f'JoystickPacket({self.x}, {self.y})'
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
