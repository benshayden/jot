import struct
from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.color_packet import ColorPacket
from adafruit_bluefruit_connect.accelerometer_packet import AccelerometerPacket
from adafruit_bluefruit_connect.magnetometer_packet import MagnetometerPacket
from adafruit_bluefruit_connect.gyro_packet import GyroPacket

def reprButtonPacket(self):
	return f'ButtonPacket({self.button}, {self.pressed})'
ButtonPacket.__repr__ = reprButtonPacket

def setButtonPacketButton(self, button):
	self._button = button
ButtonPacket.button = ButtonPacket.button.setter(setButtonPacketButton)

def setButtonPacketPressed(self, pressed):
	self._pressed = pressed
ButtonPacket.pressed = ButtonPacket.pressed.setter(setButtonPacketPressed)

def getButtonPacketIndex(self):
	return ord(self.button) - 65
ButtonPacket.index = property(getButtonPacketIndex)

def setButtonPacketIndex(self, index):
	self._button = chr(65 + index)
ButtonPacket.index = ButtonPacket.index.setter(setButtonPacketIndex)

def reprAccelerometerPacket(self):
	return f'AccelerometerPacket({self._x}, {self._y}, {self._z})'
AccelerometerPacket.__repr__ = reprAccelerometerPacket

def reprMagnetometerPacket(self):
	return f'MagnetometerPacket({self._x}, {self._y}, {self._z})'
MagnetometerPacket.__repr__ = reprMagnetometerPacket

def reprColorPacket(self):
	return f'ColorPacket({self.color})'
ColorPacket.__repr__ = reprColorPacket

def setColorPacketColor(self, color):
	self._color = color[0] & 0xff, color[1] & 0xff, color[2] & 0xff
ColorPacket.color = ColorPacket.color.setter(setColorPacketColor)

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
		return self.add_checksum(struct.pack(self._FMT_CONSTRUCT, self._TYPE_HEADER, self.x, self.y))

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
		return self.add_checksum(struct.pack(self._FMT_CONSTRUCT, self._TYPE_HEADER, self.proximity))

	def __repr__(self):
		return f'ProximityPacket({self.proximity})'
ProximityPacket.register_packet_type()

class FeatureFlagPacket(Packet):
	_FMT_PARSE: str = '<xxssx'
	PACKET_LENGTH: int = struct.calcsize(_FMT_PARSE)
	# _FMT_CONSTRUCT doesn't include the trailing checksum byte.
	_FMT_CONSTRUCT: str = '<2sss'
	_TYPE_HEADER: bytes = b'!F'
	
	ACCELEROMETER = 'A'
	COLOR = 'C'
	GYRO = 'G'
	JOYSTICK = 'J'
	MAGNETOMETER = 'M'
	PROXIMITY = 'P'

	def __init__(self, feature: str, enabled: bool):
		self.feature = feature
		self.enabled = enabled

	def to_bytes(self) -> bytes:
		return self.add_checksum(struct.pack(self._FMT_CONSTRUCT, self._TYPE_HEADER, self.feature, self.enabled))
	
	def __repr__(self):
		return f'FeatureFlagPacket({self.feature}, {self.enabled})'
FeatureFlagPacket.register_packet_type()
