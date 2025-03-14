from .packets import AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket
from .ring_buffer import RingBuffer
from . import tasks
from .joystick import Interval, JoyStick
import digitalio

def DigitalOut(pin):
	dout = digitalio.DigitalInOut(pin)
	dout.direction = digitalio.Direction.OUTPUT
	return dout

def DigitalIn(pin, *, pull_down=False):
	din = digitalio.DigitalInOut(board.SWITCH)
	din.direction = digitalio.Direction.INPUT
	din.pull = digitalio.Pull.DOWN if pull_down else digitalio.Pull.UP
	return din
