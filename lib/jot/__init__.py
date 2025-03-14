from .gamepad import Gamepad
from .packets import AccelerometerPacket, ButtonPacket, ColorPacket, GyroPacket, JoystickPacket, MagnetometerPacket, Packet, ProximityPacket
from .ring_buffer import RingBuffer
from .scripts import run_script, uncache_script, CommandLineInterface
from . import tasks
from .joystick import Interval, JoyStick
from .switch_event import SwitchEvent, PRESS, RELEASE
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
