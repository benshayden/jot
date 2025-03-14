from . import tasks

class JoyMouse:
  # joystick mouse
	def __init__(self, joystick, getmouse, ms=50, speed=0.1):
		self.joystick = joystick
    self.getmouse = getmouse
		self.speed = speed
		self._x = 0.0
		self._y = 0.0
		self.task = tasks.create(ms=ms)(self.loop)
	
	def loop(self, now):
		dx, self._x = self._buf(self._x + self.joystick.x)
		dy, self._y = self._buf(self._y + self.joystick.y)
		self.getmouse().move(dx, dy)
	
	def _buf(self, n):
		whole, frac = divmod(self.speed * n * 127.0, 1.0)
		return min(127, max(-127, int(whole))) & 0xff, (frac / 127.0) / self.speed
