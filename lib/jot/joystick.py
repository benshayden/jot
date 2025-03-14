import collections
from . import tasks, RingBuffer

Interval = collections.namedtuple('Interval', ('min', 'max'))

class JoyStick:
	def __init__(self, xpin, xneg, xpos, ypin, yneg, ypos, window_ms=50):
		self._xpin = xpin
		self._xneg = xneg
		self._xpos = xpos
		self._ypin = ypin
		self._yneg = yneg
		self._ypos = ypos
		self._x = RingBuffer(capacity=window_ms, init=0.0)
		self._y = RingBuffer(capacity=window_ms, init=0.0)
		self.task = tasks.create(ms=1)(self.loop)
		self._x_cache = None
		self._y_cache = None

	@property
	def x(self) -> float:
		if self._x_cache is None:
			self._x_cache = sum(self._x) / len(self._x) if len(self._x) else 0.0
		return self._x_cache

	@property
	def y(self) -> float:
		if self._y_cache is None:
			self._y_cache = sum(self._y) / len(self._y) if len(self._y) else 0.0
		return self._y_cache

	def _norm(self, val, neg, pos):
		if neg.min <= val <= neg.max:
			return (val - neg.max) / (neg.max - neg.min)
		elif pos.min <= val <= pos.max:
			return (val - pos.min) / (pos.max - pos.min)
		return 0.0
	
	def loop(self, now):
		self._x_cache = self._y_cache = None
		self._x(self._norm(self._xpin.value, self._xneg, self._xpos))
		self._y(self._norm(self._ypin.value, self._yneg, self._ypos))
