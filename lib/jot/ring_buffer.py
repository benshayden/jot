# Usage:
# buf = RingBuffer()
# buf(37)
# len(buf) == 1
# buf[0] # most recent event
# buf[1] # event before last
# for elem in buf: print(elem)
# buf.capacity = 10

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
