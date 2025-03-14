# Usage:
# @tasks.create(ms=16)
# def foo(now): print('60fps', now)
# foo.enabled = False
# foo.ms = 1000
# tasks.start()
# tasks.stop()

import supervisor
import traceback

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

_tasks = []

class Task:
	def __init__(self, cb, ms, priority):
		self.enabled = True
		self._cb = cb
		self.ms = ms
		self._tick = 0
		_tasks.append(self)
		self.priority = priority
	
	@property
	def priority(self):
		return self._priority
	
	@priority.setter
	def priority(self, p):
		self._priority = p
		_tasks.sort(key=lambda t:t._priority)
	
	def __call__(self, now):
		if not self.enabled or ticks_diff(now, self._tick) < self.ms:
			return
		self._tick = now
		try:
			self._cb(now)
		except Exception as e:
			print('\n'.join(traceback.format_exception(e)))
			self.enabled = False

def create(*, ms=0, priority=0x80):
	return lambda cb: Task(cb, ms, priority)

running = False

def start():
	global running
	running = True
	while running:
		now = supervisor.ticks_ms()
		for task in _tasks:
			task(now)

def stop():
	global running
	running = False
