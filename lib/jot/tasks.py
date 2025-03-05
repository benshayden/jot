# Usage:
# @tasker.add('demo', 16)
# def _(now): print('60fps', now)
# tasker.remove('demo')
# @tasker.aor(__file__, pressed, 16)
# def _(now): print('60fps while pressed', now)
# tasker.start()
# tasker.stop()

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

_tasks = {}

class Task:
	def __init__(self, cb, ms):
		self._cb = cb
		self._ms = ms
		self._tick = 0

	def __call__(self, now):
		if ticks_diff(now, self._tick) < self._ms:
			return True
		self._tick = now
		try:
			self._cb(now)
			return True
		except Exception as e:
			print('\n'.join(traceback.format_exception(e)))
			return False

def add(*, name=None, ms=0):
	def deco(cb):
		_tasks[name or cb.__name__] = Task(cb, ms)
	return deco

def remove(name):
	del _tasks[name]

def aor(do_add, *, name=None, ms=0):
	# add or remove
	if do_add:
		return add(name=name, ms=ms)
	remove(name)
	return lambda _: None

def _run():
	now = supervisor.ticks_ms()
	bad = []
	for name in _tasks:
		if not _tasks[name](now):
			bad.append(name)
	for name in bad:
		remove(name)

running = False

def start():
	global running
	running = True
	while running:
		_run()

def stop():
	global running
	running = False
