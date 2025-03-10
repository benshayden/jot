import supervisor
from . import RingBuffer

class SwitchEvent:
	current = None
	sources = []
	
	def __init__(self, pressed=False, timestamp=0, key_number=0, args=(), source=None):
		self.pressed = pressed
		self.timestamp = timestamp
		self.key_number = key_number
		self.args = args
		self.source = source

	def __repr__(self):
		return f'SwitchEvent({self.pressed}, {self.timestamp}, {self.key_number}, {self.args}, {self.source})'
	
	@staticmethod
	def source(obj, count):
		SwitchEvent.sources.append((id(obj), count))

	@staticmethod
	def dispatch(press, index, source):
		for srcid, count in SwitchEvent.sources:
			if srcid != id(source):
				index += count
		if index < 0 or index > len(SwitchEvent.keymap):
			print(f'invalid switch {index}')
			return
		SwitchEvent.current = SwitchEvent.instance
		SwitchEvent.current.key_number = index
		SwitchEvent.current.source = source
		SwitchEvent.current.pressed = press
		SwitchEvent.current.args = SwitchEvent.keymap[index]
		SwitchEvent.current.timestamp = supervisor.ticks_ms()
		print(f'running {SwitchEvent.current.args}')
		if press:
			SwitchEvent.pressed_args.add(SwitchEvent.current.args)
			SwitchEvent.histogram[SwitchEvent.current.args] = SwitchEvent.histogram.get(SwitchEvent.current.args, 0) + 1
			SwitchEvent.histogram_count += 1
		elif SwitchEvent.current.args in SwitchEvent.pressed_args:
			SwitchEvent.pressed_args.remove(SwitchEvent.current.args)
		SwitchEvent.run(SwitchEvent.current.args)
		SwitchEvent.log(SwitchEvent.current)
	
	class Log(RingBuffer):
		def __init__(self, capacity=30):
			super().__init__(capacity=capacity, init=lambda: SwitchEvent())
	
		def _update(self, e):
			self._events[self._i].pressed = e.pressed
			self._events[self._i].timestamp = e.timestamp
			self._events[self._i].key_number = e.key_number
			self._events[self._i].args = e.args
			self._events[self._i].source = e.source
	
	pressed_args = set()

	histogram = {}
	histogram_count = 0

	@staticmethod
	def flush_histogram():
		if SwitchEvent.histogram_count < 100:
			return
		with open('/hist.txt', 'w') as f:
			for s, n in sorted(SwitchEvent.histogram.items(), key=lambda item: -item[1]):
				f.write(f'{n},{' '.join(s)}\n')
		SwitchEvent.histogram_count = 0

SwitchEvent.instance = SwitchEvent()
SwitchEvent.log = SwitchEvent.Log()
PRESS = SwitchEvent(True)
RELEASE = SwitchEvent(False)

try:
	for line in open('/hist.txt'):
		line = line.strip()
		comma = line.find(',')
		if comma < 1 or not line[:comma].isdigit() or comma >= len(line):
			continue
		SwitchEvent.histogram[line[comma + 1:]] = int(line[:comma])
except Exception as e:
	print('\n'.join(traceback.format_exception(e)))
