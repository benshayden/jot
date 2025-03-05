import supervisor
import sys
import traceback
from jot import tasks

__cache = {}
__script_locals = {}
def run_switch(args, _dir='switches', namespace=None):
  if namespace is None:
    namespace = globals()
	namespace['ARGS'] = args
	if not args or not args[0]:
		return
	filename = f'/{_dir}/{args[0]}.py'
	if filename not in __cache:
		try:
			__cache[filename] = compile(open(filename).read(), filename, 'exec')
		except Exception as e:
			print(f'exception loading {filename}')
			print('\n'.join(traceback.format_exception(e)))
	if filename not in __cache:
		return
	if filename not in __script_locals:
		__script_locals[filename] = {}
	try:
		exec(__cache[filename], namespace, __script_locals[filename])
	except Exception as e:
		print('\n'.join(traceback.format_exception(e)))

def run_command(args, namespace=None):
	run_switch(args, _dir='commands', namespace=namespace)

class CommandLineInterface:
	def __init__(self, namespace=None):
    self._namespace = namespace
		self._block = ''
		self._block_callback = None
    tasks.add(name='cli', ms=100)(self.loop)
	
	def read_block(self, callback):
		self._block_callback = callback

	def loop(self):
		# If a command is entered on Serial, exec it.
		# https://webserial.io/
		if supervisor.runtime.serial_bytes_available:
			serial_bytes = sys.stdin.read(supervisor.runtime.serial_bytes_available)
			if self._block_callback:
				self._block += serial_bytes
				if self._block.endswith('\n\n') or self._block.endswith('\r\n\r\n'):
					try:
						self._block_callback(self._block)
					except Exception as e:
						print('\n'.join(traceback.format_exception(e)))
					self._block = ''
					self._block_callback = None
			else:
				run_command(serial_bytes.strip().split(), namespace=self._namespace)
