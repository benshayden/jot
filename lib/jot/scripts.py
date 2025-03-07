import supervisor
import sys
import traceback
from jot import tasks

__cache = {}
__script_locals = {}
def run_script(args, directory, namespace):
  if namespace is None:
    namespace = globals()
	namespace['ARGS'] = args
	if not args or not args[0]:
		return
	filename = f'/{directory}/{args[0]}.py'
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

class CommandLineInterface:
	def __init__(self, directory, namespace):
		self._directory = directory
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
				run_script(serial_bytes.strip().split(), directory=self._directory, namespace=self._namespace)
