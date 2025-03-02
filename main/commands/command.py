# https://webserial.io/
# command device
# print(ARGS)
#
# writes 'print(ARGS)' to /commands/device.py and updates SCRIPT_CACHE

@read_block
def _(block):
	filename = '/commands/' + ARGS[1] + '.py'
	SCRIPT_CACHE[filename] = compile(block, filename, 'exec')
	open(filename, 'w').write(block)
	print('wrote ' + str(len(block)) + ' bytes to ' + filename)

print('writing /commands/' + ARGS[1] + '.py')
