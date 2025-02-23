# https://webserial.io/
# switch device
# print(ARGS)
#
# writes 'print(ARGS)' to /switches/device.py and updates SCRIPT_CACHE

@read_block
def _(block):
	filename = '/switches/' + ARGS[1] + '.py'
	SCRIPT_CACHE[filename] = compile(block, filename, 'exec')
	open(filename, 'w').write(block)
	print('wrote ' + str(len(block)) + ' bytes to ' + filename)

print('writing /switches/' + ARGS[1] + '.py')
