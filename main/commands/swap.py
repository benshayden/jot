# Usage: swap layer number number
# Example: swap default 42 57
# updates /layers/default.txt such that lines 42 and 57 are swapped
# Line numbers are 1-based but python arrays are 0-based.

layer = get_layer(ARGS[1])
layer[int(ARGS[2]) - 1], layer[int(ARGS[3]) - 1] = layer[int(ARGS[3]) - 1], layer[int(ARGS[2]) - 1]
open('/layers/' + ARGS[1] + '.txt', 'w').write('\n'.join(' '.join(line for line in layer)))
