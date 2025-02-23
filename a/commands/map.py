# https://webserial.io/
# Usage: map layer number switch args
# Example: map default 37 kb Z
# Updates /layers/default.txt such that line 37 contains 'kb Z'
# Line numbers are 1-based but python arrays are 0-based.

layer = get_layer(ARGS[1])
layer[int(ARGS[2]) - 1] = ARGS[3:]
open('/layers/' + ARGS[1] + '.txt', 'w').write('\n'.join(' '.join(line) for line in layer))
