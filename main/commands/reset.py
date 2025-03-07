# hard reset
if len(ARGS) > 1 and hasattr(microcontroller.RunMode, ARGS[1]):
	# https://docs.circuitpython.org/en/latest/shared-bindings/microcontroller/#microcontroller.RunMode
	microcontroller.on_next_reset(microcontroller.RunMode[ARGS[1]])
microcontroller.reset()
