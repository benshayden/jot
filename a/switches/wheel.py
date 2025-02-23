if event.pressed:
	addloop('wheel', lambda wheel=int(ARGS[1]), ms=int(ARGS[2]): (mouse.move(wheel=wheel) if everyms(ms, 'wheel') else None))
else:
	removeloop('wheel')
