if SwitchEvent.current.pressed:
	set_layer(ARGS[1])
	gamepad.task.enabled = (ARGS[1] == 'gamepad')
	joymouse.task.enabled = not gamepad.task.enabled
