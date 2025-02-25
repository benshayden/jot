if event.pressed:
	if log[0].switch == ('record',):
		@addloop('macro')
		def _(f=open('/switches/macros/' + ARGS[1] + '.py', 'w')):
			if event is None:
				return
			if ('record',) in pressed_switches:
				f.close()
				removeloop('macro')
				return
			f.write('event = {event};switch({switch})\n'.format(
				event=('PRESS' if log[0].pressed else 'RELEASE'),
				switch=log[0].switch))
	else:
		switch(('macros/' + ARGS[1],))
