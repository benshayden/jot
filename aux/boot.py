import storage
import board
import supervisor
from neopixel import NeoPixel

neopixel = NeoPixel(board.NEOPIXEL, 1)
neopixel.fill((10, 0, 0))

mnt = storage.getmount('/')
label = '_JOT'
if mnt.label != label:
	storage.remount('/', readonly=False)
	mnt = storage.getmount('/')
	mnt.label = label
	storage.remount('/', readonly=True)

# todo https://pid.codes/
# 0x62='b', 0x68='h'
supervisor.set_usb_identification(manufacturer='bsh', product='_jot', vid=0x1209, pid=0x6268)

neopixel.fill((0, 0, 0))
