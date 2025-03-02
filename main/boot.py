import board
import digitalio
import neopixel
import storage
import supervisor
import time
import usb_hid

neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

supervisor.set_usb_identification('bsh', 'jot')

cathode = digitalio.DigitalInOut(board.D2)
cathode.direction = digitalio.Direction.INPUT
cathode.pull = digitalio.Pull.DOWN
anode = digitalio.DigitalInOut(board.D5)
anode.direction = digitalio.Direction.OUTPUT
anode.value = True
neopixel.fill((100, 0, 0))
time.sleep(0.2)

readonly = cathode.value
neopixel.fill((0, 100 if readonly else 0, 0 if readonly else 100))
print('host can write CIRCUITPY but not jot' if readonly else 'jot can write CIRCUITPY but not host')
storage.remount('/', readonly=readonly)
time.sleep(0.5)
neopixel.fill((0, 0, 0))

gamepad = usb_hid.Device(
	report_descriptor=bytes((
		0x05, 0x01,  # Usage Page (Generic Desktop Ctrls)
		0x09, 0x05,  # Usage (Game Pad)
		0xA1, 0x01,  # Collection (Application)
		0x85, 0x04,  #   Report ID (4)
		0x05, 0x09,  #   Usage Page (Button)
		0x19, 0x01,  #   Usage Minimum (Button 1)
		0x29, 0x18,  #   Usage Maximum (Button 24)
		0x15, 0x00,  #   Logical Minimum (0)
		0x25, 0x01,  #   Logical Maximum (1)
		0x75, 0x01,  #   Report Size (1)
		0x95, 0x18,  #   Report Count (24)
		0x81, 0x02,  #   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x05, 0x01,  #   Usage Page (Generic Desktop Ctrls)
		0x15, 0x81,  #   Logical Minimum (-127)
		0x25, 0x7F,  #   Logical Maximum (127)
		0x09, 0x30,  #   Usage (X)
		0x09, 0x31,  #   Usage (Y)
		0x09, 0x32,  #   Usage (Z)
		0x09, 0x35,  #   Usage (Rz)
		0x75, 0x08,  #   Report Size (8)
		0x95, 0x04,  #   Report Count (4)
		0x81, 0x02,  #   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0,        # End Collection
	)),
	usage_page=0x01,           # Generic Desktop Control
	usage=0x05,                # Gamepad
	report_ids=(4,),           # Descriptor uses report ID 4.
	in_report_lengths=(7,),    # This gamepad sends 7 bytes in its report.
	out_report_lengths=(0,),   # It does not receive any reports.
)

usb_hid.enable((
	usb_hid.Device.KEYBOARD,
	usb_hid.Device.MOUSE,
	usb_hid.Device.CONSUMER_CONTROL,
	gamepad,
))
