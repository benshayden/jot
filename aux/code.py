from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket
from adafruit_bluefruit_connect.color_packet import ColorPacket
import board
import neopixel
from adafruit_debouncer import Debouncer
from digitalio import DigitalInOut, Direction, Pull
from binascii import hexlify, unhexlify
from time import sleep
from adafruit_ble.services.standard.device_info import DeviceInfoService, BatteryService

def reprButtonPacket(self):
	return 'ButtonPacket({b}, {p})'.format(b=self.button, p=self.pressed)
ButtonPacket.__repr__ = reprButtonPacket

def saferead(filename):
	try:
		return open(filename).read()
	except Exception as e:
		return None

device_info_service = DeviceInfoService(
	software_revision='2025-03-02',
	manufacturer='bsh',
)
battery_service = BatteryService()
neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
ble = BLERadio()
print('my address', hexlify(ble.address_bytes))
main_address = saferead('mainaddr.txt') or b'0d5634837ffe'
ble.name = '_jot'
uart_service = UARTService()
advertisement = ProvideServicesAdvertisement(uart_service, device_info_service, battery_service)
advertisement.complete_name = ble.name
__switch = DigitalInOut(board.SWITCH)
__switch.direction = Direction.INPUT
__switch.pull = Pull.UP
switch = Debouncer(__switch)
switchprev = switch.value
client = None
print('advertising', hexlify(ble.address_bytes))
ble.start_advertising(advertisement)

while True:
	print('server', ble.connected, ble.advertising)
	if ble.connected:
		# todo confirm client address is main_address, pair
		ble.stop_advertising()
		switch.update()
		if switch.value != switchprev:
			print('sending switch', switch.value)
			uart_service.write(ButtonPacket('1', switch.value).to_bytes())
			switchprev = switch.value
			print('sent switch', switch.value)
		if uart_service.in_waiting:
			packet = Packet.from_stream(uart_service)
			print('received', packet)
			if isinstance(packet, ButtonPacket):
				neopixel.fill((0, 0, 0) if packet.pressed else (10, 10, 10))
	elif not ble.advertising:
		print('advertising', hexlify(ble.address_bytes))
		ble.start_advertising(advertisement)
	sleep(0.1)
