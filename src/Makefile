c:
	idf.py build
f:
	PORT=$$(ls /dev/cu.usb*);\
	echo $$PORT;\
	idf.py -p $$PORT flash
m:
	idf.py monitor
data:
	PORT=$$(ls /dev/cu.usb*);\
	echo $$PORT;\
	esptool.py --chip esp32s3 --port $$PORT --baud 460800 write_flash 0x148000 SALAMAN.BIN