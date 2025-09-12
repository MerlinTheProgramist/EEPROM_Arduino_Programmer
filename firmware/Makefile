FQBN=--fqbn esp32:esp32:esp32
PORT=/dev/ttyUSB0
BAUD=115200

build/firmware.ino.bin: firmware.ino
	arduino-cli compile ${FQBN} --build-path ./build

build: build/firmware.ino.bin ; 
	
upload: build
	arduino-cli upload --port ${PORT} ${FQBN} --build-path ./build 

monitor:
	arduino-cli monitor ${FQBN} --config 115200 --port ${PORT}
