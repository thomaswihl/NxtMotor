
ARDUINO_DIR  = /home/thomas/bin/arduino-1.0.6/

TARGET       = NxtMotor
ARDUINO_LIBS =  

BOARD_TAG    = pro5v328
ARDUINO_PORT = /dev/ttyUSB1

CPPFLAGS     = -O3 -I$(ARDUINO_DIR)/hardware/tools/avr/lib/avr/include


include $(ARDUINO_DIR)/Arduino.mk


