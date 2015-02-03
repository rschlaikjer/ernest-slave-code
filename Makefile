BOARD_TAG = micro
ARDUINO_LIBS =

CFLAGS:=
CXXFLAGS:=
unexport CFLAGS
unexport CXXFLAGS

MONITOR_PORT := /dev/ttyACM0

include /usr/share/arduino/Arduino.mk
