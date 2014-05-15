MODULE_COMMAND = airdog
SRCS = airdog.c \
	   i2c_helper.cpp \
	   i2c_controller.cpp \
	   i2c_display_controller.cpp

CFLAGS += -Wno-unknown-pragmas -Wno-packed
