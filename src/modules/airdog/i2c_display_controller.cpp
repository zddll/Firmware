#include "i2c_display_controller.h"

#include <nuttx/config.h>
#include <nuttx/clock.h> 

#include <drivers/drv_hrt.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include "i2c_helper.h"

#define I2C_BUTTON_COUNT 9
#define DISPLAY_PATH "/dev/display"
#define DISPLAY_NAME "display"
#define SET_MODE_CMD 0x4D
#define WRITE_DATA_CMD 0x00

I2C_DISPLAY_CONTROLLER::I2C_DISPLAY_CONTROLLER(int bus, int addr) :
    I2C(DISPLAY_NAME, DISPLAY_PATH, bus, addr, DEVICE_FREQUENCY)
{
}

I2C_DISPLAY_CONTROLLER::~I2C_DISPLAY_CONTROLLER()
{
}

int
I2C_DISPLAY_CONTROLLER::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	return OK;
}

int
I2C_DISPLAY_CONTROLLER::probe()
{
    uint8_t response[1] = {0};
    uint8_t requests[1] = {SET_MODE_CMD};

    int ret = transfer(&requests[0], sizeof(requests), nullptr, 0);
	return ret;
}

int
I2C_DISPLAY_CONTROLLER::set_symbols(uint8_t first, uint8_t second, uint8_t third)
{
    uint8_t requests[4] = {WRITE_DATA_CMD, first, second, third};
    return transfer(requests, sizeof(requests), nullptr, 0);
}

int
I2C_DISPLAY_CONTROLLER::clear_display()
{
    return set_symbols(SYMBOL_EMPTY, SYMBOL_EMPTY, SYMBOL_EMPTY);
}
