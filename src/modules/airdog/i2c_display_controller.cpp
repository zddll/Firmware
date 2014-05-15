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

I2C_DISPLAY_CONTROLLER::I2C_DISPLAY_CONTROLLER(int bus, int addr) :
    I2C("display", "/dev/display", bus, addr, 100000)
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
    uint8_t requests[1] = {0x4D};

    int ret = transfer(&requests[0], sizeof(requests), nullptr, 0);
    //ret = transfer(nullptr, 0, response, sizeof(response)); 
    warnx("I2C_DISPLAY_CONTROLLER::probe() Result code is: %d, response: %x", ret, response[0]);

	return ret;
}

int
I2C_DISPLAY_CONTROLLER::set_symbols(uint8_t first, uint8_t second, uint8_t third)
{
    uint8_t requests[4] = {0x00, first, second, third};
    return transfer(requests, sizeof(requests), nullptr, 0);
}

int
I2C_DISPLAY_CONTROLLER::clear_display()
{
    return set_symbols(SYMBOL_EMPTY, SYMBOL_EMPTY, SYMBOL_EMPTY);
}
