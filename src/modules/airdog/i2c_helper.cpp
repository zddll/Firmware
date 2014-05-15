#include "i2c_helper.h"
#include "i2c_controller.h"
#include "i2c_display_controller.h"

#include <nuttx/config.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h> 

#include <drivers/device/i2c.h>
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

#include <uORB/topics/i2c_button_status.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>

#define LISTENER_ADDR 0x20 /**< I2C adress of our button i2c controller */
#define DISPLAY_ADDR 0x38 /**< I2C adress of our display i2c controller */ 


namespace
{
I2C_CONTROLLER *_listener;
I2C_DISPLAY_CONTROLLER *_display;
}

void start_listener(int adress)
{
    int i2cdevice = -1;
	int main_adr = LISTENER_ADDR;
    int display_addr = DISPLAY_ADDR; /* 7bit */

    if (_listener != nullptr)
        errx(1, "already started");

    if (i2cdevice == -1) {
        // try the external bus first
        i2cdevice = PX4_I2C_BUS_EXPANSION;
        _listener = new I2C_CONTROLLER(i2cdevice, main_adr);
        int ret = _listener->init();
        if (_listener != nullptr && OK != ret) {
            delete _listener;
            _listener = nullptr;
            warnx("created but not inited with code: %d", ret);
        }
    }

        _display = new I2C_DISPLAY_CONTROLLER(i2cdevice, display_addr);
        int ret = _display->init();
        if (_display != nullptr && OK != ret) {
            delete _display;
            _display = nullptr;
            warnx("created but not inited with code: %d", ret);
        }

    if (_listener) {
        _listener->start_listening();
    }

    if (_display) {
        _display->clear_display();
    }
}

void stop_listener()
{
    delete _listener;
    delete _display;
    _listener = nullptr;
    _display = nullptr;
}

void set_indicators_state(led_state_t state)
{
    _listener->set_indicators_state(state);
}

void set_symbols(symbol_t first, symbol_t second, symbol_t third)
{
    _display->set_symbols(first, second, third);
}
