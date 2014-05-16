#include "i2c_controller.h"

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

#include <uORB/topics/i2c_button_status.h>

#include "i2c_helper.h"

#define I2C_BUTTON_COUNT 9
#define CONTROLLER_PATH "/dev/buttons"
#define CONTROLLER_NAME "buttons"
#define CONTROLLER_SETUP_COMMAND 0x00
#define LED_SETUP_PORT_NUMBER 0x07
#define LED_SETUP_CMD 0x3F
#define LED_WRITE_PORT_NUMBER 0x03

I2C_CONTROLLER::I2C_CONTROLLER(int bus, int addr) :
	I2C(CONTROLLER_NAME, CONTROLLER_PATH, bus, addr, DEVICE_FREQUENCY),
    _running(false),
    _should_run(false),
    _listening_interval(USEC2TICK(DEVICE_FREQUENCY))
{
	memset(&_work, 0, sizeof(_work));
}

I2C_CONTROLLER::~I2C_CONTROLLER()
{
    delete _buttons;
}

int
I2C_CONTROLLER::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

    init_buttons();

    ret = init_led();
    if (ret != OK) {
		return ret;
	}

    _led_update_in_progress = false;

	return OK;
}

int
I2C_CONTROLLER::probe()
{
    uint8_t response[1] = {0};
    uint8_t requests[1] = {CONTROLLER_SETUP_COMMAND};

    int ret = transfer(&requests[0], sizeof(requests), nullptr, 0);
    ret = transfer(nullptr, 0, response, sizeof(response));
	return ret;
}

void
I2C_CONTROLLER::init_buttons()
{
    _buttons = new i2c_button_s[I2C_BUTTON_COUNT];
    int pin_id = 0;
    for (int i = 0; i < I2C_BUTTON_COUNT; i++) {
        if (pin_id == 8) {
            pin_id = 0;
        }
        _buttons[i] = { i, pin_id, false, false, 0};
        pin_id++;
    }

    _cmd_pub = -1;
}

int
I2C_CONTROLLER::init_led()
{
    uint8_t requests[2] = {LED_SETUP_PORT_NUMBER, LED_SETUP_CMD};
    int ret = transfer(requests, sizeof(requests), nullptr, 0);
    return ret;
}

void
I2C_CONTROLLER::listener_trampoline(void *arg)
{
	I2C_CONTROLLER *rgbl = reinterpret_cast<I2C_CONTROLLER *>(arg);

	rgbl->listen();
}

void
I2C_CONTROLLER::listen()
{
    if (!_should_run) {
		_running = false;
		return;
	}

    if (!_led_update_in_progress) {
        int mask_r1 = get_buttons_state_from_r1();

        for (int i = 0; i < I2C_BUTTON_COUNT - 1; i++) {
            check_button(&_buttons[i], mask_r1);
        }
        int mask_r2 = get_buttons_state_from_r2();
        for (int i = I2C_BUTTON_COUNT - 1; i < I2C_BUTTON_COUNT; i++) {
            check_button(&_buttons[i], mask_r2);
        }
    }

    work_queue(LPWORK, &_work, (worker_t)&I2C_CONTROLLER::listener_trampoline, this, _listening_interval);
}

void
I2C_CONTROLLER::start_listening()
{
    _should_run = true;
    _running = true;
    work_queue(LPWORK, &_work, (worker_t)&I2C_CONTROLLER::listener_trampoline, this, _listening_interval);
}

int
I2C_CONTROLLER::get_buttons_state_from_r1()
{
    uint8_t response[1] = {0};
    // I do not know why it is 0x00, but it works
    uint8_t requests[1] = {0x00};

    int ret = transfer(&requests[0], sizeof(requests), nullptr, 0);
    ret = transfer(nullptr, 0, response, sizeof(response)); 

    if (ret == OK) {
        return response[0];
    }
    return -1;
}

int
I2C_CONTROLLER::get_buttons_state_from_r2()
{
    uint8_t response[1] = {0};
    // This is according to specs. Fishy.
    uint8_t requests[2] = {0x00, 0x41};

    int ret = transfer(requests, sizeof(requests), nullptr, 0);
    ret = transfer(nullptr, 0, response, sizeof(response)); 

    if (ret == OK) {
        return response[0];
    }
    return -1;
}

void
I2C_CONTROLLER::check_button(struct i2c_button_s *button, int gpio_values)
{
	if (!(gpio_values & (1 << button->register_pin))) {
		uint64_t now = hrt_absolute_time();
		float elapsed = (now - button->time_pressed) / 10000;

		if (button->button_pressed == false){
			button->button_pressed = true;
			button->time_pressed = now;
		} else if (button->button_pressed & !button->long_press & elapsed > 150) {
			warnx("long press button %d", button->pin + 1);
            button->long_press = true;
			button_pressed(button, true);
		}
	} else {
		if (button->button_pressed == true){
			if (!button->long_press)
			{
				warnx("short press button %d", button->pin + 1);
				button_pressed(button, false);
			}
			button->button_pressed = false;
			button->long_press = false;
		}
	}
};

void
I2C_CONTROLLER::button_pressed(struct i2c_button_s *button, bool long_press)
{
    if (_cmd_pub < 0) {
        _cmd_pub = orb_advertise(ORB_ID(i2c_button_status), button);
    } else {
        orb_publish(ORB_ID(i2c_button_status), _cmd_pub, button);
    }
}

void
I2C_CONTROLLER::set_indicators_state(uint8_t state)
{
    uint8_t response[1] = {0};

    uint8_t requests[2] = {LED_WRITE_PORT_NUMBER, state};
    int ret = transfer(requests, sizeof(requests), nullptr, 0);
}
