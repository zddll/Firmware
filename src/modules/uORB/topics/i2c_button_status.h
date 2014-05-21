/**
 * @file airdog_path_log.h
 * Definition of the logger trigger command uORB topic.
 */

#ifndef TOPIC_I2C_BUTTON_STATUS_H_
#define TOPIC_I2C_BUTTON_STATUS_H_

#include <stdint.h>
#include "../uORB.h"


#include <stdint.h>

struct i2c_button_s {
    int pin; 
    int register_pin;
	bool button_pressed;
	bool long_press;
	uint64_t time_pressed;
	bool started;
};

/* register this as object request broker structure */
ORB_DECLARE(i2c_button_status);

#endif
