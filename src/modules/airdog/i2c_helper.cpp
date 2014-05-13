#include "i2c_helper.h"

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <nuttx/wqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>

#define BUTTON_LISTENER_ONTIME 120
#define BUTTON_LISTENER_OFFTIME 120

#define LISTENER_ADDR			    0x40	/**< I2C adress of TCA62724FMG */

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */

extern "C" void start_listener(int adress);
extern "C" void stop_listener(void);

class BUTTON_LISTENER : public device::I2C
{
public:
	BUTTON_LISTENER(int bus, int addr);
	virtual ~BUTTON_LISTENER();


	virtual int		init();
	virtual int		probe();

private:
	work_s			_work;
};

/* for now, we only support one BUTTON_LISTENER */
namespace
{
BUTTON_LISTENER *_listener;
}


BUTTON_LISTENER::BUTTON_LISTENER(int bus, int addr) :
	I2C("buttons", "/dev/buttons", bus, addr, 100000)
{
	memset(&_work, 0, sizeof(_work));
}

BUTTON_LISTENER::~BUTTON_LISTENER()
{
}

int
BUTTON_LISTENER::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	return OK;
}

int
BUTTON_LISTENER::probe()
{
    uint8_t response[1] = {0};
    uint8_t requests[1] = {0x00};

    int ret = transfer(&requests[0], sizeof(requests), nullptr, 0);
    ret = transfer(nullptr, 0, response, sizeof(response)); 
    warnx("Result code is: %d, response: %x", ret, response[0]);

	return ret;
}

void start_listener(int adress)
{
    int i2cdevice = -1;
	int rgbledadr = adress; /* 7bit */

    if (_listener != nullptr)
        errx(1, "already started");

    if (i2cdevice == -1) {
        // try the external bus first
        i2cdevice = PX4_I2C_BUS_EXPANSION;
        _listener = new BUTTON_LISTENER(i2cdevice, rgbledadr);
        int ret = _listener->init();
        if (_listener != nullptr && OK != ret) {
            delete _listener;
            _listener = nullptr;
            warnx("created but not inited with code: %d", ret);
        }

        if (_listener == nullptr) {
            // fall back to default bus
            i2cdevice = PX4_I2C_BUS_LED;
            warnx("Falled back to default bus");
        }
    }

    if (_listener == nullptr) {
        _listener = new BUTTON_LISTENER(i2cdevice, rgbledadr);
        warnx("creating device on default bus.");
        if (_listener == nullptr)
            errx(1, "new failed");
        int ret = _listener->init();
        if (OK != ret) {
            delete _listener;
            _listener = nullptr;
            errx(1, "init failed, code: %s", ret);
        }
    }
}

void stop_listener()
{
    delete _listener;
    _listener = nullptr;
}
