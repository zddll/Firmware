#ifndef _I2C_DISPLAY_CONTROLLER_H
#define _I2C_DISPLAY_CONTROLLER_H

#include <drivers/device/i2c.h>

class __EXPORT I2C_DISPLAY_CONTROLLER : public device::I2C
{
public:
	I2C_DISPLAY_CONTROLLER(int bus, int addr);
	virtual ~I2C_DISPLAY_CONTROLLER();

	virtual int		init();
	virtual int		probe();
    int             set_symbols(uint8_t first, uint8_t second, uint8_t third);
    int             clear_display();
};

#endif 
