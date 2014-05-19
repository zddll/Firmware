#ifndef _I2C_CONTROLLER_H
#define _I2C_CONTROLLER_H

#include <drivers/device/i2c.h>
#include <nuttx/wqueue.h>
#include <uORB/topics/i2c_button_status.h> 

class __EXPORT I2C_CONTROLLER : public device::I2C
{
public:
	I2C_CONTROLLER(int bus, int addr);
	virtual ~I2C_CONTROLLER();

	virtual int		init();
	virtual int		probe();
    void            start_listening();
    void            set_green_led_on(bool set_on);
    void            set_red_led_on(bool set_on);

private:
    bool			_running;
    bool			_should_run;
    bool            _led_update_in_progress;
	work_s			_work;
    int			    _listening_interval;
    orb_advert_t    _cmd_pub;

    bool            _is_red_led_on;
    bool            _is_green_led_on;

    struct i2c_button_s *_buttons;

    static void		listener_trampoline(void *arg);
    void			listen();
    int             get_buttons_state_from_r1();
    int             get_buttons_state_from_r2();
    void            init_buttons();
    int             init_led();
    void            check_button(struct i2c_button_s *button, int values);
    void            button_pressed(struct i2c_button_s *button, bool long_press);
};

#endif
