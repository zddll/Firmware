#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/topics/send_command.h>
#include <uORB/topics/debug_key_value.h>

#include <drivers/drv_gpio.h>

struct gpio_button_s {
	char name;
	int pin;
	bool button_state;
};

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

__EXPORT int airdog_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: airdog {start|stop|status} [-p <additional params>]\n\n");
}

int airdog_main(int argc, char *argv[]) 
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("airdog already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("daemon",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 4096,
					 px4_daemon_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int px4_daemon_thread_main(int argc, char *argv[]) {

	warnx("[daemon] starting\n");

	thread_running = true;


	struct send_command_s cmd;
		cmd.param1 = 0;
		cmd.param2 = 0;
		cmd.param3 = 0;
		cmd.param4 = 0;
		cmd.param5 = 0;
		cmd.param6 = 0;
		cmd.param7 = 0;
		cmd.command = SEND_CMD_NAV_LOITER_UNLIM;
		cmd.confirmation =  1;


	orb_advert_t pub_dbg = orb_advertise(ORB_ID(send_command), &cmd);
	

	/* configure the GPIO */
	
	struct gpio_button_s button1;
	struct gpio_button_s button2;

	button1.pin = 0;
	button2.pin = 1;
	button1.button_state = false;
	button2.button_state = false;

	int inputs = 3; //pin 1+2
	int fd = open(PX4FMU_DEVICE_PATH, 0);
	ioctl(fd, GPIO_SET_INPUT, inputs);

	while (!thread_should_exit) {

		/* check the GPIO */
		uint32_t gpio_values;
		ioctl(fd, GPIO_GET, &gpio_values);
		/*warnx("values: %u", gpio_values);*/

		if (!(gpio_values & (1 << button1.pin))) {
			if (button1.button_state == false){
				warnx("button 1 pressed");
				orb_publish(ORB_ID(send_command), pub_dbg, &cmd);
				button1.button_state = true;
			}
		} else {
			if (button1.button_state == true){
				warnx("button 1 let go");
				button1.button_state = false;
			}
		}

		if (!(gpio_values & (1 << button2.pin))) {
			if (button2.button_state == false){
				warnx("button 2 pressed");
				button2.button_state = true;
			}
		} else {
			if (button2.button_state == true){
				warnx("button 2 let go");
				button2.button_state = false;
			}
		}
		sleep(1);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}