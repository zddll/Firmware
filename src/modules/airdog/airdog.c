#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>


#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/debug_key_value.h>

#include <drivers/drv_gpio.h>
#include <commander/px4_custom_mode.h>

enum REMOTE_BUTTONS {
	REMOTE_BUTTON_START_PAUSE=1,
	REMOTE_BUTTON_TAKEOFF_LAND=2,
};

enum REMOTE_BUTTON_STATE {
	PAUSE=1,
	START=2,
};

enum MAV_MODE_FLAG {
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, /* 0b00000001 Reserved for future use. | */
	MAV_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	MAV_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	MAV_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	MAV_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	MAV_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */
	MAV_MODE_FLAG_SAFETY_ARMED = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	MAV_MODE_FLAG_ENUM_END = 129, /*  | */
};

struct gpio_button_s {
	enum REMOTE_BUTTONS type;
	enum REMOTE_BUTTON_STATE state;
	int pin;
	bool button_pressed;
};

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static orb_advert_t cmd_pub = -1;

__EXPORT int airdog_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

bool send_remote_command(struct gpio_button_s *button);

void send_set_mode(uint8_t base_mode, uint8_t custom_main_mode);

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

bool send_remote_command(struct gpio_button_s *button)
{
	uint8_t base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	switch(button->type){
	case REMOTE_BUTTON_START_PAUSE: {

		if (button->state == START){
			send_set_mode(base_mode, PX4_CUSTOM_MAIN_MODE_EASY);
			button->state = PAUSE;

		} else if (button->state == PAUSE){
			send_set_mode(base_mode, PX4_CUSTOM_MAIN_MODE_FOLLOW);
			button->state = START;
		}

		break;
	}
	case REMOTE_BUTTON_TAKEOFF_LAND: {

		send_set_mode(base_mode, PX4_CUSTOM_MAIN_MODE_EASY);

		break;
	}
	}

	return true;
}

void send_set_mode(uint8_t base_mode, enum PX4_CUSTOM_MAIN_MODE custom_main_mode) {
	/* TODO this is very ugly, need to rewrite app to C++ and use class fields instead of static var */
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s state;
	orb_copy(ORB_ID(vehicle_status), state_sub, &state);

	/* fill command */
	cmd.command = VEHICLE_CMD_DO_SET_MODE;
	cmd.confirmation = false;
	cmd.param1 = base_mode;
	cmd.param2 = custom_main_mode;
	// TODO subscribe to vehicle_status topic and use values from it
	cmd.source_system = state.system_id;
	cmd.source_component = state.component_id;
	// TODO add parameters AD_VEH_SYSID, AD_VEH_COMP to set target id
	cmd.target_system = 1;
	cmd.target_component = 50;

	if (cmd_pub < 0) {
		cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);

	} else {
		orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);
	}
}

int px4_daemon_thread_main(int argc, char *argv[]) {

	warnx("[daemon] starting\n");

	thread_running = true;

	/* configure the GPIO */
	struct gpio_button_s button1;
	struct gpio_button_s button2;

	button1.pin = 0;
	button2.pin = 1;
	button1.type = REMOTE_BUTTON_START_PAUSE;
	button2.type = REMOTE_BUTTON_TAKEOFF_LAND;
	button1.button_pressed = false;
	button2.button_pressed = false;
	button1.state = PAUSE;

	int inputs = 3; //pin 1+2
	int fd = open(PX4FMU_DEVICE_PATH, 0);
	ioctl(fd, GPIO_SET_INPUT, inputs);

	cmd_pub = -1;

	while (!thread_should_exit) {
		/* check the GPIO */
		uint32_t gpio_values;
		ioctl(fd, GPIO_GET, &gpio_values);
		/*warnx("values: %u", gpio_values);*/

		if (!(gpio_values & (1 << button1.pin))) {
			if (button1.button_pressed == false){
				warnx("button 1 pressed");
				bool success = send_remote_command(&button1);
				button1.button_pressed = true;
			}
		} else {
			if (button1.button_pressed == true){
				warnx("button 1 let go");
				button1.button_pressed = false;
			}
		}

		if (!(gpio_values & (1 << button2.pin))) {
			if (button2.button_pressed == false){
				warnx("button 2 pressed");
				bool success = send_remote_command(&button2);
				button2.button_pressed = true;
			}
		} else {
			if (button2.button_pressed == true){
				warnx("button 2 let go");
				button2.button_pressed = false;
			}
		}
		sleep(1);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
