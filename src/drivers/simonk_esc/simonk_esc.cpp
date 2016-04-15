/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_multirotor.generated.h>
#include <systemlib/param/param.h>
#include <dev_fs_lib_serial.h>
#include <v1.0/checksum.h>

#define MAX_LEN_DEV_PATH 32

namespace simonk_esc
{

#define MAX_MOTORS  4

volatile bool _task_should_exit = false; // flag indicating if simonk_esc task should exit
static char _device[MAX_LEN_DEV_PATH];
static bool _is_running = false;         // flag indicating if simonk_esc app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread

// subscriptions
int		_controls_sub;
int		_armed_sub;
int		_param_sub;
int 	_fd;
// filenames
// /dev/fs/ is mapped to /usr/share/data/adsp/
static const char *MIXER_FILENAME = "/dev/fs/quad_x.main.mix";


// publications
orb_advert_t        	_outputs_pub;
orb_advert_t 			_rc_pub;

// topic structures
actuator_controls_s     _controls;
actuator_armed_s        _armed;
parameter_update_s      _param_update;
actuator_outputs_s      _outputs;
input_rc_s 				_rc;

/** Print out the usage information */
void usage();

/** simonk_esc start */
void start(const char *device);

/** simonk_esc stop */
void stop();

/** task main trampoline function */
void	task_main_trampoline(int argc, char *argv[]);

/** simonk_esc thread primary entry point */
void task_main(int argc, char *argv[]);

/** mixer initialization */
MultirotorMixer *mixer;
int initialize_mixer(const char *mixer_filename);
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

int mixer_control_callback(uintptr_t handle,
			   uint8_t control_group,
			   uint8_t control_index,
			   float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;
	input = controls[control_group].control[control_index];

	return 0;
}


int initialize_mixer(const char *mixer_filename)
{
	mixer = nullptr;

	int mixer_initialized = -1;

	char buf[2048];
	unsigned int buflen = sizeof(buf);

	PX4_INFO("Initializing mixer from config file in %s", mixer_filename);

	int fd_load = ::open(mixer_filename, O_RDONLY);

	if (fd_load != -1) {
		int nRead = read(fd_load, buf, buflen);
		close(fd_load);

		if (nRead > 0) {
			mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);

			if (mixer != nullptr) {
				PX4_INFO("Successfully initialized mixer from config file");
				mixer_initialized = 0;

			} else {
				PX4_WARN("Unable to parse from mixer config file");
			}

		} else {
			PX4_WARN("Unable to read from mixer config file");
		}

	} else {
		PX4_WARN("Unable to open mixer config file");
	}

	// mixer file loading failed, fall back to default mixer configuration for
	// QUAD_X airframe
	if (mixer_initialized < 0) {
		float roll_scale = 1;
		float pitch_scale = 1;
		float yaw_scale = 1;
		float deadband = 0;

		mixer = new MultirotorMixer(mixer_control_callback, (uintptr_t)&_controls,
					    MultirotorGeometry::QUAD_X,
					    roll_scale, pitch_scale, yaw_scale, deadband);

		if (mixer == nullptr) {
			PX4_ERR("mixer initialization failed");
			mixer_initialized = -1;
			return mixer_initialized;
		}

		PX4_WARN("mixer config file not found, successfully initialized default quad x mixer");
		mixer_initialized = 0;
	}

	return mixer_initialized;
}

int simonk_initialize(const char *device)
{

	_fd = ::open(device, O_RDWR | O_NONBLOCK);

	if (_fd == -1) {
		PX4_ERR("Failed to open UART.");
		return -1;
	}

	struct dspal_serial_ioctl_data_rate rate;

	rate.bit_rate = DSPAL_SIO_BITRATE_38400;

	int ret = ioctl(_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);

	if (ret != 0) {
		PX4_ERR("Failed to set UART bitrate.");
		return -2;
	}

	return 0;
}

void task_main(int argc, char *argv[])
{

	_outputs_pub = nullptr;

	if (simonk_initialize(_device) < 0) {
		PX4_ERR("failed to initialize SimonkESC");

	} else {

		PX4_WARN("Initialized SimonkESC");
		// Subscribe for orb topics
		_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0)); // single group for now
		_armed_sub    = orb_subscribe(ORB_ID(actuator_armed));

		// initialize publication structures
		memset(&_outputs, 0, sizeof(_outputs));

		// set up poll topic and limit poll interval
		px4_pollfd_struct_t fds[1];
		fds[0].fd     = _controls_sub;
		fds[0].events = POLLIN;
		//orb_set_interval(_controls_sub, 10);  // max actuator update period, ms

		// set up mixer
		if (initialize_mixer(MIXER_FILENAME) < 0) {
			PX4_ERR("Mixer initialization failed.");
			_task_should_exit = true;
		}

		uint8_t data[MAX_MOTORS + 1];

		// Main loop
		while (!_task_should_exit) {
			data[0] = 0xF5; // magic
			for (uint8_t i = 1; i <= MAX_MOTORS; i++) {
				data[i] = 0x9B;
			}

			PX4_WARN("Send ESC frame");
			int ret = ::write(_fd, &data[0], sizeof(data));

			if (ret < 1) {
				PX4_WARN("Failed sending ESC packet, ret: %d, errno: %d", ret, errno);
			}


		}
	}

	PX4_WARN("stopping simonk_esc");
}

/** simonk_esc main entrance */
void task_main_trampoline(int argc, char *argv[])
{
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("simonk_esc_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	// TODO - set thread exit signal to terminate the task main thread

	_is_running = false;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -d device");
}

} // namespace simonk_esc

/** driver 'main' command */
extern "C" __EXPORT int simonk_esc_main(int argc, char *argv[]);

int simonk_esc_main(int argc, char *argv[])
{
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			simonk_esc::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		simonk_esc::usage();
		return 1;
	}

	memset(simonk_esc::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(simonk_esc::_device, device, strlen(device));

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (simonk_esc::_is_running) {
			PX4_WARN("simonk_esc already running");
			return 1;
		}

		simonk_esc::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (simonk_esc::_is_running) {
			PX4_WARN("simonk_esc is not running");
			return 1;
		}

		simonk_esc::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("simonk_esc is %s", simonk_esc::_is_running ? "running" : "stopped");
		return 0;

	} else {
		simonk_esc::usage();
		return 1;
	}

	return 0;
}
