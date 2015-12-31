/****************************************************************************
 *
 *   Copyright (c) 2015 Mohammed Kabir. All rights reserved.
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

/**
 * @file sat_att_control_main.c
 * Implementation of a CubeSat attitude controller based on orthogonal PIDs.
 *
 * @author Mohammed Kabir	<mhkabir98@gmail.com>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <platforms/px4_defines.h>

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sat_att_control_main(int argc, char *argv[]);

class SatelliteAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	SatelliteAttitudeControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~SatelliteAttitudeControl();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;		/**< control state subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_pub;			/**< actuator control group 0 setpoint */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;		/**< control state */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s			_rates_sp;		/* attitude rates setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	struct {
		float p_tc;
		float p_p;
		float p_i;
		
		float r_tc;
		float r_p;
		float r_i;
		
		float y_tc;
		float y_p;
		float y_i;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t p_tc;
		param_t p_p;
		param_t p_i;

		param_t r_tc;
		param_t r_p;
		param_t r_i;

		param_t y_tc;
		param_t y_p;
		param_t y_i;

	}		_parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;
	
	float _roll_sp;
	float _pitch_sp;
	float _yaw_sp;
	float _roll_rate_sp;
	float _pitch_rate_sp;
	float _yaw_rate_sp;
	float _roll_bodyrate_sp;
	float _pitch_bodyrate_sp;
	float _yaw_bodyrate_sp;
	
	float _roll_bodyrate_sp_prev;
	float _pitch_bodyrate_sp_prev;
	float _yaw_bodyrate_sp_prev;
	
	float _roll_integrator;
	float _pitch_integrator;
	float _yaw_integrator;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();
	
	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();
	
	/**
	 * Run attitude controller
	 */
	void		control_attitude(float dt);

	/**
	 * Run attitude rate controller
	 */
	void		control_bodyrates(float dt);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

SatelliteAttitudeControl	*g_control = nullptr;
}

SatelliteAttitudeControl::SatelliteAttitudeControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_accel_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_vehicle_status_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_pub(nullptr),

	_rates_sp_id(0),
	_actuators_id(0),
	_attitude_setpoint_id(0),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "sat att control")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "sat att control nonfinite input")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "sat att control nonfinite output")),
	
	/* states */
	_setpoint_valid(false),
	_debug(false)
{
	/* safely initialize structs */
	_ctrl_state = {};
	_accel = {};
	_att_sp = {};
	_rates_sp = {};
	_manual = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};

	_parameter_handles.p_tc = param_find("SAT_P_TC");
	_parameter_handles.p_p = param_find("SAT_PR_P");
	_parameter_handles.p_i = param_find("SAT_PR_I");

	_parameter_handles.r_tc = param_find("SAT_R_TC");
	_parameter_handles.r_p = param_find("SAT_RR_P");
	_parameter_handles.r_i = param_find("SAT_RR_I");

	_parameter_handles.y_tc = param_find("SAT_Y_TC");
	_parameter_handles.y_p = param_find("SAT_YR_P");
	_parameter_handles.y_i = param_find("SAT_YR_I");

	/* fetch initial parameter values */
	parameters_update();
}

SatelliteAttitudeControl::~SatelliteAttitudeControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_control::g_control = nullptr;
}

int
SatelliteAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));

	param_get(_parameter_handles.y_tc, &(_parameters.y_tc));
	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));

	return OK;
}

void
SatelliteAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
SatelliteAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
SatelliteAttitudeControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
SatelliteAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
SatelliteAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}


void
SatelliteAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
		_actuators_id = ORB_ID(actuator_controls_0);
		_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			
	}
}

void
SatelliteAttitudeControl::control_attitude(float dt)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(_pitch_sp) &&
	      PX4_ISFINITE(_roll_sp) &&
	      PX4_ISFINITE(_yaw_sp) &&
	      PX4_ISFINITE(_roll) &&
	      PX4_ISFINITE(_pitch) &&
	      PX4_ISFINITE(_yaw))) {
		perf_count(_nonfinite_input_perf);
		return;
	}

	/* Calculate the error */
	float roll_error = _roll_sp - _roll;
	float pitch_error = _pitch_sp - _pitch;
	float yaw_error = _yaw_sp - _yaw;
	
	/*  Apply P controller: rate setpoint from current error and time constant */
	_roll_rate_sp =  roll_error / _parameters.r_tc;
	_pitch_rate_sp =  pitch_error / _parameters.p_tc;
	_yaw_rate_sp =  yaw_error / _parameters.y_tc;

	// XXX TODO introduce rate limiting

}
void
SatelliteAttitudeControl::control_bodyrates(float dt)
{
	bool lock_integrator = false;
	
	if (dt > 500000) { // microsec
		lock_integrator = true;
	}
	
	/* Transform setpoints to body angular rates (jacobian) */
	float roll_bodyrate = _roll_rate_sp - sinf(_pitch) * _yaw_rate_sp;
			 
	float pitch_bodyrate = cosf(_roll) * _pitch_rate_sp +
			       cosf(_pitch) * sinf(_roll) * _yaw_rate_sp;
			  
	float yaw_bodyrate = -sinf(_roll) * _pitch_rate_sp +
			      cosf(_roll) * cosf(_pitch) * _yaw_rate_sp;
	
	/* Calculate the error */
	float roll_rate_error = roll_bodyrate - _ctrl_state.roll_rate;
	float pitch_rate_error = pitch_bodyrate - _ctrl_state.pitch_rate;
	float yaw_rate_error = yaw_bodyrate - _ctrl_state.yaw_rate;

	/* Stop integrators if actuators are saturated */
	if (!lock_integrator && _parameters.r_i > 0.0f) {
		float id = roll_rate_error * dt;
		if (_roll_bodyrate_sp_prev < -1.0f) {
			id = math::max(id, 0.0f);
		} else if (_roll_bodyrate_sp_prev > 1.0f) {
			id = math::min(id, 0.0f);
		}
		_roll_integrator += id;
	}
	if (!lock_integrator && _parameters.p_i > 0.0f) {
		float id = pitch_rate_error * dt;
		if (_pitch_bodyrate_sp_prev < -1.0f) {
			id = math::max(id, 0.0f);
		} else if (_pitch_bodyrate_sp_prev > 1.0f) {
			id = math::min(id, 0.0f);
		}
		_pitch_integrator += id;
	}
	if (!lock_integrator && _parameters.y_i > 0.0f) {
		float id = yaw_rate_error * dt;
		if (_pitch_bodyrate_sp_prev < -1.0f) {
			id = math::max(id, 0.0f);
		} else if (_pitch_bodyrate_sp_prev > 1.0f) {
			id = math::min(id, 0.0f);
		}
		_yaw_integrator += id;
	}
	
	float integrator_max = 0.5f;
	
	/* Integrator limit */
	float _roll_integrator_constr = math::constrain(_roll_integrator * _parameters.r_i, -integrator_max, integrator_max); 
	float _pitch_integrator_constr = math::constrain(_pitch_integrator * _parameters.p_i, -integrator_max, integrator_max);
	float _yaw_integrator_constr = math::constrain(_yaw_integrator * _parameters.y_i, -integrator_max, integrator_max);
	
	/* Apply PI rate controller and store non-limited output TODO : feedforward */
	_roll_bodyrate_sp_prev = roll_bodyrate /** _k_ff */ + roll_rate_error * _parameters.r_p + _roll_integrator_constr;
	_pitch_bodyrate_sp_prev = pitch_bodyrate /** _k_ff */ + pitch_rate_error * _parameters.p_p + _pitch_integrator_constr;
	_yaw_bodyrate_sp_prev = yaw_bodyrate /** _k_ff */ + yaw_rate_error * _parameters.y_p + _yaw_integrator_constr;
		       
	_roll_bodyrate_sp = math::constrain(_roll_bodyrate_sp_prev, -1.0f, 1.0f);
	_pitch_bodyrate_sp = math::constrain(_pitch_bodyrate_sp_prev, -1.0f, 1.0f);
	_yaw_bodyrate_sp = math::constrain(_yaw_bodyrate_sp_prev, -1.0f, 1.0f);
			 
}

void
SatelliteAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
SatelliteAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);


			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_roll    = euler_angles(0);
			_pitch   = euler_angles(1);
			_yaw     = euler_angles(2);

			vehicle_setpoint_poll();

			vehicle_accel_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();
			
			global_pos_poll();
			
			vehicle_status_poll();
			
			/* Manual setpoint input TODO : make sure we can get this from elsewhere as well*/
			_roll_sp = _manual.y;
			_pitch_sp = - _manual.x;
			_yaw_sp = _manual.r;

			/* Publish setpoints for attitude controller tuning */
			struct vehicle_attitude_setpoint_s att_sp = {} ;
			att_sp.timestamp = hrt_absolute_time();
			att_sp.roll_body = _roll_sp;
			att_sp.pitch_body = _pitch_sp;
			att_sp.yaw_body = _yaw_sp;

			/* lazily publish the setpoint only once available */
			if (_attitude_sp_pub != nullptr) {
				/* publish the attitude setpoint */
				orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &att_sp);
				
			} else if (_attitude_setpoint_id) {
				/* advertise and publish */
				_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &att_sp);
			}
			
			/* Run controllers */
			control_attitude(deltaT);
			control_bodyrates(deltaT);
			
			/* XXX Publish rate setpoints and all others too! */
			
			 /* Fill acutator controls */
			_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(_roll_bodyrate_sp)) ? _roll_bodyrate_sp : 0.0f;
			_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(_pitch_bodyrate_sp)) ? _pitch_bodyrate_sp : 0.0f;
			_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(_yaw_bodyrate_sp)) ? _yaw_bodyrate_sp : 0.0f;
			
			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;

			/* publish the actuator controls */
			if (_actuators_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_pub, &_actuators);

			} else if (_actuators_id) {
				_actuators_pub = orb_advertise(_actuators_id, &_actuators);
			}
		
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

int
SatelliteAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("sat_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1300,
					   (px4_main_t)&SatelliteAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int sat_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: sat_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		att_control::g_control = new SatelliteAttitudeControl;

		if (att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (att_control::g_control == nullptr || !att_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete att_control::g_control;
		att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
