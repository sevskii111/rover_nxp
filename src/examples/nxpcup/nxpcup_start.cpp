/****************************************************************************
 *
 *   Copyright 2019 NXP.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file nxpcup_main.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_start.h"

#include "nxpcup_race.h"

using namespace matrix;

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit = false;
bool threadIsRunning = false;

float speed = 0.1f;
float steer = 0;

void roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp)
{
	// Converting steering value from percent to euler angle
	control.steer *= 0.7f;//60.0f; //max turn angle 60 degree

	// Converting steering value from percent to euler angle
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159 / 180; // change to radians


	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	Eulerf euler{0.0, 0.0, control.steer};
	Quatf qe{euler};

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	_att_sp.q_d[0] = qe(0);
	_att_sp.q_d[1] = qe(1);
	_att_sp.q_d[2] = qe(2);
	_att_sp.q_d[3] = qe(3);

}

int race_thread_main(int argc, char **argv)
{
	threadIsRunning = true;

	#ifndef ROVER_INIT_VALUES
	#define ROVER_INIT_Values

	/* Publication of uORB messages */
	uORB::Publication<vehicle_control_mode_s>		_control_mode_pub{ORB_ID(vehicle_control_mode)};
	struct vehicle_control_mode_s				_control_mode {};

	uORB::Publication<vehicle_attitude_setpoint_s>		_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	struct vehicle_attitude_setpoint_s			_att_sp {};

	/* Publication of uORB messages */
	struct safety_s safety;
	uORB::Subscription safety_sub{ORB_ID(safety)};		// Safety switch request for starting and stopping the racing
	safety_sub.copy(&safety);

	/* Return motor control variables */
	roverControl motorControl;

	/* Start condition of the race */
	bool start = 0;		// Create your own start condition

	/* Pixy2 Instance */
	Pixy2 pixy;
	bool wait = 1;		// needed for waiting for valid data
	usleep(5000);		// give pixy time to init
	#endif


	if (pixy.init() == 0) {

		pixy.getVersion();
		pixy.version->print();
		usleep(1000);

		while (1) {
			safety_sub.copy(&safety);				// request Safety swutch state
			safety.safety_off = 1;
			pixy.line.getAllFeatures(LINE_VECTOR, wait);		// get line vectors from pixy

			switch (safety.safety_off) {
			case 0:
				// Setting vehicle into the default state
				_control_mode.flag_control_manual_enabled	= true;
				_control_mode.flag_control_attitude_enabled	= true;
				_control_mode.flag_control_velocity_enabled	= true;
				_control_mode.flag_control_position_enabled	= true;

				pixy.setLED(0,0,0);		// Pixy: reset RGB led
				pixy.setLamp(false,false);	// Pixy: reset upper leds
				// reset PWM outputs
				motorControl.speed = 0.0f;
				motorControl.steer = 0.0f;
				break;
			case 1:
				// Setting vehicle to attitude control mode
				_control_mode.flag_control_manual_enabled 	= false;
				_control_mode.flag_control_attitude_enabled 	= true;
				_control_mode.flag_control_velocity_enabled 	= false;
				_control_mode.flag_control_position_enabled	= false;

				start = false;			// create your own start condition
				pixy.setLED(0,0,255);		// Pixy: set RGB led to blue
				pixy.setLamp(true,false);	// Pixy: sets upper led

				if (start) {
					motorControl = raceTrack(pixy);
				} else {
					motorControl.speed = speed;
					motorControl.steer = steer;
				}
				break;
			}
			roverSteerSpeed(motorControl, _att_sp);		// setting values for speed and steering to attitude setpoints

			// Publishing all
			_control_mode.timestamp = hrt_absolute_time();
			_control_mode_pub.publish(_control_mode);
			_att_sp.timestamp = hrt_absolute_time();
			_att_sp_pub.publish(_att_sp);

			if (threadShouldExit) {
				threadIsRunning = false;
				// reset speed and steering
				roverSteerSpeed(motorControl, _att_sp);
				// puplishing attitude setpoints
				_att_sp.timestamp = hrt_absolute_time();
				_att_sp_pub.publish(_att_sp);

				// Setting vehicle into the default state
				_control_mode.flag_control_manual_enabled 	= true;
				_control_mode.flag_control_attitude_enabled 	= true;
				_control_mode.flag_control_velocity_enabled 	= true;
				_control_mode.flag_control_position_enabled	= true;
				_control_mode.timestamp = hrt_absolute_time();
				_control_mode_pub.publish(_control_mode);

				PX4_INFO("Exit Rover Thread!\n");
				return 1;
			}
		}
	}
	return 0;
}


extern "C" __EXPORT int nxpcup_main(int argc, char *argv[]);
int nxpcup_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: nxpcup {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit = false;
		daemon_task = px4_task_spawn_cmd("nxpcup",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 race_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "speed")) {
			speed = atof(argv[2]);
			return 0;
	}

	if (!strcmp(argv[1], "steer")) {
			steer = atof(argv[2]);
			return 0;
	}

	PX4_WARN("usage: race {start|stop|status}\n");
	return 1;
}
