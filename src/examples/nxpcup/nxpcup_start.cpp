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

float speed = 0.f;
float steer = 0;
bool start = true;			// create your own start condition
bool debug = true;


//AUTOPILOT PARAMS
float FAST_SPEED = 0.5f;
float SLOW_SPEED = 0.2f;
float STEER_MULT = 1.3f;
float BIAS = 0.05f;
int BIAS_PARTS = 2;
int MAX_PARTS = 3;
int MAX_LIMIT = 2;
float MAX_STEER_PER_SECOND = 50.f;
float LINE_MEMORY = 300.f;
int VERTICAL_PARTS = 2;
float THRESHOLD = 0.7f;

void roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp)
{

	control.steer += 0.05f;
	// Converting steering value from percent to euler angle
	control.steer *= 0.7f;//60.0f; //max turn angle 60 degree

	// Converting steering value from percent to euler angle
	control.steer *= 60.0f; //max turn angle 60 degree
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

float my_abs(float x)
{
	if (x > 0) {
		return x;

	} else {
		return -x;
	}
}

float vector_len(const PixyVector &vector)
{
	return (vector.m_x0 - vector.m_x1) * (vector.m_x0 - vector.m_x1) + (vector.m_y0 - vector.m_y1) *
	       (vector.m_y0 - vector.m_y1);
}

void swap(PixyVector *xp, PixyVector *yp)
{
	PixyVector temp = *xp;
	*xp = *yp;
	*yp = temp;
}

double deg2rad(double degrees)
{
	return degrees * 4.0 * std::atan(1.0) / 180.0;
}

int PIXY_WIDTH = 72;
int PIXY_HEIGHT = 52;
void find_track_lines(PixyVector *lines, int line_count, PixyVector *result, int &lines_found,
		      float curr_steer, PixyVector *last_track_lines, hrt_abstime *last_track_lines_seen)
{
	result[0] = result[1] = {};
	PixyVector sorted_lines[line_count];
	memcpy(sorted_lines, lines, sizeof(sorted_lines));
	int i, j;

	for (i = 0; i < line_count - 1; i++)
		for (j = 0; j < line_count - i - 1; j++)
			if (vector_len(sorted_lines[j]) < vector_len(sorted_lines[j + 1])) {
				swap(&sorted_lines[j], &sorted_lines[j + 1]);
			}

	for (i = 0; i < line_count; i++) {
		PixyVector &line = lines[i];

		if (line.m_y1 > line.m_y0) {
			int t1 = line.m_x0;
			int t2 = line.m_y0;
			line.m_x0 = line.m_x1;
			line.m_y0 = line.m_y1;
			line.m_x1 = t1;
			line.m_y1 = t2;
		}
	}

	bool found_left = false;
	bool found_right = false;
	float right_max = PIXY_WIDTH / MAX_PARTS * MAX_LIMIT;
	float left_max = PIXY_WIDTH - PIXY_WIDTH / MAX_PARTS * MAX_LIMIT;
	lines_found = 0;
	float theta = deg2rad(curr_steer * 30);

	float cs = cos(theta);
	float sn = sin(theta);

	hrt_abstime curr_time = hrt_absolute_time();

	for (int vertical_part = VERTICAL_PARTS - 1; vertical_part >= 0 && lines_found == 0; vertical_part--) {
		for (i = 0; i < line_count; i++) {
			PixyVector &line = sorted_lines[i];
			int dx = line.m_x1 - line.m_x0;
			int dy = line.m_y1 - line.m_y0;

			float rotated_x1 =  line.m_x0 + dx * cs - dy * sn;
			//float rotated_y1 = line.m_x1 * sn + line.m_y1 * cs;

			if (!found_left && line.m_x0 < right_max
			    && line.m_x0 < rotated_x1
			    && (line.m_y0 > PIXY_HEIGHT * vertical_part / VERTICAL_PARTS)
			    && my_abs((int)line.m_x0 - last_track_lines[0].m_x0) < LINE_MEMORY * (curr_time - last_track_lines_seen[0]) / 1e6f) {
				memcpy(&result[0], &line, sizeof(line));
				//result[0].m_x1 = rotated_x1;
				found_left = true;
				lines_found++;

			} else if (!found_right && line.m_x0 > left_max
				   && line.m_x0 > rotated_x1
				   && (line.m_y0 > PIXY_HEIGHT * vertical_part / VERTICAL_PARTS)
				   &&  my_abs((int)line.m_x0 - last_track_lines[1].m_x0) < LINE_MEMORY * (curr_time - last_track_lines_seen[1]) / 1e6f) {
				memcpy(&result[1], &line, sizeof(line));
				//result[1].m_x1 = rotated_x1;
				found_right = true;
				lines_found++;
			}
		}
	}

	if (lines_found == 2 && result[0].m_x0 > result[1].m_x0) {
		swap(&result[0], &result[1]);
	}

}

PixyVector find_target_vector(PixyVector left_vector, PixyVector right_vector)
{
	PixyVector result;

	uint8_t offset = 0;
	float bias_coeff = BIAS;
	uint8_t bias_part = BIAS_PARTS;
	result.m_y0 = PIXY_HEIGHT - 1;

	if (left_vector.m_x0 == 0 && left_vector.m_x1 == 0 && right_vector.m_x0 == 0 && right_vector.m_x1 == 0) {

		result.m_x0 = right_vector.m_x0;
		result.m_x1 = right_vector.m_x1;
		result.m_y1 = right_vector.m_y1;

	} else if (left_vector.m_x0 == 0 && left_vector.m_x1 == 0) {
		//result.m_y0 = right_vector.m_y0;
		result.m_x0 = right_vector.m_x0;
		result.m_x1 = right_vector.m_x1 - offset;
		result.m_y1 = right_vector.m_y1;

	} else if (right_vector.m_x0 == 0 && right_vector.m_x1 == 0) {
		//result.m_y0 = left_vector.m_y0;
		result.m_x0 = left_vector.m_x0;
		result.m_x1 = left_vector.m_x1 + offset;
		result.m_y1 = left_vector.m_y1;

	} else {
		//result.m_y0 = PIXY_HEIGHT;
		result.m_y1 = right_vector.m_y1 / 2 + left_vector.m_y1 / 2;
		result.m_x0 = PIXY_WIDTH / 2;
		result.m_x1 = (left_vector.m_x1 + right_vector.m_x1) / 2;
	}

	//if (right_vector.m_x0 != 0 && right_vector.m_x0 < PIXY_WIDTH * (bias_part - 1) / bias_part) {
	//result.m_x1 += bias_coeff * (right_vector.m_x0 - PIXY_WIDTH * (bias_part - 1) / bias_part) / (PIXY_WIDTH / bias_part);
	//result.m_x1 -= bias_coeff * (PIXY_WIDTH - right_vector.m_x0) / PIXY_WIDTH;
	//result.m_x1 -= PIXY_WIDTH / 4;
	//result.m_x0 = PIXY_WIDTH / 2;
	//result.m_x1 = PIXY_WIDTH / 2;
	int part = PIXY_WIDTH / bias_part;


	if (right_vector.m_x0 != 0) {
		int diff = (PIXY_WIDTH - right_vector.m_x0 + part);
		result.m_x1 += bias_coeff * diff * diff / (1.0f - part);
	}

	//}

	//if (left_vector.m_x0 != 0 && left_vector.m_x0 > PIXY_WIDTH * 1 / bias_part) {
	//result.m_x1 += bias_coeff * (left_vector.m_x0 - PIXY_WIDTH * 1 / bias_part) / (PIXY_WIDTH / bias_part);
	//result.m_x1 += bias_coeff * (right_vector.m_x0) / PIXY_WIDTH;
	//result.m_x1 += PIXY_WIDTH / 4;
	//result
	if (left_vector.m_x0 != 0) {
		int diff = (left_vector.m_x0 + part);
		result.m_x1 -= bias_coeff * diff * diff / (1.0f - part);
	}

	//}

	return result;
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

	/* Pixy2 Instance */
	Pixy2 pixy;
	bool wait = 1;		// needed for waiting for valid data
	usleep(5000);		// give pixy time to init
#endif

	struct debug_array_s dbg_array = {};
	dbg_array.id = 1;
	strncpy(dbg_array.name, "dbg_array", 10);
	orb_advert_t pub_dbg_array = orb_advertise(ORB_ID(debug_array), &dbg_array);

	if (pixy.init() == 0) {

		pixy.getVersion();
		pixy.version->print();
		usleep(1000);
		//int TOTAL_VECTORS_TO_REPORT = 8;
		//int curr_vector_to_report = 0;
		//bool report_vector_start = true;
		float curr_steer = 0;
		hrt_abstime last_update = hrt_absolute_time();
		PixyVector last_track_lines[2];
		last_track_lines[0] = last_track_lines[1] = {};
		hrt_abstime last_track_lines_seen[2];

		while (1) {
			float max_steer_per_second = MAX_STEER_PER_SECOND;//1e-6;//0.1;

			safety_sub.copy(&safety);				// request Safety swutch state
			safety.safety_off = 1;
			pixy.line.getAllFeatures(LINE_VECTOR, wait);		// get line vectors from pixy
			PixyVector trackLines[2];



			dbg_array.data[0] = pixy.line.numVectors;

			for (int i = 0; i < pixy.line.numVectors; i++) {
				dbg_array.data[1 + i * 4] = pixy.line.vectors[i].m_x0;
				dbg_array.data[2 + i * 4] = pixy.line.vectors[i].m_y0;
				dbg_array.data[3 + i * 4] = pixy.line.vectors[i].m_x1;
				dbg_array.data[4 + i * 4] = pixy.line.vectors[i].m_y1;
			}

			int lines_found;
			find_track_lines(pixy.line.vectors, pixy.line.numVectors, trackLines, lines_found, curr_steer,
					 last_track_lines, last_track_lines_seen);

			if (false && lines_found == 1) {
				int offset = 5;
				int len = 5;
				uint8_t r[len * 2];
				uint8_t g[len * 2];
				uint8_t b[len * 2];
				PixyVector foundLine;

				if (trackLines[0].m_x0 == trackLines[0].m_x1) {
					foundLine = trackLines[1];

				} else {
					foundLine = trackLines[0];
				}

				int left_sum = 0;
				int right_sum = 0;

				for (int i = 0; i < len; i++) {
					int left_pos = foundLine.m_x0 - offset - i;

					if (left_pos < 0) {
						left_pos = 0;
					}

					int right_pos = foundLine.m_x0 + offset + i;

					if (right_pos >= PIXY_WIDTH) {
						right_pos = PIXY_WIDTH - 1;
					}

					pixy.video.getRGB(left_pos, foundLine.m_y0, &r[i], &g[i], &b[i], false);
					pixy.video.getRGB(right_pos, foundLine.m_y0, &r[len + i], &g[len + i], &b[len + i], false);
					left_sum += r[i] + g[i] + b[i];
					right_sum += r[len + i] + g[len + i] + b[len + i];
				}

				if (left_sum > right_sum && trackLines[1].m_x0 == trackLines[1].m_x1) {
					swap(&trackLines[0], &trackLines[1]);
				}

				if (right_sum > left_sum && trackLines[0].m_x0 == trackLines[0].m_x1) {
					swap(&trackLines[0], &trackLines[1]);
				}
			}

			for (int i = 0; i < 2; i++) {
				dbg_array.data[1 + (i + pixy.line.numVectors) * 4] = trackLines[i].m_x0;
				dbg_array.data[2 + (i + pixy.line.numVectors) * 4] = trackLines[i].m_y0;
				dbg_array.data[3 + (i + pixy.line.numVectors) * 4] = trackLines[i].m_x1;
				dbg_array.data[4 + (i + pixy.line.numVectors) * 4] = trackLines[i].m_y1;
			}

			PixyVector target_vector = find_target_vector(trackLines[0], trackLines[1]);


			dbg_array.data[1 + (2 + pixy.line.numVectors) * 4] = target_vector.m_x0;
			dbg_array.data[2 + (2 + pixy.line.numVectors) * 4] = target_vector.m_y0;
			dbg_array.data[3 + (2 + pixy.line.numVectors) * 4] = target_vector.m_x1;
			dbg_array.data[4 + (2 + pixy.line.numVectors) * 4] = target_vector.m_y1;

			float target_steer = std::atan(1.f * (target_vector.m_x1 - target_vector.m_x0) / (target_vector.m_y1 -
						       target_vector.m_y0)) / 1.4;

			target_steer *= STEER_MULT;

			if (target_vector.m_x0 == target_vector.m_x1 && target_vector.m_y0 == target_vector.m_y1) {
				target_steer = 0;
			}

			dbg_array.data[5 + (2 + pixy.line.numVectors) * 4] = target_steer;
			float steer_diff = target_steer - curr_steer;
			hrt_abstime curr_time = hrt_absolute_time();
			float max_steer = (double)max_steer_per_second * (curr_time - last_update) * 1e-6;

			if (lines_found == 1) {
				//max_steer /= 2;

			} else if (lines_found == 0) {
				max_steer /= 10;
			}

			if (trackLines[0].m_x0 != trackLines[0].m_x1) {
				last_track_lines[0] = trackLines[0];
				last_track_lines_seen[0] = hrt_absolute_time();
			}

			if (trackLines[1].m_x0 != trackLines[1].m_x1) {
				last_track_lines[1] = trackLines[1];
				last_track_lines_seen[1] = hrt_absolute_time();
			}

			last_update = curr_time;

			if (my_abs(steer_diff) < max_steer) {
				curr_steer += steer_diff;

			} else {
				if (steer_diff > 0) {
					curr_steer += max_steer;

				} else {
					curr_steer -= max_steer;
				}
			}

			dbg_array.data[6 + (2 + pixy.line.numVectors) * 4] = curr_steer;

			switch (safety.safety_off) {
			case 0:
				// Setting vehicle into the default state
				_control_mode.flag_control_manual_enabled	= true;
				_control_mode.flag_control_attitude_enabled	= true;
				_control_mode.flag_control_velocity_enabled	= true;
				_control_mode.flag_control_position_enabled	= true;

				pixy.setLED(0, 0, 0);		// Pixy: reset RGB led
				pixy.setLamp(false, false);	// Pixy: reset upper leds
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

				pixy.setLED(0, 0, 255);		// Pixy: set RGB led to blue
				pixy.setLamp(true, true);

				if (lines_found == 0) {
					//pixy.setLamp(false, false);	// Pixy: sets upper led

				} else if (lines_found == 1) {
					//pixy.setLamp(true, false);	// Pixy: sets upper led

				} else {
					//pixy.setLamp(false, true);	// Pixy: sets upper led
				}

				if (debug) {
					motorControl.speed = speed;
					motorControl.steer = steer;

				} else if (start) {
					//motorControl = raceTrack(pixy);
					float speedup = (THRESHOLD - my_abs(curr_steer)) * (FAST_SPEED - SLOW_SPEED);

					if (speedup < 0) {
						speedup = 0;
					}

					motorControl.speed = SLOW_SPEED + speedup;
					motorControl.steer = curr_steer;

				} else {
					motorControl.speed = speed;
					motorControl.steer = curr_steer;
				}

				break;
			}

			roverSteerSpeed(motorControl, _att_sp);		// setting values for speed and steering to attitude setpoints

			// Publishing all
			_control_mode.timestamp = hrt_absolute_time();
			_control_mode_pub.publish(_control_mode);
			_att_sp.timestamp = hrt_absolute_time();
			_att_sp_pub.publish(_att_sp);

			// dbg_vect.timestamp = hrt_absolute_time();
			// orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);

			dbg_array.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(debug_array), pub_dbg_array, &dbg_array);

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

	if (!strcmp(argv[1], "k")) {
		start = !start;
		return 0;
	}

	if (!strcmp(argv[1], "d")) {
		debug = !debug;
		return 0;
	}

	if (!strcmp(argv[1], "fs")) {
		FAST_SPEED = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "ss")) {
		SLOW_SPEED = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "sm")) {
		STEER_MULT = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "b")) {
		BIAS = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "bp")) {
		BIAS_PARTS = atoi(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "mp")) {
		MAX_PARTS = atoi(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "ml")) {
		MAX_LIMIT = atoi(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "ms")) {
		MAX_STEER_PER_SECOND = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "lm")) {
		LINE_MEMORY = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "t")) {
		THRESHOLD = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "vp")) {
		VERTICAL_PARTS = atoi(argv[2]);
		return 0;
	}

	PX4_WARN("usage: race {start|stop|status}\n");
	return 1;
}
