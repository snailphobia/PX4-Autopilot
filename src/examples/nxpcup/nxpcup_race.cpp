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
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

uint8_t get_num_vectors(Vector &vec1, Vector &vec2) {
	uint8_t numVectors = 0;
	if(!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) numVectors++;
	if(!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) numVectors++;
	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num) {
	Vector vec;
	if(num == 1) {
		vec.m_x0 = pixy.m0_x0;
		vec.m_x1 = pixy.m0_x1;
		vec.m_y0 = pixy.m0_y0;
		vec.m_y1 = pixy.m0_y1;
	}
	if(num == 2) {
		vec.m_x0 = pixy.m1_x0;
		vec.m_x1 = pixy.m1_x1;
		vec.m_y0 = pixy.m1_y0;
		vec.m_y1 = pixy.m1_y1;
	}
	return vec;
}

void push_vhs(vhs &v, Vector vec) {
	if (v.crt_num < POOL_SIZE) {
		v.pool[v.crt_num] = vec;
		v.crt_num++;
	} else {
		for (int i = 0; i < POOL_SIZE - 1; i++) {
			v.pool[i] = v.pool[i + 1];
		}
		v.pool[POOL_SIZE - 1] = vec;
	}
}

void pop_vhs(vhs &v) {
	if (v.crt_num > 0) {
		v.pool[v.crt_num] = {};
		v.crt_num--;
	}
}

vhs vec_history = {};

roverControl __attribute__((optimize(0))) raceTrack(const pixy_vector_s &pixy)
{
	// Vector main_vec;

	Vector vec1 = copy_vectors(pixy, 1);
	Vector vec2 = copy_vectors(pixy, 2);

	// if vec1: a->b and vec2: b->c, keep only one from a->c as one vector
	if(abs(vec1.m_x0 - vec2.m_x1) <= 33 && abs(vec1.m_y0 - vec2.m_y1) <= 3) {
		vec1.m_x1 = vec2.m_x1;
		vec1.m_y1 = vec2.m_y1;
		vec2.m_x0 = 0;
		vec2.m_x1 = 0;
		vec2.m_y0 = 0;
		vec2.m_y1 = 0;
	}

	uint8_t frameWidth = 79;
	uint8_t frameHeight = 52;
	// int16_t window_center = (frameWidth / 2);
	roverControl control{};
	float x, y;					 // calc gradient and position of main vector
	static hrt_abstime no_line_time = 0;		// time variable for time since no line detected
	hrt_abstime time_diff = 0;
	static bool first_call = true;
	uint8_t num_vectors = get_num_vectors(vec1, vec2);
	time_diff = hrt_elapsed_time_atomic(&no_line_time);
	static float last_steer = 0.0f;
	static float last_speed = 0.0f;
	// PX4_WARN("Pula\n");
	int spion = 0;
	// dummy vector: straight upward vector
	Vector dummy = {.m_x0 = 0, .m_y0 = 0, .m_x1 = 0, .m_y1 = 0};
	switch (num_vectors) {
	case 0:{
		push_vhs(vec_history, dummy);
		if(first_call){
			no_line_time = hrt_absolute_time();
			first_call = false;
		}else{
			time_diff = hrt_elapsed_time_atomic(&no_line_time);
			if(!spion) {
				control.steer = 2*last_steer;
				control.speed = 0.5f*last_speed;
				spion = 1;
			}

			if(time_diff > 10000){
				/* Stopping if no vector is available */
				control.steer = 0.0f;
				control.speed = SPEED_STOP;
			}
		}
		break;
	}
	case 2:{
		first_call = true;
		spion = 0;
		/* Very simple steering angle calculation, get average of the x of top two points and
		   find distance from center of frame */
		// main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
		// control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;
		int8_t vec1_right = vec1.m_x1 > (frameWidth / 2) ? -1 : 1;
		int8_t vec2_right = vec2.m_x1 > (frameWidth / 2) ? -1 : 1;
		int8_t resx = vec1.m_x1 - vec1.m_x0, res2x = vec2.m_x1 - vec2.m_x0;
		int8_t resy = vec1.m_y1 - vec1.m_y0, res2y = vec2.m_y1 - vec2.m_y0;
		float f = resy / (sqrt(resy * resy + resx * resx));
		float f2 = res2y / (sqrt(res2y * res2y + res2x * res2x));
		float angle = (acos(f * vec1_right) + acos(f2 * vec2_right))/2;
		float medcos = cos(angle);
		control.steer = medcos;
		control.speed = SPEED_NORMAL;

		push_vhs(vec_history, vec1);
		push_vhs(vec_history, vec2);

		break;
	}
	default: {
		first_call = true;
		spion = 0;
		/* Following the main vector */
		if (vec1.m_x1 > vec1.m_x0) {
			x = (float)(vec1.m_x1 - vec1.m_x0) / (float)frameWidth;
			y = (float)(vec1.m_y1 - vec1.m_y0) / (float)frameHeight;
		} else {
			x = (float)(vec1.m_x0 - vec1.m_x1) / (float)frameWidth;
			y = (float)(vec1.m_y0 - vec1.m_y1) / (float)frameHeight;
		}
		if(vec1.m_x0 != vec1.m_x1){
			control.steer = (-1) * x / y; // Gradient of the main vector
			control.speed = SPEED_NORMAL;
		}else{
			control.steer = 0.0;
			control.speed = SPEED_NORMAL;
		}
		push_vhs(vec_history, vec1);

		break;
	}
	}

	// compute the average of the last 10 vectors
	// pixy_vector_s vec = {};
	// for (auto vector : vec_history) {
	// 	vec += vector;
	// }
	// vec /= vec_history.size();
	float composite_ = 0.f;
	for (int i = 0; i < vec_history.crt_num; i++) {
		if (vec_history.pool[i].m_x0 != 0 || vec_history.pool[i].m_y0 != 0) {
			vec_history.pool[i].m_y1 -= vec_history.pool[i].m_y0;
			vec_history.pool[i].m_x1 -= vec_history.pool[i].m_x0;
		}
		if (vec_history.pool[i].m_x1 != 0)
			composite_ += atan2(vec_history.pool[i].m_y1, vec_history.pool[i].m_x1);
	}
	composite_ /= vec_history.crt_num;

	control.steer = -control.steer;

	control.steer = composite_;
	last_steer = control.steer;
	last_speed = control.speed;
	roverControl rc;
	rc.speed = control.speed;
	rc.steer = control.steer;

	return rc;
}
