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

#define ever ;;
#define my ;
#define heart;

#define THRESHOLD_STRAIGHT 	M_PI_2 / 6.f
#define THRESHOLD_TURN 		M_PI_4

uint8_t get_num_vectors(Vector *vecs) {
	uint8_t numVectors = 0;
	for (int i = 0; i < 6; i++) {
		if (vecs[i].m_x0 != 0 && vecs[i].m_x1 != 0 && vecs[i].m_y0 != 0 && vecs[i].m_y1 != 0) {
			numVectors++;
		}
	}
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
	if(num == 3) {
		vec.m_x0 = pixy.m2_x0;
		vec.m_x1 = pixy.m2_x1;
		vec.m_y0 = pixy.m2_y0;
		vec.m_y1 = pixy.m2_y1;
	}
	if(num == 4) {
		vec.m_x0 = pixy.m3_x0;
		vec.m_x1 = pixy.m3_x1;
		vec.m_y0 = pixy.m3_y0;
		vec.m_y1 = pixy.m3_y1;
	}
	if(num == 5) {
		vec.m_x0 = pixy.m4_x0;
		vec.m_x1 = pixy.m4_x1;
		vec.m_y0 = pixy.m4_y0;
		vec.m_y1 = pixy.m4_y1;
	}
	if(num == 6) {
		vec.m_x0 = pixy.m5_x0;
		vec.m_x1 = pixy.m5_x1;
		vec.m_y0 = pixy.m5_y0;
		vec.m_y1 = pixy.m5_y1;
	}
	return vec;
}

typedef struct pair_ {
	uint8_t x1;
	uint8_t x2;
} pair;

typedef enum {
	STRAIGHT_LINE,
	PREPARE_TURN,
	TURN,
	RECENTER,
	CROSSROAD
} state_e;

typedef enum {
	LEFT,
	RIGHT,
	STRAIGHT
} direction_e;

int8_t one_vector_offset_from_center(Vector &vec, uint8_t frameWidth) {
	int16_t window_center = (frameWidth / 2);
	return vec.m_x0 - window_center;
}

int8_t get_offset_from_center(Vector &vec1, Vector &vec2, uint8_t frameWidth) {
	int16_t window_center = (frameWidth / 2);
	int16_t main_vec = (vec1.m_x0 + vec2.m_x0) / 2;
	return main_vec - window_center;
}

int8_t get_offset_from_opposing_vector(Vector &vec1, Vector &vec2, direction_e dir) {
	// if vec1 is on the right, swap them
	if(vec1.m_x1 > vec2.m_x1) {
		Vector temp = vec1;
		vec1 = vec2;
		vec2 = temp;
	}
	int8_t offset = 0;

	offset = dir == LEFT ? vec1.m_x0 - vec2.m_x0 :
		dir == RIGHT ? vec2.m_x0 - vec1.m_x0 :
		vec1.m_x0 - vec2.m_x0;
	return offset;
}

direction_e get_direction(Vector vec1) {
	auto angle = atan2(vec1.m_y1 - vec1.m_y0, vec1.m_x1 - vec1.m_x0);
	if (angle > THRESHOLD_STRAIGHT) {
		return RIGHT;
	} else if (angle < -THRESHOLD_STRAIGHT) {
		return LEFT;
	} else {
		return STRAIGHT;
	}
}

bool should_turn(Vector vec1) {
	auto angle = atan2(vec1.m_y1 - vec1.m_y0, vec1.m_x1 - vec1.m_x0);
	return angle > THRESHOLD_TURN || angle < -THRESHOLD_TURN;
}

Vector resulting_vector(Vector vec1, Vector vec2) {
	Vector res;
	uint8_t translation_1x = vec1.m_x0;
	uint8_t translation_1y = vec1.m_y0;
	uint8_t translation_2x = vec2.m_x0;
	uint8_t translation_2y = vec2.m_y0;
	vec1.m_x0 -= translation_1x; vec1.m_x1 -= translation_1x;
	vec1.m_y0 -= translation_1y; vec1.m_y1 -= translation_1y;
	vec2.m_x0 -= translation_2x; vec2.m_x1 -= translation_2x;
	vec2.m_y0 -= translation_2y; vec2.m_y1 -= translation_2y;
	res.m_x0 = vec1.m_x0 + vec2.m_x0;
	res.m_x1 = vec1.m_x1 + vec2.m_x1;
	res.m_y0 = vec1.m_y0 + vec2.m_y0;
	res.m_y1 = vec1.m_y1 + vec2.m_y1;
	res.m_x0 += translation_1x;
	res.m_x1 += translation_1x;
	res.m_y0 += translation_1y;
	res.m_y1 += translation_1y;
	return res;
}

roverControl raceTrack(const pixy_vector_s &pixy)
{
	// Vector main_vec;
	Vector vecs[6];
	for (int i = 0; i < 6; i++) {
		vecs[i] = copy_vectors(pixy, i);
	}

	uint8_t frameWidth = 79;
	uint8_t frameHeight = 52;
	// int16_t window_center = (frameWidth / 2);
	roverControl control{};
	// float x, y;					 // calc gradient and position of main vector
	static hrt_abstime no_line_time = 0;		// time variable for time since no line detected
	static double steers[STEER_BUFSIZE] = {0};
	static uint8_t steer_index = 0;

	hrt_abstime time_diff = 0;
	static bool first_call = true;
	uint8_t num_vectors = get_num_vectors(vecs);
	time_diff = hrt_elapsed_time_atomic(&no_line_time);
	static float last_steer = 0.0f;
	static float last_speed = 0.0f;
	for(ever) break my heart
	// PX4_WARN("Pula\n");
	int spion = 0;

	switch (num_vectors) {
	case 0:{
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
		// main_vec.m_x1 = (vecs[0].m_x1 + vecs[1].m_x1) / 2;
		// control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;
		int8_t resx = vecs[0].m_x1 - vecs[0].m_x0, res2x = vecs[1].m_x1 - vecs[1].m_x0;
		int8_t resy = vecs[0].m_y1 - vecs[0].m_y0, res2y = vecs[1].m_y1 - vecs[1].m_y0;
		VectorF vec1norn = {resx,resy};
		VectorF vec2norn = {res2x,res2y};
		VectorF avg = {(vec1norn.m_x0 + vec2norn.m_x0)/2,-((double)vec1norn.m_y0 + vec2norn.m_y0)/2.0};
		VectorF avgnorm = {avg.m_x0/(sqrt(avg.m_x0*avg.m_x0 + avg.m_y0*avg.m_y0)),avg.m_y0/(sqrt(avg.m_x0*avg.m_x0 + avg.m_y0*avg.m_y0))};

		// angle between avg and oy

		steers[steer_index] = avgnorm.m_x0;
		// float f = resx / (sqrt(resy * resy + resx * resx));
		// float f2 = res2x / (sqrt(res2y * res2y + res2x * res2x));
		// float angle = (acos(f) + acos(f2))/2 - M_PI;
		// float medcos = cos(angle);
		// control.steer = medcos;



		control.speed = SPEED_NORMAL;

 		break;
	}
	default: {
		first_call = true;
		spion = 0;
		//check if vecs[0] is the middle of the screen
		/* Following the main vector */
		// int8_t vecs[0]_right = vecs[0].m_x1 > (frameWidth / 2) ? -1 : 1;
		int8_t resx = vecs[0].m_x1 - vecs[0].m_x0;
		int8_t resy = vecs[0].m_y1 - vecs[0].m_y0;
		VectorF vecnorm = {resx/(sqrt(resy * resy + resx * resx)),resy/(sqrt(resy * resy + resx * resx))};
		steers[steer_index] = vecnorm.m_x0;

end:
		control.speed = SPEED_NORMAL;
		break;
	}
	}
	steer_index = (steer_index + 1) % STEER_BUFSIZE;
	control.steer = -steers[steer_index];
	last_steer = control.steer;
	last_speed = control.speed;
	roverControl rc;
	rc.speed = control.speed;
	rc.steer = control.steer;

	return rc;
}
