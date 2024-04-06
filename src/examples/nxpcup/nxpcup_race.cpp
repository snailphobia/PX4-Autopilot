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
#include "blackmagic.h"
#define ever ;;
#define my ;
#define heart;

#define THRESHOLD_STRAIGHT 	M_PI_2 / 5.f
#define THRESHOLD_STRAIGHT2 	(M_PI_2 * 2.f / 3.f)

#define THRESHOLD_TURN 		M_PI_4

#define radian_to_steer(radian) (radian / M_PI_2) * 1.0f;

uint8_t get_num_vectors(Vector *vecs) {
	uint8_t numVectors = 0;
	for (int i = 0; i < 6; i++) {
		if (vecs[i].m_x0 != 0 || vecs[i].m_x1 != 0 || vecs[i].m_y0 != 0 || vecs[i].m_y1 != 0) {
			numVectors++;
		}
	}
	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num) {
	Vector vec;
	if(num == 1) {
		vec.m_x0 = (int) pixy.m0_x0;
		vec.m_x1 = (int) pixy.m0_x1;
		vec.m_y0 = (int) pixy.m0_y0;
		vec.m_y1 = (int) pixy.m0_y1;
	}
	if(num == 2) {
		vec.m_x0 = (int) pixy.m1_x0;
		vec.m_x1 = (int) pixy.m1_x1;
		vec.m_y0 = (int) pixy.m1_y0;
		vec.m_y1 = (int) pixy.m1_y1;
	}
	if(num == 3) {
		vec.m_x0 = (int) pixy.m2_x0;
		vec.m_x1 = (int) pixy.m2_x1;
		vec.m_y0 = (int) pixy.m2_y0;
		vec.m_y1 = (int) pixy.m2_y1;
	}
	if(num == 4) {
		vec.m_x0 = (int) pixy.m3_x0;
		vec.m_x1 = (int) pixy.m3_x1;
		vec.m_y0 = (int) pixy.m3_y0;
		vec.m_y1 = (int) pixy.m3_y1;
	}
	if(num == 5) {
		vec.m_x0 = (int) pixy.m4_x0;
		vec.m_x1 = (int) pixy.m4_x1;
		vec.m_y0 = (int) pixy.m4_y0;
		vec.m_y1 = (int) pixy.m4_y1;
	}
	if(num == 6) {
		vec.m_x0 = (int) pixy.m5_x0;
		vec.m_x1 = (int) pixy.m5_x1;
		vec.m_y0 = (int) pixy.m5_y0;
		vec.m_y1 = (int) pixy.m5_y1;
	}
	return vec;
}

typedef struct pair_ {
	uint8_t x1;
	uint8_t x2;
} pair;

typedef enum {
	STRAIGHT_STATE,
	TURN_L,
	TURN_R,
	CROSSROAD,
} state_e;

static state_e last_different_state = STRAIGHT_STATE;
static state_e current_state = STRAIGHT_STATE;

typedef enum {
	LEFT = 0,
	RIGHT,
	STRAIGHT
} direction_e;

#define GET_OFFSET_FROM_CENTER(vec1, vec2) (vec1.m_x0 + vec2.m_x0) / 2

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
	direction_e dir = STRAIGHT;
	float th1 = THRESHOLD_STRAIGHT;
	float th2 = THRESHOLD_STRAIGHT2;
	if (angle > THRESHOLD_STRAIGHT) {
		dir = RIGHT;
	} else if (angle < -THRESHOLD_STRAIGHT) {
		dir = LEFT;
	} else {
		dir = STRAIGHT;
	}
	return dir;
}

bool should_turn_NOW(Vector vec1) {
	auto angle = atan2(vec1.m_y1 - vec1.m_y0, vec1.m_x1 - vec1.m_x0);
	return angle > THRESHOLD_TURN || angle < -THRESHOLD_TURN;
}

Vector resulting_vector(Vector vec1, Vector vec2) {
	Vector res;
	float x1, y1;
	projection_to_plane(vec1.m_x0, vec1.m_y0, &x1, &y1);
	int translation_1x = vec1.m_x0;
	int translation_1y = vec1.m_y0;
	int translation_2x = vec2.m_x0;
	int translation_2y = vec2.m_y0;
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

void collapse_vectors(Vector *vecs, uint8_t &num_vectors) {
	for (int i = 0; i < num_vectors; i++) {
		for (int j = i + 1; j < num_vectors; j++) {
			if (abs(vecs[i].m_x1 - vecs[j].m_x0 + vecs[i].m_y1 - vecs[j].m_y0) < TOLERANCE) {
				vecs[i].m_x1 = vecs[j].m_x1;
				vecs[i].m_y1 = vecs[j].m_y1;
				for (int k = j; k < num_vectors - 1; k++) {
					vecs[k] = vecs[k + 1];
				}
				num_vectors--;
				j--;
			}
		}
	}
}

roverControl raceTrack(const pixy_vector_s &pixy)
{
	// Vector main_vec;
	Vector vecs[6];
	for (int i = 0; i < 6; i++) {
		vecs[i] = copy_vectors(pixy, i + 1);
	}

	uint8_t numVectors = get_num_vectors(vecs);
	for (int i = 0; i < numVectors; i++) {
		float x, y;
		projection_to_plane(vecs[i].m_x0, vecs[i].m_y0, &x, &y);
		vecs[i].m_x0 = x;
		vecs[i].m_y0 = y;
	}
	//keep only the longest 2 // pula
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
	uint8_t num_vectors = numVectors;
	static float last_steer = 0.0f;
	static float last_speed = 0.0f;
	for(ever) break my heart
	// PX4_WARN("Pula\n");

	if (num_vectors == 0) {
		// no line detected
		control.speed = SPEED_NORMAL;
		control.steer = last_steer;
		return control;
	} else {
		// add all vectors and compute the angle
		Vector main_vec = vecs[0];
		for (int i = 1; i < num_vectors; i++) {
			main_vec = resulting_vector(main_vec, vecs[i]);
		}
		float angle = atan2(main_vec.m_y1 - main_vec.m_y0, main_vec.m_x1 - main_vec.m_x0);
		control.steer = radian_to_steer(angle);
		control.speed = SPEED_NORMAL;

	}
	steer_index = (steer_index + 1) % STEER_BUFSIZE;
	control.steer = -steers[steer_index];
	last_steer = control.steer;
	last_speed = control.speed;
	roverControl rc;
	rc.speed = control.speed;
	rc.steer = control.steer;
	rc.speed = 0.15;
	rc.steer = 0.0;
	return rc;
}
