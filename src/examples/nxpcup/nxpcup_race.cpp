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

#define ANGLE_UNIT 		4.f

#define THRESHOLD_STRAIGHT 	M_PI_2 / 5.f
#define THRESHOLD_STRAIGHT2 	(M_PI_2 * 2.f / 3.f)

#define THRESHOLD_TURN 		M_PI_2 / 3.f

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
	if (angle < M_PI_2 - THRESHOLD_STRAIGHT) {
		dir = RIGHT;
	} else if (angle > M_PI_2 + THRESHOLD_STRAIGHT) {
		dir = LEFT;
	} else {
		dir = STRAIGHT;
	}
	return dir;
}

direction_e get_direction(Vector_F vec1) {
	auto angle = atan2(vec1.m_y1 - vec1.m_y0, vec1.m_x1 - vec1.m_x0);
	direction_e dir = STRAIGHT;
	float th1 = THRESHOLD_STRAIGHT;
	float th2 = THRESHOLD_STRAIGHT2;
	if (angle < M_PI_2 - THRESHOLD_STRAIGHT) {
 		dir = RIGHT;
	} else if (angle > M_PI_2 + THRESHOLD_STRAIGHT) {
		dir = LEFT;
	} else {
		dir = STRAIGHT;
	}
	return dir;

}

bool should_turn_NOW(Vector vec1) {
	auto angle = atan2(vec1.m_y1 - vec1.m_y0, vec1.m_x1 - vec1.m_x0);
	return angle > M_PI_2 + THRESHOLD_TURN || angle < M_PI_2 - THRESHOLD_TURN;
}

bool should_turn_NOW(Vector_F vec1) {
	auto angle = atan2(vec1.m_y1 - vec1.m_y0, vec1.m_x1 - vec1.m_x0);
	return angle > M_PI_2 + THRESHOLD_TURN || angle < M_PI_2 - THRESHOLD_TURN;
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

Vector_F resulting_vector(Vector_F vec1, Vector_F vec2) {
	Vector_F res;
	float x1, y1;
	float translation_1x = vec1.m_x0;
	float translation_1y = vec1.m_y0;
	float translation_2x = vec2.m_x0;
	float translation_2y = vec2.m_y0;
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

void collapse_vectors(Vector_F *vecs, uint8_t &num_vectors) {
	for (int i = 0; i < num_vectors; i++) {
		for (int j = i + 1; j < num_vectors; j++) {
			if (abs(vecs[i].m_x1 - vecs[j].m_x0 + vecs[i].m_y1 - vecs[j].m_y0) < TOLERANCE_CLPS) {
				auto comp = vecs[i];
				comp.m_x1 = vecs[j].m_x1;
				comp.m_y1 = vecs[j].m_y1;
				if (comp.m_x0 > 0 && get_direction(comp) == LEFT ||
					comp.m_x0 < 0 && get_direction(comp) == RIGHT) {
					vecs[i] = comp;
					for (int k = j; k < num_vectors - 1; k++) {
						vecs[k] = vecs[k + 1];
					}
					num_vectors--;
					j--;
				}
				// vecs[i].m_x1 = vecs[j].m_x1;
				// vecs[i].m_y1 = vecs[j].m_y1;
				// for (int k = j; k < num_vectors - 1; k++) {
				// 	vecs[k] = vecs[k + 1];
				// }
				// num_vectors--;
				// j--;
			}
		}
	}

}

/**
 * rotate the vector by a given angle (in degrees)
*/
Vector_F rotate_by_value(Vector_F vec, float angle) {
	Vector_F res;
	angle = angle / 180.f * M_PI;
	auto start_point_x = vec.m_x0, start_point_y = vec.m_y0;
	vec.m_x0 -= start_point_x; vec.m_y0 -= start_point_y;
	vec.m_x1 -= start_point_x; vec.m_y1 -= start_point_y;

	res.m_x0 = vec.m_x0 * cos(angle) - vec.m_y0 * sin(angle);
	res.m_y0 = vec.m_x0 * sin(angle) + vec.m_y0 * cos(angle);
	res.m_x1 = vec.m_x1 * cos(angle) - vec.m_y1 * sin(angle);
	res.m_y1 = vec.m_x1 * sin(angle) + vec.m_y1 * cos(angle);

	res.m_x0 += start_point_x; res.m_y0 += start_point_y;
	res.m_x1 += start_point_x; res.m_y1 += start_point_y;
	return res;
}

Vector_F tribute_to_benjamin_spaghetti(Vector_F vec1, Vector_F vec2) {
	/* check if the 2 vectors are such that 1.x0 is approximately as much as 2.x1
	if yes, return the vector starting in 1.(x0, y0) and ending in 2.(x1, y1)
	else, return a 0 vector */
	Vector_F res = {};
	auto original_vec1 = vec1;
	auto original_vec2 = vec2;
	for (int i = -2; i < 2; i++) {
		vec1 = rotate_by_value(original_vec1, ANGLE_UNIT * (float)i);
		vec2 = rotate_by_value(original_vec2, ANGLE_UNIT * (float)i);
		if (abs(vec1.m_x0 - vec2.m_x1) < TOLERANCE_PRJ) {
			res.m_x0 = original_vec1.m_x0;
			res.m_y0 = original_vec1.m_y0;
			res.m_x1 = original_vec2.m_x1;
			res.m_y1 = original_vec2.m_y1;
			return res;
		}
	}

	// if (abs(vec1.m_x1 - vec2.m_x0) < TOLERANCE_PRJ) {
	// 	res.m_x0 = vec1.m_x0;
	// 	res.m_y0 = vec1.m_y0;
	// 	res.m_x1 = vec2.m_x1;
	// 	res.m_y1 = vec2.m_y1;
	// }

	return res;
}


void remodel_vectors_using_BS(Vector_F *vecs, uint8_t &num_vectors) {
	for (int i = 0; i < num_vectors; i++) {
		for (int j = i + 1; j < num_vectors; j++) {
			Vector_F res = tribute_to_benjamin_spaghetti(vecs[i], vecs[j]);
			if (res.m_x0 != 0 || res.m_x1 != 0 || res.m_y0 != 0 || res.m_y1 != 0) {
				vecs[i] = res;
				for (int k = j; k < num_vectors - 1; k++) {
					vecs[k] = vecs[k + 1];
				}
				num_vectors--;
				j--;
			}
		}
	}
}

void remove_integrally_topside_vectors(Vector *vecs, uint8_t &num_vectors) {
	for (int i = 0; i < num_vectors; i++) {
		if (vecs[i].m_y0 < 7 && vecs[i].m_y1 < 7) {
			for (int j = i; j < num_vectors - 1; j++) {
				vecs[j] = vecs[j + 1];
			}
			num_vectors--;
			i--;
		}
	}
}

void remove_outer_vectors(Vector_F *vecs, uint8_t &num_vectors) {
	Vector_F vec_left = vecs[0], vec_right = vecs[0];
	for (int i = 1; i < num_vectors; i++) {
		if (vecs[i].m_x0 > vec_left.m_x0 && vecs[i].m_x0 < 0)
			vec_left = vecs[i];
		if (vecs[i].m_x0 < vec_right.m_x0 && vecs[i].m_x0 > 0)
			vec_right = vecs[i];
	}
	uint8_t ctr = 0;
	if (vec_left.m_x0 < 0) {
		ctr++;
		vecs[0] = vec_left;
	}
	if (vec_right.m_x0 > 0) {
		ctr++;
		vecs[1] = vec_right;
	}
	num_vectors = ctr;
}

// static int steer_spion = 0;


roverControl raceTrack(const pixy_vector_s &pixy)
{
	// Vector main_vec;
	Vector vecs[6];
	Vector_F vecsF[6] = {};
	for (int i = 0; i < 6; i++) {
		vecs[i] = copy_vectors(pixy, i + 1);
	}

	uint8_t numVectors = get_num_vectors(vecs);
	uint8_t num_vectors = numVectors;
	collapse_vectors(vecs, num_vectors);
	remove_integrally_topside_vectors(vecs, num_vectors);


	for (int i = 0; i < 6; i++) {
		// flip the vectors if upside down
		if (vecs[i].m_y0 < vecs[i].m_y1) {
			int temp = vecs[i].m_y0;
			vecs[i].m_y0 = vecs[i].m_y1;
			vecs[i].m_y1 = temp;
			temp = vecs[i].m_x0;
			vecs[i].m_x0 = vecs[i].m_x1;
			vecs[i].m_x1 = temp;
		}
	}

	float x0 = 0.f, y0 = 0.f, x1 = 0.f, y1 = 0.f;
	for (int i = 0; i < num_vectors; i++) {
		projection_to_plane(vecs[i].m_x0, vecs[i].m_y0, &x0, &y0);
		projection_to_plane(vecs[i].m_x1, vecs[i].m_y1, &x1, &y1);
		vecsF[i].m_x0 = x0; vecsF[i].m_y0 = y0;
		vecsF[i].m_x1 = x1; vecsF[i].m_y1 = y1;
	}

	remodel_vectors_using_BS(vecsF, num_vectors);
	// remove_outer_vectors(vecsF, num_vectors);

	uint8_t frameWidth = 79;
	uint8_t frameHeight = 52;
	// int16_t window_center = (frameWidth / 2);
	roverControl control{};
	// float x, y;					 // calc gradient and position of main vector
	static hrt_abstime no_line_time = 0;		// time variable for time since no line detected

	hrt_abstime time_diff = 0;
	static bool first_call = true;
	time_diff = hrt_elapsed_time_atomic(&no_line_time);
	static float last_steer = 0.0f;
	static float last_speed = 0.0f;
	// PX4_WARN("Pula\n");

	switch (current_state) {
		case STRAIGHT_STATE: {
			if (num_vectors == 0) {
				control.speed = SPEED_NORMAL;
				control.steer = 0.0f;
			}
			if (num_vectors == 1) {
				// check if vector is on the left or the right of the pov
				direction_e dir = get_direction(vecsF[0]);
				float offset = vecsF[0].m_x0;

				if (dir == LEFT) {
					if (offset > 0) {
						/* if the vector is on the right side, listen to it*/
						last_different_state = STRAIGHT_STATE;
						current_state = TURN_L;
					} else {
						/* if the vector is on the left side,
						try to recenter until there are 2+ vectors */
						control.speed = SPEED_SLOW;
						control.steer = RECENTER_RIGHT;
					}
				} else if (dir == RIGHT) {
					if (offset < 0) {
						/* if the vector is on the left side, listen to it*/
						last_different_state = STRAIGHT_STATE;
						current_state = TURN_R;
					} else {
						/* if the vector is on the right side,
						try to recenter until there are 2+ vectors */
						control.speed = SPEED_SLOW;
						control.steer = RECENTER_LEFT;
					}
				} else {
					control.speed = SPEED_NORMAL;
					// control.steer = atan2(vecsF[0].m_y1 - vecsF[0].m_y0, vecsF[0].m_x1 - vecsF[0].m_x0) - M_PI_2;
					// control.steer = radian_to_steer(control.steer);
					if (last_different_state == TURN_L) {
						control.speed = SPEED_SLOW;
						control.steer = RECENTER_LEFT;
					} else {
						control.speed = SPEED_SLOW;
						control.steer = RECENTER_RIGHT;
					}
				}
			}
			if (num_vectors >= 2) {
				auto res = resulting_vector(vecsF[0], vecsF[1]);
				direction_e dir = get_direction(res);
				if (dir == LEFT) {
					last_different_state = STRAIGHT_STATE;
					current_state = TURN_L;
				} else if (dir == RIGHT) {
					last_different_state = STRAIGHT_STATE;
					current_state = TURN_R;
				} else {
					control.speed = SPEED_NORMAL;
					// control.steer = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0) - M_PI_2;
					// control.steer = radian_to_steer(control.steer);
					float center = 0.f;
					for (int i = 0; i < num_vectors; i++)
						center += vecsF[i].m_x0;
					center /= num_vectors;
					control.steer = -center / 5.f;
				}
			}
			break;
		}
		case TURN_L: {
			if (num_vectors == 0) {
				last_different_state = TURN_L;
				current_state = STRAIGHT_STATE;
			}
			if (num_vectors == 1) {
				direction_e dir = get_direction(vecsF[0]);
				auto offset = vecsF[0].m_x0;
				if (dir == LEFT) {
					if (should_turn_NOW(vecsF[0])) {
						control.speed = SPEED_SLOW;
						control.steer = FULL_LEFT;
					} else {
						control.speed = SPEED_NORMAL;
						control.steer = FULL_LEFT;
						// control.steer = atan2(vecsF[0].m_y1 - vecsF[0].m_y0, vecsF[0].m_x1 - vecsF[0].m_x0) - M_PI_2;
						// control.steer = radian_to_steer(control.steer);
					}
					// if (offset < 0) {
					// 	control.speed = SPEED_SLOW;
					// 	control.steer = HALF_LEFT;
					// } else {
					// 	if (should_turn_NOW(vecsF[0])) {
					// 		control.speed = SPEED_SLOW;
					// 		control.steer = FULL_LEFT;
					// 	} else {
					// 		control.speed = SPEED_SLOW;
					// 		control.steer = FULL_LEFT;
					// 		// control.steer = atan2(vecsF[0].m_y1 - vecsF[0].m_y0, vecsF[0].m_x1 - vecsF[0].m_x0) - M_PI_2;
					// 	// if (!should_turn_NOW(vecsF[0]))
					// 	// 	control.steer /= 1.25f;
					// 		// control.steer = radian_to_steer(control.steer);
					// 	}
					// }
				} else {
					last_different_state = TURN_L;
					current_state = STRAIGHT_STATE;
				}
			}
			if (num_vectors >= 2) {
				auto res = resulting_vector(vecsF[0], vecsF[1]);
				for (int i = 2; i < num_vectors; i++) {
					res = resulting_vector(res, vecsF[i]);
				}
				direction_e dir = get_direction(res);
				if (dir == LEFT) {
					// control.steer = radian_to_steer(control.steer);
					if (should_turn_NOW(res)) {
						control.speed = SPEED_SLOW;
						control.steer = FULL_LEFT;
					} else {
						control.speed = SPEED_SLOW;
						control.steer = FULL_LEFT;
						// control.steer = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0) - M_PI_2;
					}
					// if (!should_turn_NOW(res))
					// 	control.steer /= 1.25f;
				} else if (dir == RIGHT) {
					control.speed = SPEED_SLOW;
					control.steer = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0) - M_PI_2;
					// control.steer = radian_to_steer(control.steer);
					last_different_state = TURN_L;
					current_state = TURN_R;
				} else {
					last_different_state = TURN_L;
					current_state = STRAIGHT_STATE;
				}
			}
			break;
		}
		case TURN_R: {
			if (num_vectors == 0) {
				last_different_state = TURN_R;
				current_state = STRAIGHT_STATE;
			}
			if (num_vectors == 1) {
				direction_e dir = get_direction(vecsF[0]);
				if (dir == RIGHT) {
					auto offset = vecsF[0].m_x0;
					{
						if (should_turn_NOW(vecsF[0])) {
							control.speed = SPEED_SLOW;
							control.steer = FULL_RIGHT;
						} else {
							control.speed = SPEED_SLOW;
							control.steer = FULL_RIGHT;
							// control.steer = atan2(vecsF[0].m_y1 - vecsF[0].m_y0, vecsF[0].m_x1 - vecsF[0].m_x0) - M_PI_2;
						// if (!should_turn_NOW(vecsF[0]))
						// 	control.steer /= 1.25f;
							// control.steer = radian_to_steer(control.steer);
						}
					}
				} else {
					last_different_state = TURN_R;
					current_state = STRAIGHT_STATE;
				}
			}
			if (num_vectors >= 2) {
				auto res = resulting_vector(vecsF[0], vecsF[1]);
				for (int i = 2; i < num_vectors; i++) {
					res = resulting_vector(res, vecsF[i]);
				}
				direction_e dir = get_direction(res);
				if (dir == RIGHT) {
					if (should_turn_NOW(res)) {
						control.speed = SPEED_SLOW;
						control.steer = FULL_RIGHT;
					} else {
						control.speed = SPEED_SLOW;
						control.steer = FULL_RIGHT;
						// control.steer = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0) - M_PI_2;
					}
					// control.steer = radian_to_steer(control.steer);
					// if (!should_turn_NOW(res))
					// 	control.steer /= 1.25f;
				} else if (dir == LEFT) {
					control.speed = SPEED_SLOW;
					control.steer = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0) - M_PI_2;
					// control.steer = radian_to_steer(control.steer);
					last_different_state = TURN_R;
					current_state = TURN_L;
				} else {
					last_different_state = TURN_R;
					current_state = STRAIGHT_STATE;
				}
			}
			break;
		}
	}


	// control.steer = -control.steer;
	// control.steer = FULL_RIGHT;
	// control.steer = FULL_LEFT;
	// control.steer = 2.5f * control.steer;
	// if (steer_spion == 0) {
	// 	control.steer = -0.2f;
	// 	steer_spion = 1;
	// 	sleep(2);
	// } else if (steer_spion == 1) {
	// 	steer_spion = 2;
	// 	control.steer = -0.4f;
	// 	sleep(2);
	// } else if (steer_spion == 2) {
	// 	steer_spion = 3;
	// 	control.steer = -0.6f;
	// 	sleep(2);
	// } else if (steer_spion == 3) {
	// 	steer_spion = 4;
	// 	control.steer = -0.8f;
	// 	sleep(2);
	// } else if (steer_spion == 4) {
	// 	steer_spion = 0;
	// 	control.steer = -1.f;
	// 	sleep(2);
	// }

	auto spion45 = atan2(1.f, 1.f);
	auto spion90 = atan2(0.f, 1.f);
	auto spion0 = atan2(0.f, 0.f);
	auto spion135 = atan2(1.f, -1.f);
	auto spion180 = atan2(0.f, -1.f);
	printf("%f", spion45); printf("%f", spion90); printf("%f", spion0); printf("%f", spion135); printf("%f", spion180);
	control.speed = SPEED_SLOW;
	last_steer = control.steer;
	last_speed = control.speed;
	roverControl rc;
	rc.speed = control.speed;
	rc.steer = control.steer;
	return rc;
}
