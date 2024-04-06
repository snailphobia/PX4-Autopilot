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
#define THRESHOLD_STRAIGHT2 	(M_PI_2 / 5.f + M_PI_2)

#define THRESHOLD_TURN 		M_PI_4

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
	STRAIGHT_LINE,
	PREPARE_TURN,
	TURN,
	RECENTER,
	CROSSROAD,
	NONE
} state_e;

static state_e last_different_state = NONE;
static state_e current_state = NONE;

typedef enum {
	LEFT = 0,
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
	direction_e dir = STRAIGHT;
	float th1 = THRESHOLD_STRAIGHT;
	float th2 = THRESHOLD_STRAIGHT2;
	if (angle > THRESHOLD_STRAIGHT && angle < THRESHOLD_STRAIGHT2) {
		dir = RIGHT;
	} else if (angle < -THRESHOLD_STRAIGHT && angle > -THRESHOLD_STRAIGHT2) {
		dir = LEFT;
	} else {
		dir = STRAIGHT;
	}
	return dir;;;;;
}

bool should_turn(Vector vec1) {
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
	float x0,y0,x1,y1;
	projection_to_plane(vecs[0].m_x0, vecs[0].m_y0, &x0, &y0);
	projection_to_plane(vecs[0].m_x1, vecs[0].m_y1, &x1, &y1);

	uint8_t numVectors = get_num_vectors(vecs);
	//keep only the longest 2
	Vector vecs2[2];
	float max_norms[2] = {0};
	for (int i = 0; i < numVectors; i++) {
		float norm = sqrt((vecs[i].m_x1 - vecs[i].m_x0) * (vecs[i].m_x1 - vecs[i].m_x0) + (vecs[i].m_y1 - vecs[i].m_y0) * (vecs[i].m_y1 - vecs[i].m_y0));
		if (norm > max_norms[0]) {
			max_norms[1] = max_norms[0];
			vecs2[1] = vecs2[0];
			max_norms[0] = norm;
			vecs2[0] = vecs[i];
		} else if (norm > max_norms[1]) {
			max_norms[1] = norm;
			vecs2[1] = vecs[i];
		}
	}
	for (int i = 0; i < 6; i++) {
		vecs[i] = {0, 0, 0, 0};
	}
	for (int i = 0; i < 2; i++){
		vecs[i] = vecs2[i];
	}

	numVectors = 2;

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
	collapse_vectors(vecs, num_vectors);
	time_diff = hrt_elapsed_time_atomic(&no_line_time);
	static float last_steer = 0.0f;
	static float last_speed = 0.0f;
	for(ever) break my heart
	// PX4_WARN("Pula\n");
	// int spion = 0;
	switch(current_state) {
		case NONE: {
			if (num_vectors == 0) {
				if (first_call) {
					no_line_time = hrt_absolute_time();
					first_call = false;
				} else {
					control.speed = SPEED_NORMAL;
					control.steer = last_steer;
					if (time_diff > 3000) {
						/* Stopping if no vector is available */
						control.steer = 0.0f;
						control.speed = SPEED_STOP;
					}
				}
				current_state = NONE;
			}
			if (num_vectors >= 1) {
				first_call = true;
				auto res = vecs[0];
				for (int i = 1; i < num_vectors; i++) {
					res = resulting_vector(res, vecs[i]);
				}
				float th1 = THRESHOLD_STRAIGHT;
				float th2 = THRESHOLD_STRAIGHT2;
				direction_e dir = get_direction(res);
				auto angle = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0);
				if (dir != STRAIGHT) {
					control.speed = SPEED_SLOW;
					control.steer = dir == LEFT ? 1.0f : -1.0f;
					current_state = PREPARE_TURN;
				} else {
					control.speed = SPEED_NORMAL;
					control.steer = atan2(res.m_y1 - res.m_y0, res.m_x1 - res.m_x0);
					if (num_vectors >= 2)
						current_state = RECENTER;
					else
						current_state = STRAIGHT_LINE;
				}
			}
			break;
		}
		case PREPARE_TURN: {
			if (num_vectors == 0) {
				if (first_call) {
					no_line_time = hrt_absolute_time();
					first_call = false;
				} else {
					control.speed = SPEED_NORMAL;
					control.steer = last_steer;
					if (time_diff > 3000) {
						/* Stopping if no vector is available */
						control.steer = 0.0f;
						control.speed = SPEED_STOP;
					}
				}
			}
			if (num_vectors == 1) {
CASE_PREP_TURN_ONE_VECTOR:;
				auto res = vecs[0];
				direction_e dir = get_direction(res);
				auto offset = one_vector_offset_from_center(res, frameWidth);
				if (dir == STRAIGHT) {
					// sanki, should not happen
				}
				if (dir == LEFT) {
					// close in to the right side of the track
					// if the vector is on the left, you will still go to the right until the right vector is visible
					if (offset < 0) {
						// vector is on the left, and signals to turn left
						// we can assume it is a crossroad, going straight
						// for now, until new input, which should occur shortly
						last_different_state = PREPARE_TURN;
						current_state = CROSSROAD;
						control.speed = SPEED_NORMAL;
						control.steer = STRAIGHT_;
					} else {
						if (should_turn(res)) {
							// we are going to turn LEFT, the vector is rightmost
							// we should turn start turning left
							last_different_state = PREPARE_TURN;
							current_state = TURN;
							control.steer = FULL_LEFT;
							control.speed = SPEED_SLOW;
						} else {
							// do not start turning yet
							// move alongside rightmost vector
							control.steer = (CAR_HALF_WIDTH - offset) / SCALING_FACTOR;
							// scale speed with amount of turning
							control.speed = SPEED_NORMAL * (1.f - abs(control.steer / 2.f));
						}
					}
				}
				// symmetrically for the right dir
				if (dir == RIGHT) {
					if (offset > 0) {
						// vector is on the right, and signals to turn right
						// we can assume it is a crossroad, going straight
						// for now, until new input, which should occur shortly
						last_different_state = PREPARE_TURN;
						current_state = CROSSROAD;
						control.speed = SPEED_NORMAL;
						control.steer = STRAIGHT_;
					} else {

						if (should_turn(res)) {
							// we are going to turn RIGHT, the vector is leftmost
							// we should start turning right
							last_different_state = PREPARE_TURN;
							current_state = TURN;
							control.steer = FULL_RIGHT;
							control.speed = SPEED_SLOW;
						} else {
							// do not start turning yet
							// move alongside leftmost vector
							control.steer = (CAR_HALF_WIDTH + offset) / SCALING_FACTOR;
							// scale speed with amount of turning
							control.speed = SPEED_NORMAL * (1.f - abs(control.steer / 2.f));
						}
					}
				}
			}
			if (num_vectors >= 2) {
				// select the most viable vector; the one that is opposite to the turn and starts as close to the camera as possible
				int selected = 0;
				// keep only the vectors that start from the bottom of the frame (discard the far away ones)
				int nb = 0, j = 0;
				Vector bottom[6];
				for (int i = 0; i < num_vectors; i++) {
					if (vecs[i].m_y0 > BOT)
						bottom[j++] = vecs[i];
				}
				nb = j;

				Vector res = bottom[0];
				for (int i = 1; i < nb; i++) {
					res = resulting_vector(res, bottom[i]);
				}
				direction_e dir = get_direction(res);

				if (dir == LEFT) {
					// select the rightmost vector
					for (int i = 1; i < nb; i++) {
						if (bottom[i].m_x0 > bottom[selected].m_x0 && bottom[i].m_x0 > frameWidth / 2)
							selected = i;
					}
				}
				if (dir == RIGHT) {
					// select the leftmost vector
					for (int i = 1; i < nb; i++) {
						if (bottom[i].m_x0 < bottom[selected].m_x0 && bottom[i].m_x0 < frameWidth / 2)
							selected = i;
					}
				}


				// vector is selected, keep only that one and discard the rest
				num_vectors = 1;
				vecs[0] = bottom[selected];
				goto CASE_PREP_TURN_ONE_VECTOR;
			}
			break;
		}
		case STRAIGHT_LINE: {
			if (num_vectors == 0) {
				last_different_state = STRAIGHT_LINE;
				current_state = NONE;
			}
			if (num_vectors == 1) {
				auto dir = get_direction(vecs[0]);
				if (dir == STRAIGHT) {
					control.speed = SPEED_NORMAL;
					control.steer = atan2(vecs[0].m_y1 - vecs[0].m_y0, vecs[0].m_x1 - vecs[0].m_x0);
				} else {
					last_different_state = STRAIGHT_LINE;
					current_state = PREPARE_TURN;
				}
			}
			if (num_vectors >= 2) {
				last_different_state = STRAIGHT_LINE;
				current_state = RECENTER;
			}
		}
	}

// 	switch (num_vectors) {
// 	case 0:{
// 		if(first_call){
// 			no_line_time = hrt_absolute_time();
// 			first_call = false;
// 		}else{
// 			time_diff = hrt_elapsed_time_atomic(&no_line_time);
// 			if(!spion) {
// 				control.steer = 2*last_steer;
// 				control.speed = 0.5f*last_speed;
// 				spion = 1;
// 			}

// 			if(time_diff > 10000){
// 				/* Stopping if no vector is available */
// 				control.steer = 0.0f;
// 				control.speed = SPEED_STOP;
// 			}
// 		}
// 		break;
// 	}
// 	case 2:{
// 		first_call = true;
// 		spion = 0;

// 		/* Very simple steering angle calculation, get average of the x of top two points and
// 		   find distance from center of frame */
// 		// main_vec.m_x1 = (vecs[0].m_x1 + vecs[1].m_x1) / 2;
// 		// control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;
// 		int8_t resx = vecs[0].m_x1 - vecs[0].m_x0, res2x = vecs[1].m_x1 - vecs[1].m_x0;
// 		int8_t resy = vecs[0].m_y1 - vecs[0].m_y0, res2y = vecs[1].m_y1 - vecs[1].m_y0;
// 		VectorF vec1norn = {resx,resy};
// 		VectorF vec2norn = {res2x,res2y};
// 		VectorF avg = {(vec1norn.m_x0 + vec2norn.m_x0)/2,-((double)vec1norn.m_y0 + vec2norn.m_y0)/2.0};
// 		VectorF avgnorm = {avg.m_x0/(sqrt(avg.m_x0*avg.m_x0 + avg.m_y0*avg.m_y0)),avg.m_y0/(sqrt(avg.m_x0*avg.m_x0 + avg.m_y0*avg.m_y0))};

// 		// angle between avg and oy

// 		steers[steer_index] = avgnorm.m_x0;
// 		// float f = resx / (sqrt(resy * resy + resx * resx));
// 		// float f2 = res2x / (sqrt(res2y * res2y + res2x * res2x));
// 		// float angle = (acos(f) + acos(f2))/2 - M_PI;
// 		// float medcos = cos(angle);
// 		// control.steer = medcos;



// 		control.speed = SPEED_NORMAL;

//  		break;
// 	}
// 	default: {
// 		first_call = true;
// 		spion = 0;
// 		//check if vecs[0] is the middle of the screen
// 		/* Following the main vector */
// 		// int8_t vecs[0]_right = vecs[0].m_x1 > (frameWidth / 2) ? -1 : 1;
// 		int8_t resx = vecs[0].m_x1 - vecs[0].m_x0;
// 		int8_t resy = vecs[0].m_y1 - vecs[0].m_y0;
// 		VectorF vecnorm = {resx/(sqrt(resy * resy + resx * resx)),resy/(sqrt(resy * resy + resx * resx))};
// 		steers[steer_index] = vecnorm.m_x0;

// end:
// 		control.speed = SPEED_NORMAL;
// 		break;
// 	}
// 	}
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
