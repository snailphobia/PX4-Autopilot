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
 * @file hello_example.h
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */
#ifndef NXPCUP_RACE_
#define NXPCUP_RACE_

#include <px4_defines.h>
#include <uORB/topics/pixy_vector.h>

#define SPEED_FAST	0.3f
#define SPEED_NORMAL	0.2f
#define SPEED_SLOW	0.1f
#define SPEED_STOP	0.0f

struct roverControl {
	float steer;
	float speed;
};

struct _vector {
	float x;
	float y;
	float norm;
	float grad;
};

struct Vector
{
	void print()
	{
		char buf[64];
		sprintf(buf, "vector: (%d %d) (%d %d)", m_x0, m_y0, m_x1, m_y1);
		printf(buf);
		printf("\n");
	}

	uint8_t m_x0;
	uint8_t m_y0;
	uint8_t m_x1;
	uint8_t m_y1;
};

roverControl raceTrack(const pixy_vector_s &pixy);
uint8_t get_num_vectors(Vector &vec1, Vector &vec2);
Vector copy_vectors(pixy_vector_s &pixy, uint8_t num);

#endif
