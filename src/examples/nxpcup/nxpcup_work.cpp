/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "nxpcup_work.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

NxpCupWork::NxpCupWork() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

NxpCupWork::~NxpCupWork()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool NxpCupWork::init()
{
	ScheduleOnInterval(50_ms); // 1000 us interval, 1000 Hz rate

	return true;
}

void NxpCupWork::roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp, vehicle_attitude_s &att)
{
	// Converting steering value from percent to euler angle
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159/180; // change to radians

	// Get current attitude quaternion
	matrix::Quatf current_qe{att.q[0], att.q[1], att.q[2], att.q[3]};

	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	matrix::Eulerf euler{0.0, 0.0, control.steer};
	matrix::Quatf qe{euler};

	// Create new quaternion from the difference of current vs steering
	matrix::Quatf new_qe;
	new_qe = current_qe * qe.inversed();

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	new_qe.copyTo(_att_sp.q_d);

}

void NxpCupWork::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// DO WORK
	roverControl motorControl;

	struct vehicle_control_mode_s _control_mode{};

	_control_mode.flag_control_manual_enabled = false;
	_control_mode.flag_control_attitude_enabled = true;
	_control_mode.flag_control_velocity_enabled = false;
	_control_mode.flag_control_position_enabled = false;

	/* Get pixy data */
	pixy_sub.update();
	const pixy_vector_s &pixy = pixy_sub.get();

	motorControl = raceTrack(pixy);

	att_sub.update();
	struct vehicle_attitude_s att = att_sub.get();

	struct vehicle_attitude_setpoint_s _att_sp{};

	NxpCupWork::roverSteerSpeed(motorControl, _att_sp, att);

	_control_mode.timestamp = hrt_absolute_time();
	_control_mode_pub.publish(_control_mode);
	_att_sp.timestamp = hrt_absolute_time();
	_att_sp_pub.publish(_att_sp);

	perf_end(_loop_perf);
}

int NxpCupWork::task_spawn(int argc, char *argv[])
{
	NxpCupWork *instance = new NxpCupWork();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int NxpCupWork::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NxpCupWork::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NxpCupWork::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nxpcup_work_main(int argc, char *argv[])
{
	return NxpCupWork::main(argc, argv);
}
