/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/predict_covariance.h"
#include "utils.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	_delta_angle_bias_var_accum.setZero();
	_delta_vel_bias_var_accum.setZero();

	const float dt = _dt_ekf_avg;

	resetQuatCov();

	// velocity
	const float vel_var = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx, Vector3f(vel_var, vel_var, sq(1.5f) * vel_var));

	// position
	const float xy_pos_var = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	float z_pos_var = sq(fmaxf(_params.baro_noise, 0.01f));

	if (_control_status.flags.gps_hgt) {
		z_pos_var = sq(fmaxf(1.5f * _params.gps_pos_noise, 0.01f));
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		z_pos_var = sq(fmaxf(_params.range_noise, 0.01f));
	}
#endif // CONFIG_EKF2_RANGE_FINDER

	P.uncorrelateCovarianceSetVariance<State::pos.dof>(State::pos.idx, Vector3f(xy_pos_var, xy_pos_var, z_pos_var));

	// gyro bias
	const float delta_ang_var = sq(_params.switch_on_gyro_bias * dt);
	P.uncorrelateCovarianceSetVariance<State::delta_ang_bias.dof>(State::delta_ang_bias.idx, delta_ang_var);
	_prev_delta_ang_bias_var.setAll(delta_ang_var);

	// accel bias
	const float delta_vel_var = sq(_params.switch_on_accel_bias * dt);
	P.uncorrelateCovarianceSetVariance<State::delta_vel_bias.dof>(State::delta_vel_bias.idx, delta_vel_var);
	_prev_dvel_bias_var.setAll(delta_vel_var);

	resetMagCov();

	// wind
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, sq(_params.initial_wind_uncertainty));
}

void Ekf::predictCovariance(const imuSample &imu_delayed)
{
	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = _dt_ekf_avg;
	const float dt_inv = 1.f / dt;

	// convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	const float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

	// convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	const float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	const float beta = 1.0f - alpha;
	_ang_rate_magnitude_filt = fmaxf(dt_inv * imu_delayed.delta_ang.norm(), beta * _ang_rate_magnitude_filt);
	_accel_magnitude_filt = fmaxf(dt_inv * imu_delayed.delta_vel.norm(), beta * _accel_magnitude_filt);
	_accel_vec_filt = alpha * dt_inv * imu_delayed.delta_vel + beta * _accel_vec_filt;

	const bool is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim
					     || _accel_magnitude_filt > _params.acc_bias_learn_acc_lim;

	// gyro bias inhibit
	const bool do_inhibit_all_gyro_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias));

	for (unsigned index = 0; index < State::delta_ang_bias.dof; index++) {
		const unsigned stateIndex = State::delta_ang_bias.idx + index;

		bool is_bias_observable = true;

		// TODO: gyro bias conditions

		const bool do_inhibit_axis = do_inhibit_all_gyro_axes || !is_bias_observable;

		if (do_inhibit_axis) {
			// store the bias state variances to be reinstated later
			if (!_gyro_bias_inhibit[index]) {
				_prev_delta_ang_bias_var(index) = P(stateIndex, stateIndex);
				_gyro_bias_inhibit[index] = true;
			}

		} else {
			if (_gyro_bias_inhibit[index]) {
				// reinstate the bias state variances
				P(stateIndex, stateIndex) = _prev_delta_ang_bias_var(index);
				_gyro_bias_inhibit[index] = false;
			}
		}
	}

	// accel bias inhibit
	const bool do_inhibit_all_accel_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
					 || is_manoeuvre_level_high
					 || _fault_status.flags.bad_acc_vertical;

	for (unsigned index = 0; index < State::delta_vel_bias.dof; index++) {
		const unsigned stateIndex = State::delta_vel_bias.idx + index;

		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// when using fake position (but not fake height) only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		const bool do_inhibit_axis = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;

		if (do_inhibit_axis) {
			// store the bias state variances to be reinstated later
			if (!_accel_bias_inhibit[index]) {
				_prev_dvel_bias_var(index) = P(stateIndex, stateIndex);
				_accel_bias_inhibit[index] = true;
			}

		} else {
			if (_accel_bias_inhibit[index]) {
				// reinstate the bias state variances
				P(stateIndex, stateIndex) = _prev_dvel_bias_var(index);
				_accel_bias_inhibit[index] = false;
			}
		}
	}

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_I_sig;

	if (_control_status.flags.mag_3D && P.trace<State::mag_I.dof>(State::mag_I.idx) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;

	if (_control_status.flags.mag_3D && P.trace<State::mag_B.dof>(State::mag_B.idx) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1.0f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_nsd_scaled;

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && P.trace<State::wind_vel.dof>(State::wind_vel.idx) < sq(_params.initial_wind_uncertainty)) {
		wind_vel_nsd_scaled = math::constrain(_params.wind_vel_nsd, 0.0f, 1.0f) * (1.0f + _params.wind_vel_nsd_scaler * fabsf(_height_rate_lpf));

	} else {
		wind_vel_nsd_scaled = 0.0f;
	}

	// compute noise variance for stationary processes
	Vector24f process_noise;

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	process_noise.slice<State::delta_ang_bias.dof, 1>(State::delta_ang_bias.idx, 0) = sq(d_ang_bias_sig);
	process_noise.slice<State::delta_vel_bias.dof, 1>(State::delta_vel_bias.idx, 0) = sq(d_vel_bias_sig);
	process_noise.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = sq(mag_I_sig);
	process_noise.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = sq(mag_B_sig);
	process_noise.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = sq(wind_vel_nsd_scaled) * dt;

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	const float d_ang_var = sq(dt * gyro_noise);

	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	Vector3f d_vel_var;

	for (unsigned i = 0; i < 3; i++) {
		if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
			// Increase accelerometer process noise if bad accel data is detected
			d_vel_var(i) = sq(dt * BADACC_BIAS_PNOISE);

		} else {
			d_vel_var(i) = sq(dt * accel_noise);
		}
	}

	// predict the covariance
	SquareMatrix24f nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	sym::PredictCovariance(getStateAtFusionHorizonAsVector(), P, imu_delayed.delta_vel, d_vel_var, imu_delayed.delta_ang, d_ang_var, dt, &nextP);

	// process noise contribution for delta angle states can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (unsigned index = 0; index < State::delta_ang_bias.dof; index++) {
		const unsigned i = State::delta_ang_bias.idx + index;

		if (!_gyro_bias_inhibit[index]) {
			// add process noise that is not from the IMU
			// process noise contribution for delta velocity states can be very small compared to
			// the variances, therefore use algorithm to minimise numerical error
			nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_angle_bias_var_accum(index));

		} else {
			nextP.uncorrelateCovarianceSetVariance<1>(i, _prev_delta_ang_bias_var(index));
			_delta_angle_bias_var_accum(index) = 0.f;
		}
	}

	for (unsigned index = 0; index < State::delta_vel_bias.dof; index++) {
		const unsigned i = State::delta_vel_bias.idx + index;

		if (!_accel_bias_inhibit[index]) {
			// add process noise that is not from the IMU
			// process noise contribution for delta velocity states can be very small compared to
			// the variances, therefore use algorithm to minimise numerical error
			nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_vel_bias_var_accum(index));

		} else {
			nextP.uncorrelateCovarianceSetVariance<1>(i, _prev_dvel_bias_var(index));
			_delta_vel_bias_var_accum(index) = 0.f;
		}
	}

	if (_control_status.flags.mag_3D) {
		// add process noise that is not from the IMU
		for (unsigned i = State::mag_I.idx; i < (State::mag_I.idx + State::mag_I.dof); i++) {
			nextP(i, i) += process_noise(i);
		}

		for (unsigned i = State::mag_B.idx; i < (State::mag_B.idx + State::mag_B.dof); i++) {
			nextP(i, i) += process_noise(i);
		}

	} else {
		// keep old covariance
		for (unsigned i = State::mag_B.idx; i < (State::mag_B.idx + State::mag_B.dof); i++) {
			for (unsigned j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}

		for (unsigned i = State::mag_I.idx; i < (State::mag_I.idx + State::mag_I.dof); i++) {
			for (unsigned j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	if (_control_status.flags.wind) {
		// add process noise that is not from the IMU
		for (unsigned i = State::wind_vel.idx; i < (State::wind_vel.idx + State::wind_vel.dof); i++) {
			nextP(i, i) += process_noise(i);
		}

	} else {
		// keep old covariance
		for (unsigned i = State::wind_vel.idx; i < (State::wind_vel.idx + State::wind_vel.dof); i++) {
			for (unsigned j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if (P.trace<2>(State::pos.idx) > 1e4f) {
		for (unsigned i = State::pos.idx; i <= (State::pos.idx + 1); i++) {
			for (unsigned j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P(row, column) = P(column, row) = nextP(column, row);
		}

		P(row, row) = nextP(row, row);
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);
}

void Ekf::fixCovarianceErrors(bool force_symmetry)
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)
	float P_lim[8] = {};
	P_lim[0] = 1.0f;		// quaternion max var
	P_lim[1] = 1e6f;		// velocity max var
	P_lim[2] = 1e6f;		// position max var
	P_lim[3] = 1.0f;		// gyro bias max var
	P_lim[4] = 1.0f;		// delta velocity z bias max var
	P_lim[5] = 1.0f;		// earth mag field max var
	P_lim[6] = 1.0f;		// body mag field max var
	P_lim[7] = 1e6f;		// wind max var

	for (unsigned i = State::quat_nominal.idx; i < (State::quat_nominal.idx + State::quat_nominal.dof); i++) {
		// quaternion states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[0]);
	}

	for (unsigned i = State::vel.idx; i < (State::vel.idx + State::vel.dof); i++) {
		// NED velocity states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[1]);
	}

	for (unsigned i = State::pos.idx; i < (State::pos.idx + State::pos.dof); i++) {
		// NED position states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[2]);
	}

	for (unsigned i = State::delta_ang_bias.idx; i < (State::delta_ang_bias.idx + State::delta_ang_bias.dof); i++) {
		// gyro bias states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[3]);
	}

	// force symmetry on the quaternion, velocity and position state covariances
	if (force_symmetry) {
		P.makeRowColSymmetric<State::quat_nominal.dof>(State::quat_nominal.idx);
		P.makeRowColSymmetric<State::vel.dof>(State::vel.idx);
		P.makeRowColSymmetric<State::pos.dof>(State::pos.idx);
		P.makeRowColSymmetric<State::delta_ang_bias.dof>(State::delta_ang_bias.idx); //TODO: needed?
	}

	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if (!_accel_bias_inhibit[0] || !_accel_bias_inhibit[1] || !_accel_bias_inhibit[2]) {
		// Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		const float minSafeStateVar = 1e-9f;
		float maxStateVar = minSafeStateVar;
		bool resetRequired = false;

		for (unsigned axis = 0; axis < State::delta_vel_bias.dof; axis++) {
			const unsigned stateIndex = State::delta_vel_bias.idx + axis;

			if (_accel_bias_inhibit[axis]) {
				// Skip the check for the inhibited axis
				continue;
			}

			if (P(stateIndex, stateIndex) > maxStateVar) {
				maxStateVar = P(stateIndex, stateIndex);

			} else if (P(stateIndex, stateIndex) < minSafeStateVar) {
				resetRequired = true;
			}
		}

		// To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
		// not exceed 100 and the minimum variance must not fall below the target minimum
		// Also limit variance to a maximum equivalent to a 0.1g uncertainty
		const float minStateVarTarget = 5E-8f;
		float minAllowedStateVar = fmaxf(0.01f * maxStateVar, minStateVarTarget);

		for (unsigned axis = 0; axis < State::delta_vel_bias.dof; axis++) {
			const unsigned stateIndex = State::delta_vel_bias.idx + axis;

			if (_accel_bias_inhibit[axis]) {
				// Skip the check for the inhibited axis
				continue;
			}

			P(stateIndex, stateIndex) = math::constrain(P(stateIndex, stateIndex), minAllowedStateVar,
						    sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
		}

		// If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) {
			P.uncorrelateCovariance<State::delta_vel_bias.dof>(State::delta_vel_bias.idx);
		}

		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		const float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		const float down_dvel_bias = _state.delta_vel_bias.dot(Vector3f(_R_to_earth.row(2)));

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = false;
		if (fabsf(down_dvel_bias) > dVel_bias_lim) {

			bool bad_vz_gps = _control_status.flags.gps    && (down_dvel_bias * _aid_src_gnss_vel.innovation[2] < 0.0f);
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
			bool bad_vz_ev  = _control_status.flags.ev_vel && (down_dvel_bias * _aid_src_ev_vel.innovation[2] < 0.0f);
#else
			bool bad_vz_ev  = false;
#endif // CONFIG_EKF2_EXTERNAL_VISION

			if (bad_vz_gps || bad_vz_ev) {
				bool bad_z_baro = _control_status.flags.baro_hgt && (down_dvel_bias * _aid_src_baro_hgt.innovation < 0.0f);
				bool bad_z_gps  = _control_status.flags.gps_hgt  && (down_dvel_bias * _aid_src_gnss_hgt.innovation < 0.0f);

#if defined(CONFIG_EKF2_RANGE_FINDER)
				bool bad_z_rng  = _control_status.flags.rng_hgt  && (down_dvel_bias * _aid_src_rng_hgt.innovation  < 0.0f);
#else
				bool bad_z_rng  = false;
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				bool bad_z_ev   = _control_status.flags.ev_hgt   && (down_dvel_bias * _aid_src_ev_hgt.innovation   < 0.0f);
#else
				bool bad_z_ev   = false;
#endif // CONFIG_EKF2_EXTERNAL_VISION

				if (bad_z_baro || bad_z_gps || bad_z_rng || bad_z_ev) {
					bad_acc_bias = true;
				}
			}
		}

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_delayed_us;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(_time_acc_bias_check, (uint64_t)7e6)) {

			P.uncorrelateCovariance<State::delta_vel_bias.dof>(State::delta_vel_bias.idx);

			_time_acc_bias_check = _time_delayed_us;
			_fault_status.flags.bad_acc_bias = false;
			_warning_events.flags.invalid_accel_bias_cov_reset = true;
			ECL_WARN("invalid accel bias - covariance reset");

		} else if (force_symmetry) {
			// ensure the covariance values are symmetrical
			P.makeRowColSymmetric<State::delta_vel_bias.dof>(State::delta_vel_bias.idx);
		}

	}

	// magnetic field states
	if (!_control_status.flags.mag_3D) {
		zeroMagCov();

	} else {
		// constrain variances
		for (unsigned i = State::mag_I.idx; i < (State::mag_I.idx + State::mag_I.dof); i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[5]);
		}

		for (unsigned i = State::mag_B.idx; i < (State::mag_B.idx + State::mag_B.dof); i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[6]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<State::mag_I.dof>(State::mag_I.idx);
			P.makeRowColSymmetric<State::mag_B.dof>(State::mag_B.idx);
		}

	}

	// wind velocity states
	if (!_control_status.flags.wind) {
		P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, 0.0f);

	} else {
		// constrain variances
		for (unsigned i = State::wind_vel.idx; i < (State::wind_vel.idx + State::wind_vel.dof); i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[7]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<State::wind_vel.dof>(State::wind_vel.idx);
		}
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP)
{
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);
			healthy = false;
		}
	}

	return healthy;
}

void Ekf::resetMagRelatedCovariances()
{
	resetQuatCov();
	resetMagCov();
}

void Ekf::resetQuatCov()
{
	zeroQuatCov();

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var.setAll(sq(_params.initial_tilt_err));

	initialiseQuatCovariances(rot_vec_var);

	// update the yaw angle variance using the variance of the measurement
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		// using magnetic heading tuning parameter
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}
}

void Ekf::zeroQuatCov()
{
	// Optimization: avoid the creation of a <4> function
	P.uncorrelateCovarianceSetVariance<2>(State::quat_nominal.idx, 0.0f);
	P.uncorrelateCovarianceSetVariance<2>(State::quat_nominal.idx + 2, 0.0f);
}

void Ekf::resetMagCov()
{
	// reset the corresponding rows and columns in the covariance matrix and
	// set the variances on the magnetic field states to the measurement variance
	clearMagCov();

	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, sq(_params.mag_noise));
	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, sq(_params.mag_noise));

	if (!_control_status.flags.mag_3D) {
		// save covariance data for re-use when auto-switching between heading and 3-axis fusion
		// if already in 3-axis fusion mode, the covariances are automatically saved when switching out
		// of this mode
		saveMagCovData();
	}
}

void Ekf::clearMagCov()
{
	zeroMagCov();
	_mag_decl_cov_reset = false;
}

void Ekf::zeroMagCov()
{
	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, 0.0f);
	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, 0.0f);
}

void Ekf::resetZDeltaAngBiasCov()
{
	const float init_delta_ang_bias_var = sq(_params.switch_on_gyro_bias * _dt_ekf_avg);

	P.uncorrelateCovarianceSetVariance<1>(State::delta_ang_bias.idx + 2, init_delta_ang_bias_var);
}
