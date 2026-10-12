/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "Intercept.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

Intercept::Intercept() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

Intercept::~Intercept()
{
	if (_sent_mode_registration) {
		UnregisterFlightMode();
	}
}

bool Intercept::init()
{
	ScheduleOnInterval(20_ms); // 50 Hz
	return true;
}

void Intercept::RegisterFlightMode()
{
	register_ext_component_request_s req{};
	req.timestamp = hrt_absolute_time();
	strncpy(req.name, "Intercept", sizeof(req.name) - 1);
	req.request_id = _mode_request_id;
	req.px4_ros2_api_version = 1;
	req.register_arming_check = true;
	req.register_mode = true;
	req.register_mode_executor = false;
	req.enable_replace_internal_mode = false;
	req.activate_mode_immediately = false;
	_register_ext_component_request_pub.publish(req);
}

void Intercept::UnregisterFlightMode()
{
	unregister_ext_component_s unregister{};
	unregister.timestamp = hrt_absolute_time();
	strncpy(unregister.name, "Intercept", sizeof(unregister.name) - 1);
	unregister.arming_check_id = _arming_check_id;
	unregister.mode_id = _mode_id;
	unregister.mode_executor_id = -1;
	_unregister_ext_component_pub.publish(unregister);
}

void Intercept::CheckModeRegistration()
{
	register_ext_component_reply_s reply;
	int tries = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&reply) && --tries >= 0) {
		if (reply.request_id == _mode_request_id && reply.success) {
			_arming_check_id = reply.arming_check_id;
			_mode_id = reply.mode_id;
			PX4_INFO("Intercept mode registered: arming_check_id=%d, mode_id=%d", _arming_check_id, _mode_id);
			break;
		}
	}
}

void Intercept::ReplyToArmingCheck(uint8_t request_id)
{
	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request_id;
	reply.registration_id = _arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.num_events = 0;
	reply.can_arm_and_run = true;
	reply.mode_req_angular_velocity = false;
	reply.mode_req_attitude = true;
	reply.mode_req_local_alt = true;
	reply.mode_req_local_position = true;
	reply.mode_req_local_position_relaxed = false;
	reply.mode_req_global_position = false;
	reply.mode_req_global_position_relaxed = false;
	reply.mode_req_mission = false;
	reply.mode_req_home_position = false;
	reply.mode_req_prevent_arming = false;
	reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(reply);
}

void Intercept::UpdateTarget()
{
	// 1. Maintain local map projection reference from local position
	if (_local_pos.xy_global && _local_pos.z_global) {
		if (!_map_ref.isInitialized()
		    || _map_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp) {
			_map_ref.initReference(_local_pos.ref_lat, _local_pos.ref_lon, _local_pos.ref_timestamp);
		}
	}

	// 2. Read new follow_target messages from relay
	if (_follow_target_sub.updated()) {
		follow_target_s ft;

		if (_follow_target_sub.copy(&ft)) {
			const double target_lat = ft.lat;
			const double target_lon = ft.lon;

			if (_map_ref.isInitialized() && PX4_ISFINITE(target_lat) && PX4_ISFINITE(target_lon)) {
				float x{0.f};
				float y{0.f};
				_map_ref.project(target_lat, target_lon, x, y);

				_target_pos(0) = x;
				_target_pos(1) = y;
				_target_pos(2) = -(ft.alt - _local_pos.ref_alt);

				_target_vel(0) = ft.vx;
				_target_vel(1) = ft.vy;
				_target_vel(2) = ft.vz;

				_target_pos_valid = true;
				_last_target_update = hrt_absolute_time();
			}
		}
	}

	// 3. Invalidate target if no update for more than 3 seconds
	if (_target_pos_valid && (hrt_elapsed_time(&_last_target_update) > 3_s)) {
		_target_pos_valid = false;
	}
}

void Intercept::UpdateVisualDetection()
{
	target_detection_s td;

	if (_target_detection_sub.update(&td)) {
		if (td.detected) {
			if (!_visual_contact) {
				PX4_INFO("Visual contact acquired! Range: %.1f m, bbox: [%.2f, %.2f, %.2f, %.2f]",
					 (double)td.range_m, (double)td.bbox[0], (double)td.bbox[1],
					 (double)td.bbox[2], (double)td.bbox[3]);
			}

			_visual_contact = true;
			_visual_range = td.range_m;
			_target_detection = td;
			_last_visual_contact = hrt_absolute_time();
		}
	}

	const float lost_timeout_s = math::max(_param_int_lost_timeout.get(), 0.2f);
	const uint64_t lost_timeout_us = (uint64_t)(lost_timeout_s * 1e6f);

	if (_visual_contact && (hrt_elapsed_time(&_last_visual_contact) > lost_timeout_us)) {
		PX4_WARN("Visual contact lost (>%.1fs)", (double)lost_timeout_s);
		_visual_contact = false;
	}
}

void Intercept::ComputeApproachGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd)
{
	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.02f; // default 50 Hz work item
	if (_last_approach_time > 0 && now > _last_approach_time) {
		dt = math::constrain((float)(now - _last_approach_time) * 1e-6f, 0.005f, 0.1f);
	}
	_last_approach_time = now;

	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const float dist_to_target = (_target_pos - self_pos).norm();

	// 1. Closing rate calculation (Yol B: mesafe kapanma hızı)
	if (_last_approach_dist < 0.f) {
		_last_approach_dist = dist_to_target;
		_filt_closing_speed = 0.f;
	} else if (dt > 0.005f) {
		const float raw_closing_speed = (_last_approach_dist - dist_to_target) / dt;
		_filt_closing_speed = 0.85f * _filt_closing_speed + 0.15f * raw_closing_speed;
		_last_approach_dist = dist_to_target;
	}

	// Initialize continuous speed from current forward speed if needed
	const float psi = _local_pos.heading;
	const float current_fwd_speed = _local_pos.vx * cosf(psi) + _local_pos.vy * sinf(psi);
	const float max_xy_vel = _param_mpc_xy_vel_max.get();
	if (!_speed_initialized) {
		_fwd_speed_cmd = math::constrain(current_fwd_speed, 20.0f, max_xy_vel);
		_fwd_accel_cmd = 0.0f;
		_speed_initialized = true;
	}

	// Target flight direction and speed
	const float target_speed = _target_vel.norm();
	matrix::Vector3f target_dir(1.f, 0.f, 0.f);

	if (target_speed > 1.f) {
		target_dir = _target_vel / target_speed;
	}

	const float min_closing = _param_int_min_closing.get(); // default 5.0 m/s

	// 2. Direct acceleration control based on closing distance and closing rate
	if (dist_to_target > 100.0f) {
		// Far away (>100m): full throttle pursuit (+5.0 m/s^2 towards top speed)
		_fwd_accel_cmd = 5.0f;
	} else {
		// Inside 100m approaching visual range (50m):
		// Desired closing speed tapers from ~16 m/s at 100m down to min_closing (8 m/s) at 50m
		const float desired_closing = min_closing + (dist_to_target - 50.0f) * 0.16f;
		const float closing_err = _filt_closing_speed - desired_closing;

		if (closing_err > 0.0f) {
			// Closing faster than desired: smooth braking
			_fwd_accel_cmd = math::constrain(-1.5f - 0.3f * closing_err, -5.0f, -0.8f);
		} else if (closing_err < -2.0f) {
			// Closing too slowly: gentle positive acceleration
			_fwd_accel_cmd = 2.0f;
		} else {
			_fwd_accel_cmd = 0.0f;
		}
	}

	// Integrate continuous speed from acceleration command
	_fwd_speed_cmd += _fwd_accel_cmd * dt;

	// Clamp forward speed:
	// Minimum speed GUARANTEES closing speed never drops below min_closing (default 8 m/s)!
	const float min_approach_speed = _target_pos_valid ? math::max(target_speed + min_closing, 15.0f) : 15.0f;
	_fwd_speed_cmd = math::constrain(_fwd_speed_cmd, min_approach_speed, max_xy_vel);

	// 3. Aim point placed 30m directly behind target along its flight line
	const float dist_behind = 30.f;
	const matrix::Vector3f aim_point = _target_pos - target_dir * dist_behind;

	const matrix::Vector3f to_aim = aim_point - self_pos;
	const float dist_to_aim = to_aim.norm();
	const float time_to_aim = dist_to_aim / math::max(_fwd_speed_cmd, 10.0f);

	const matrix::Vector3f intercept_point = aim_point + _target_vel * time_to_aim;
	const matrix::Vector3f to_intercept = intercept_point - self_pos;

	// 4. Horizontal velocity setpoint in inertial NED
	const matrix::Vector2f to_intercept_xy(to_intercept(0), to_intercept(1));
	const float dist_xy = to_intercept_xy.norm();
	float desired_vx{0.f};
	float desired_vy{0.f};

	if (dist_xy > 0.5f) {
		const matrix::Vector2f vel_xy = (to_intercept_xy / dist_xy) * _fwd_speed_cmd;
		desired_vx = vel_xy(0);
		desired_vy = vel_xy(1);
	} else {
		desired_vx = target_dir(0) * _fwd_speed_cmd;
		desired_vy = target_dir(1) * _fwd_speed_cmd;
	}

	// Horizontal acceleration slew-rate limit matching MPC_ACC_HOR
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_acc_hor = _param_mpc_acc_hor.get();
		const float max_dvh = max_acc_hor * dt;
		vel_cmd(0) = math::constrain(desired_vx, _last_vel_cmd(0) - max_dvh, _last_vel_cmd(0) + max_dvh);
		vel_cmd(1) = math::constrain(desired_vy, _last_vel_cmd(1) - max_dvh, _last_vel_cmd(1) + max_dvh);
	} else {
		vel_cmd(0) = desired_vx;
		vel_cmd(1) = desired_vy;
	}

	// 5. Altitude controller in Approach mode
	// delta_z in NED: negative means intercept point is higher (climb), positive is lower (descend)
	const float delta_z = intercept_point(2) - self_pos(2);
	const float target_vz = _target_vel(2);
	const float K_z_p = _param_int_kp_z_app.get();
	const float vz_desired = math::constrain(target_vz + K_z_p * delta_z, -7.5f, 3.5f);

	// Slew-rate limit vertical velocity (max 5.0 m/s^2 acceleration) to rapidly match altitude
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dvz = 5.0f * dt;
		vel_cmd(2) = math::constrain(vz_desired, _last_vel_cmd(2) - max_dvz, _last_vel_cmd(2) + max_dvz);
	} else {
		vel_cmd(2) = vz_desired;
	}

	// 6. Yaw setpoint: point drone nose horizontally towards target position with slew-rate limiting
	const float dx = _target_pos(0) - _local_pos.x;
	const float dy = _target_pos(1) - _local_pos.y;
	const float desired_yaw = atan2f(dy, dx);

	if (_last_cmd_valid && dt > 0.005f) {
		const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
		const float max_dyaw = max_yaw_rate * dt;
		const float yaw_err = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
		yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(yaw_err, -max_dyaw, max_dyaw));
	} else {
		yaw_cmd = desired_yaw;
	}
}

void Intercept::ComputeClosePursuitGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd)
{
	const hrt_abstime now = hrt_absolute_time();

	const float cx = _target_detection.bbox[0]; // [0.0, 1.0], center is 0.50

	// Compute time step for optical derivative estimation
	float dt = 0.033f; // default 30 Hz
	if (_last_visual_time > 0 && now > _last_visual_time) {
		dt = math::constrain((float)(now - _last_visual_time) * 1e-6f, 0.005f, 0.15f);
	}
	_last_visual_time = now;

	// 1. 3D Line-of-Sight in Inertial NED frame
	const matrix::Dcmf R_nb(matrix::Quatf(_vehicle_attitude.q));
	const matrix::Vector3f los_body(_target_detection.los_body);
	matrix::Vector3f los_ned = R_nb * los_body;

	if (los_ned.longerThan(FLT_EPSILON)) {
		los_ned.normalize();
	} else {
		los_ned = matrix::Vector3f(1.f, 0.f, 0.f);
	}

	_last_los_ned = los_ned;
	_los_ned_valid = true;

	// Decompose 3D Line-of-Sight into horizontal and vertical components:
	const float los_xy_norm = sqrtf(los_ned(0) * los_ned(0) + los_ned(1) * los_ned(1));
	const float inv_los_xy = (los_xy_norm > 0.01f) ? (1.0f / los_xy_norm) : 1.0f;
	const float los_x_norm = los_ned(0) * inv_los_xy;
	const float los_y_norm = los_ned(1) * inv_los_xy;

	// 3D slant range along optical sightline from camera
	const float current_range = math::max(_target_detection.range_m, 0.5f);

	// True horizontal following distance (projection along the ground plane):
	// d_horiz = range * cos(elevation) = range * los_xy_norm
	const float current_horiz_dist = math::max(current_range * los_xy_norm, 0.1f);

	// Current forward ground speed along heading
	const float psi = _local_pos.heading;
	const float current_fwd_speed = _local_pos.vx * cosf(psi) + _local_pos.vy * sinf(psi);
	const float max_xy_vel = _param_mpc_xy_vel_max.get();

	// Continuously estimate target forward speed purely from visual closing rate + drone speed
	// Target forward speed along line of flight = drone forward speed - closing rate
	// (Eliminates 2 Hz ground telemetry coupling and discrete speed steps)
	const float raw_target_fwd_speed = current_fwd_speed - _filt_visual_closing_speed;
	if (!_speed_initialized || _target_speed_est < 5.0f) {
		_target_speed_est = math::constrain(raw_target_fwd_speed, 18.0f, max_xy_vel);
		_fwd_speed_cmd = math::constrain(current_fwd_speed, 18.0f, max_xy_vel);
		_fwd_accel_cmd = 0.0f;
		_speed_initialized = true;
	} else if (dt > 0.005f) {
		constexpr float SPEED_EST_TAU = 0.40f;
		const float alpha_spd = math::constrain(dt / (SPEED_EST_TAU + dt), 0.02f, 0.30f);
		_target_speed_est += alpha_spd * (math::constrain(raw_target_fwd_speed, 15.0f, max_xy_vel) - _target_speed_est);
	}

	// 2. Horizontal Standoff Velocity Control (using horizontal distance d_horiz)
	// Estimate horizontal closing speed (m/s) from horizontal distance derivative:
	if (_last_visual_dist < 0.0f) {
		_last_visual_dist = current_horiz_dist;
		_filt_visual_closing_speed = 0.0f;
	} else if (dt > 0.005f) {
		const float raw_closing = (_last_visual_dist - current_horiz_dist) / dt;
		constexpr float CLOSING_TAU = 0.15f;
		const float alpha_close = math::constrain(dt / (CLOSING_TAU + dt), 0.05f, 0.50f);
		_filt_visual_closing_speed += alpha_close * (math::constrain(raw_closing, -15.0f, 15.0f) - _filt_visual_closing_speed);
		_last_visual_dist = current_horiz_dist;
	}

	const float desired_dist = math::constrain(_param_int_standoff.get(), 1.0f, 10.0f);
	const float dist_err = current_horiz_dist - desired_dist; // positive = too far behind, negative = too close/underneath

	// Desired closing speed tapers smoothly to 0 at standoff distance
	const float desired_closing = math::constrain(0.40f * dist_err, -3.0f, 5.0f);

	// Closing speed error: difference between actual closing rate and desired closing rate
	const float closing_err = _filt_visual_closing_speed - desired_closing;

	const float kp_opt = _param_int_kp_opt.get();
	const float kd_opt = _param_int_kd_opt.get();
	const float max_acc_hor = _param_mpc_acc_hor.get();

	constexpr float DIST_TOL = 0.25f; // +/- 25 cm corridor around desired_dist
	float delta_v = 0.0f;

	if (fabsf(dist_err) > DIST_TOL) {
		delta_v = -kd_opt * closing_err + math::constrain(kp_opt * 0.15f * dist_err, -3.0f, 3.0f);
	} else {
		// Inside corridor: gently damp any residual closing velocity
		delta_v = -kd_opt * 0.5f * _filt_visual_closing_speed;
	}

	// Clamp commanded velocity delta to prevent abrupt pitch swings
	delta_v = math::constrain(delta_v, -4.0f, 4.0f);

	// Adaptive lateral corner/swerve braking when close (< 5m):
	// If the target shifts laterally away from boresight center, brake forward speed
	// to tighten turn radius and prevent overshooting / cutting past the target.
	float corner_brake = 0.0f;
	if (current_range < 5.0f) {
		const float lat_offset = fabsf(cx - 0.50f);
		constexpr float LAT_BRAKE_DEADBAND = 0.08f; // center corridor [0.42, 0.58]
		if (lat_offset > LAT_BRAKE_DEADBAND) {
			const float lat_drift = lat_offset - LAT_BRAKE_DEADBAND;
			corner_brake = math::constrain(lat_drift * 15.0f, 0.0f, 5.0f);
		}
	}

	// Overhead protection: if drone is underneath the target (los_xy_norm is small / elevation is steep),
	// throttle back forward speed to avoid rushing forward under the target and losing visual lock
	const float overhead_factor = math::constrain(los_xy_norm / 0.70f, 0.25f, 1.0f);

	// Commanded forward speed referenced to smooth optical target speed estimate
	const float desired_fwd_speed = math::constrain((_target_speed_est + delta_v) * overhead_factor - corner_brake, 10.0f, max_xy_vel);

	// Slew-rate limit forward speed command in close pursuit
	// Smooth positive accel (max 2.0 m/s^2) eliminates pitch bobbing,
	// while allowing slightly crisper braking (max 3.5 m/s^2) to prevent corner overshoot.
	constexpr float MAX_CLOSE_ACCEL = 2.0f;
	constexpr float MAX_CLOSE_DECEL = 3.5f;
	const float prev_speed_cmd = _fwd_speed_cmd;
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dv_acc = MAX_CLOSE_ACCEL * dt;
		const float max_dv_dec = MAX_CLOSE_DECEL * dt;
		_fwd_speed_cmd = math::constrain(desired_fwd_speed, _fwd_speed_cmd - max_dv_dec, _fwd_speed_cmd + max_dv_acc);
		_fwd_accel_cmd = (_fwd_speed_cmd - prev_speed_cmd) / dt;
	} else {
		_fwd_speed_cmd = desired_fwd_speed;
		_fwd_accel_cmd = 0.0f;
	}

	// 3. 3D Velocity Command
	// Coordinated Flight: forward velocity flows primarily along Line-of-Sight.
	// To prevent uncoordinated sideslip and roll banking at 32 m/s, lateral strafe
	// is heavily bounded to a gentle nudge (+/- 0.8 m/s max) with deadband [0.45, 0.55].
	float lat_err = 0.0f;
	if (cx < 0.45f) {
		lat_err = cx - 0.45f;
	} else if (cx > 0.55f) {
		lat_err = cx - 0.55f;
	} else {
		lat_err = 0.0f;
	}
	const float kp_lat = 1.0f;
	const float vel_lat_corr = math::constrain(lat_err * kp_lat, -0.8f, 0.8f);

	// Unit horizontal vector perpendicular to LOS in XY plane (pointing right)
	const float right_x = -los_y_norm;
	const float right_y =  los_x_norm;

	float desired_vx = los_x_norm * _fwd_speed_cmd + right_x * vel_lat_corr;
	float desired_vy = los_y_norm * _fwd_speed_cmd + right_y * vel_lat_corr;

	// Slew-rate limit horizontal acceleration matching MPC_ACC_HOR
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dvh = max_acc_hor * dt;
		vel_cmd(0) = math::constrain(desired_vx, _last_vel_cmd(0) - max_dvh, _last_vel_cmd(0) + max_dvh);
		vel_cmd(1) = math::constrain(desired_vy, _last_vel_cmd(1) - max_dvh, _last_vel_cmd(1) + max_dvh);
	} else {
		vel_cmd(0) = desired_vx;
		vel_cmd(1) = desired_vy;
	}

	// Vertical altitude tracking with deadband corridor (+/- 0.30m)
	// Computes physical altitude difference directly from optical line-of-sight elevation and metric distance:
	const float delta_z = current_range * los_ned(2); // In NED: negative = target is higher (climb), positive = lower

	const float z_tol = _param_int_z_tol.get(); // default 0.3m (+/- 30 cm deadband corridor)
	float delta_z_corr = 0.0f;
	if (delta_z < -z_tol) {
		delta_z_corr = delta_z + z_tol; // target higher (NED negative) -> climb
	} else if (delta_z > z_tol) {
		delta_z_corr = delta_z - z_tol; // target lower (NED positive) -> descend
	} else {
		delta_z_corr = 0.0f; // within +/- corridor -> hold level flight!
	}

	const float kp_z = _param_int_kp_z.get(); // default 0.6
	float vz_desired = math::constrain(delta_z_corr * kp_z, -2.5f, 2.0f);
	if (fabsf(delta_z_corr) < FLT_EPSILON) {
		vz_desired = 0.0f;
	}

	// Slew-rate limit vertical acceleration (max 1.2 m/s^2) to prevent pitch oscillations
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dvz = 1.2f * dt;
		vel_cmd(2) = math::constrain(vz_desired, _last_vel_cmd(2) - max_dvz, _last_vel_cmd(2) + max_dvz);
	} else {
		vel_cmd(2) = vz_desired;
	}

	// 4. Optical-Centering Seeker Yaw:
	// Automatically cancels Pitch-Roll kinematic coupling by feeding back horizontal optical offset (cx - 0.50).
	// When drone banks (rolls) during high-speed forward flight, the camera optical axis rotates
	// in azimuth relative to vehicle heading. Adding optical azimuth correction keeps the target
	// pinned dead-center on the HUD crosshair.
	float desired_yaw = _last_cmd_valid ? _last_yaw_cmd : _local_pos.heading;
	if (los_xy_norm > 0.20f && current_horiz_dist > 0.8f) {
		const float los_yaw = atan2f(los_ned(1), los_ned(0));

		// Camera horizontal FOV = 80 deg. Optical azimuth error angle:
		constexpr float HFOV_RAD = math::radians(80.0f);
		constexpr float OPTICAL_YAW_GAIN = 2.0f * tanf(HFOV_RAD / 2.0f); // ~1.678 rad
		const float visual_yaw_offset = (cx - 0.50f) * OPTICAL_YAW_GAIN;

		// Deadband corridor around screen center: +/- 0.025 (corridor [0.475, 0.525])
		constexpr float YAW_DEADBAND = 0.025f;
		if (fabsf(cx - 0.50f) > YAW_DEADBAND) {
			desired_yaw = matrix::wrap_pi(los_yaw + visual_yaw_offset);
		}
	}

	if (_last_cmd_valid && dt > 0.005f) {
		// Soft deceleration (tapering): smoothly reduce maximum yaw rate as target approaches center
		// Eliminates overshoot completely by docking gently at the deadband threshold
		const float center_dist = fabsf(cx - 0.50f);
		const float rate_factor = math::constrain((center_dist - 0.02f) / 0.10f, 0.25f, 1.0f);
		constexpr float MAX_CLOSE_YAW_RATE_RAD = math::radians(18.0f);
		const float max_dyaw = (MAX_CLOSE_YAW_RATE_RAD * rate_factor) * dt;

		const float yaw_err = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
		yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(yaw_err, -max_dyaw, max_dyaw));
	} else {
		yaw_cmd = desired_yaw;
	}
}

void Intercept::Run()
{
	if (should_exit()) {
		ScheduleClear();

		if (_sent_mode_registration) {
			UnregisterFlightMode();
		}

		exit_and_cleanup();
		return;
	}

	// 1. Parameter update
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// 2. Register flight mode with Commander
	if (!_sent_mode_registration) {
		RegisterFlightMode();
		_sent_mode_registration = true;
		return;
	}

	// 3. Check for registration confirmation
	if (_mode_id == -1 || _arming_check_id == -1) {
		CheckModeRegistration();
		return;
	}

	// 4. Respond to arming checks from Commander
	if (_arming_check_request_sub.updated()) {
		arming_check_request_s req;
		_arming_check_request_sub.copy(&req);
		ReplyToArmingCheck(req.request_id);
	}

	// 5. Update local position, attitude, target tracking, and visual detection
	_vehicle_local_position_sub.update(&_local_pos);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	UpdateTarget();
	UpdateVisualDetection();

	// 6. Check if Intercept is currently the active navigation mode
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_is_active = (vehicle_status.nav_state == _mode_id);
	}

	// 7. Mode execution
	if (_is_active) {
		if (!_was_active) {
			PX4_INFO("Intercept mode activated: pursuing target");
			_was_active = true;
			_hold_valid = false;
			_guidance_state = GuidanceState::APPROACH;
			_last_cmd_valid = false;
			_last_visual_time = 0;
			_speed_initialized = false;
			_los_ned_valid = false;
			_fwd_speed_cmd = _param_mpc_xy_vel_max.get();
			_fwd_accel_cmd = 0.f;
			_last_approach_dist = -1.f;
			_filt_closing_speed = 0.f;
			_last_approach_time = 0;
			_last_visual_dist = -1.f;
			_filt_visual_closing_speed = 0.f;
			_target_speed_est = 25.f;
		}

		// State transitions: APPROACH <-> CLOSE_PURSUIT
		if (_guidance_state == GuidanceState::APPROACH) {
			// Transition to CLOSE_PURSUIT as soon as target is acquired visually
			if (_visual_contact && _target_detection.detected && (_target_detection.bbox[2] > 0.005f)) {
				_guidance_state = GuidanceState::CLOSE_PURSUIT;
				_last_visual_time = 0;
				_los_ned_valid = false;
				_last_visual_dist = -1.f;
				_filt_visual_closing_speed = 0.f;
				_target_speed_est = _fwd_speed_cmd;

				if (!_last_cmd_valid) {
					_last_yaw_cmd = _local_pos.heading;
					_last_vel_cmd(0) = _local_pos.vx;
					_last_vel_cmd(1) = _local_pos.vy;
					_last_vel_cmd(2) = _local_pos.vz;
					_last_cmd_valid = true;
				}

				PX4_INFO("Visual contact acquired! Handover to CLOSE_PURSUIT, speed=%.1f m/s, range=%.1f m",
					 (double)_fwd_speed_cmd, (double)_target_detection.range_m);
			}

		} else if (_guidance_state == GuidanceState::CLOSE_PURSUIT) {
			if (!_visual_contact) {
				_guidance_state = GuidanceState::APPROACH;
				_last_cmd_valid = false;
				_last_visual_time = 0;
				_speed_initialized = false;
				_los_ned_valid = false;
				_last_approach_dist = -1.f;
				_last_approach_time = 0;
				_last_visual_dist = -1.f;
				_filt_visual_closing_speed = 0.f;
				_target_speed_est = 25.f;
				PX4_WARN("Visual contact lost (>%.1fs), reverting to APPROACH", (double)_param_int_lost_timeout.get());
			}
		}

		if (_guidance_state == GuidanceState::CLOSE_PURSUIT) {
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};

			if (_target_detection.detected && (_target_detection.bbox[2] > 0.005f)) {
				ComputeClosePursuitGuidance(vel_cmd, yaw_cmd);
				_last_vel_cmd = vel_cmd;
				_last_yaw_cmd = yaw_cmd;
				_last_cmd_valid = true;

			} else if (_target_pos_valid) {
				// Target lost from camera frame:
				// Immediately turn drone head (yaw) towards target's current position to re-acquire!
				const float dx = _target_pos(0) - _local_pos.x;
				const float dy = _target_pos(1) - _local_pos.y;
				const float dist_xy = sqrtf(dx * dx + dy * dy);
				const float desired_yaw = atan2f(dy, dx);
				const float dt = 0.02f; // ~50Hz work item step

				if (_last_cmd_valid) {
					const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
					const float max_dyaw = max_yaw_rate * dt;
					const float yaw_err = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
					yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(yaw_err, -max_dyaw, max_dyaw));
				} else {
					yaw_cmd = desired_yaw;
				}

				// Fly towards target position with continuous speed
				const float pursuit_speed = math::constrain(_fwd_speed_cmd, 12.0f, _param_mpc_xy_vel_max.get());
				if (dist_xy > 0.5f) {
					vel_cmd(0) = (dx / dist_xy) * pursuit_speed;
					vel_cmd(1) = (dy / dist_xy) * pursuit_speed;
				} else {
					vel_cmd(0) = _last_vel_cmd(0);
					vel_cmd(1) = _last_vel_cmd(1);
				}

				// Altitude: track target altitude
				const float dz = _target_pos(2) - _local_pos.z;
				vel_cmd(2) = math::constrain(dz * _param_int_kp_z.get(), -2.5f, 2.0f);

				_last_vel_cmd = vel_cmd;
				_last_yaw_cmd = yaw_cmd;
				_last_cmd_valid = true;

			} else if (_last_cmd_valid) {
				// Coast on last known command if target position is unavailable
				vel_cmd = _last_vel_cmd;
				yaw_cmd = _last_yaw_cmd;
			}

			trajectory_setpoint_s sp{};
			sp.timestamp = hrt_absolute_time();
			sp.position[0] = NAN;
			sp.position[1] = NAN;
			sp.position[2] = NAN;
			sp.velocity[0] = vel_cmd(0);
			sp.velocity[1] = vel_cmd(1);
			sp.velocity[2] = vel_cmd(2);
			sp.acceleration[0] = NAN;
			sp.acceleration[1] = NAN;
			sp.acceleration[2] = NAN;
			sp.yaw = yaw_cmd;
			sp.yawspeed = NAN;
			_trajectory_setpoint_pub.publish(sp);

		} else if (_target_pos_valid) {
			// Approach guidance: fly towards aim point behind target
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};
			ComputeApproachGuidance(vel_cmd, yaw_cmd);
			_last_vel_cmd = vel_cmd;
			_last_yaw_cmd = yaw_cmd;
			_last_cmd_valid = true;

			trajectory_setpoint_s sp{};
			sp.timestamp = hrt_absolute_time();
			sp.position[0] = NAN;
			sp.position[1] = NAN;
			sp.position[2] = NAN;
			sp.velocity[0] = vel_cmd(0);
			sp.velocity[1] = vel_cmd(1);
			sp.velocity[2] = vel_cmd(2);
			sp.acceleration[0] = NAN;
			sp.acceleration[1] = NAN;
			sp.acceleration[2] = NAN;
			sp.yaw = yaw_cmd;
			sp.yawspeed = NAN;
			_trajectory_setpoint_pub.publish(sp);

		} else {
			// Fallback: if target GPS not yet valid, hold current position
			if (!_hold_valid && _local_pos.xy_valid && _local_pos.z_valid) {
				_hold_position(0) = _local_pos.x;
				_hold_position(1) = _local_pos.y;
				_hold_position(2) = _local_pos.z;
				_hold_yaw = _local_pos.heading;
				_hold_valid = true;
			}

			if (_hold_valid) {
				trajectory_setpoint_s sp{};
				sp.timestamp = hrt_absolute_time();
				sp.position[0] = _hold_position(0);
				sp.position[1] = _hold_position(1);
				sp.position[2] = _hold_position(2);
				sp.velocity[0] = 0.f;
				sp.velocity[1] = 0.f;
				sp.velocity[2] = 0.f;
				sp.acceleration[0] = NAN;
				sp.acceleration[1] = NAN;
				sp.acceleration[2] = NAN;
				sp.yaw = _hold_yaw;
				sp.yawspeed = 0.f;
				_trajectory_setpoint_pub.publish(sp);
			}
		}

	} else {
		if (_was_active) {
			PX4_INFO("Intercept mode deactivated");
			_was_active = false;
			_hold_valid = false;
			_guidance_state = GuidanceState::APPROACH;
			_last_cmd_valid = false;
			_last_visual_time = 0;
			_speed_initialized = false;
			_los_ned_valid = false;
			_fwd_speed_cmd = _param_mpc_xy_vel_max.get();
			_fwd_accel_cmd = 0.f;
			_last_approach_dist = -1.f;
			_filt_closing_speed = 0.f;
			_last_approach_time = 0;
			_last_visual_dist = -1.f;
			_filt_visual_closing_speed = 0.f;
			_target_speed_est = 25.f;
		}
	}
}

int Intercept::print_status()
{
	PX4_INFO("Running: %s", is_running() ? "yes" : "no");
	PX4_INFO("Registered: %s (mode_id=%d, arming_check_id=%d)",
		 (_mode_id != -1) ? "yes" : "no", _mode_id, _arming_check_id);
	PX4_INFO("Active: %s", _is_active ? "yes" : "no");

	if (_hold_valid) {
		PX4_INFO("Holding position: [%.2f, %.2f, %.2f] m, yaw: %.1f deg",
			 (double)_hold_position(0), (double)_hold_position(1), (double)_hold_position(2),
			 (double)math::degrees(_hold_yaw));
	}

	if (_target_pos_valid) {
		const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
		const matrix::Vector3f rel_pos = _target_pos - self_pos;
		const float dist = rel_pos.norm();
		const float age_s = (float)(hrt_elapsed_time(&_last_target_update)) * 1e-6f;

		PX4_INFO("Target GPS: [%.1f, %.1f, %.1f] m, vel: [%.1f, %.1f, %.1f] m/s, dist: %.1f m (age: %.2f s)",
			 (double)_target_pos(0), (double)_target_pos(1), (double)_target_pos(2),
			 (double)_target_vel(0), (double)_target_vel(1), (double)_target_vel(2),
			 (double)dist, (double)age_s);

	} else {
		PX4_INFO("Target GPS: no valid fix");
	}

	PX4_INFO("Guidance state: %s", (_guidance_state == GuidanceState::CLOSE_PURSUIT) ? "CLOSE_PURSUIT (IBVS Visual Servoing)" : "APPROACH (Midcourse GPS)");
	PX4_INFO("Speed setpoint: %.1f m/s, accel_cmd: %.2f m/s^2, closing rate: %.1f m/s",
		 (double)_fwd_speed_cmd, (double)_fwd_accel_cmd, (double)_filt_closing_speed);

	if (_visual_contact) {
		PX4_INFO("Visual contact: YES (range: %.1f m, bbox: [%.2f, %.2f, %.2f, %.2f])",
			 (double)_visual_range, (double)_target_detection.bbox[0], (double)_target_detection.bbox[1],
			 (double)_target_detection.bbox[2], (double)_target_detection.bbox[3]);
		PX4_INFO("Forward speed command: %.1f m/s, LOS NED: [%.2f, %.2f, %.2f]",
			 (double)_fwd_speed_cmd, (double)_last_los_ned(0), (double)_last_los_ned(1), (double)_last_los_ned(2));
	} else {
		PX4_INFO("Visual contact: NO");
	}

	return 0;
}

int Intercept::task_spawn(int argc, char *argv[])
{
	Intercept *instance = new Intercept();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		PX4_ERR("init failed");

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Intercept::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int Intercept::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Autonomous target intercept and close pursuit flight mode.
Two-stage guidance:
  1. APPROACH: Midcourse lead-pursuit towards target GPS position.
  2. CLOSE_PURSUIT: Pure IBVS visual servoing & 3D LOS station-keeping.
Registers external flight mode 'Intercept' with Commander.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("intercept", "mode");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int intercept_main(int argc, char *argv[])
{
	return Intercept::main(argc, argv);
}
