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
	ScheduleOnInterval(10_ms); // 100 Hz guidance execution loop
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

void Intercept::UpdateTarget(float dt)
{
	// 1. Maintain local map projection reference from local position
	if (PX4_ISFINITE(_local_pos.ref_lat) && PX4_ISFINITE(_local_pos.ref_lon)
	    && (fabsf((float)_local_pos.ref_lat) > 0.0001f || fabsf((float)_local_pos.ref_lon) > 0.0001f)) {
		if (!_map_ref.isInitialized()
		    || _map_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp) {
			_map_ref.initReference(_local_pos.ref_lat, _local_pos.ref_lon, _local_pos.ref_timestamp);
		}
	}

	// 2. Read new follow_target messages from ground truth / ground relay
	if (_follow_target_sub.updated()) {
		if (_follow_target_sub.copy(&_last_follow_target)) {
			_has_follow_target = true;
		}
	}

	bool new_sample_processed = false;
	const hrt_abstime now = hrt_absolute_time();

	if (_has_follow_target && _map_ref.isInitialized()
	    && PX4_ISFINITE(_last_follow_target.lat) && PX4_ISFINITE(_last_follow_target.lon)) {

		if (_last_follow_target.timestamp != _last_target_rx_time) {
			_last_target_rx_time = _last_follow_target.timestamp;

			float x{0.f};
			float y{0.f};
			_map_ref.project(_last_follow_target.lat, _last_follow_target.lon, x, y);

			const matrix::Vector3f new_target_vel(_last_follow_target.vx, _last_follow_target.vy, _last_follow_target.vz);

			if (_last_target_update > 0 && now > _last_target_update) {
				const float dt_meas = (float)(now - _last_target_update) * 1e-6f;

				if (dt_meas > 0.005f && dt_meas < 1.0f) {
					const matrix::Vector3f raw_accel = (new_target_vel - _target_vel) / dt_meas;
					// Singer maneuver / low-pass filter on target acceleration
					const float alpha = math::constrain(dt_meas / 0.15f, 0.10f, 0.80f);
					_target_accel = _target_accel * (1.0f - alpha) + raw_accel * alpha;
				}
			}

			_target_pos(0) = x;
			_target_pos(1) = y;
			_target_pos(2) = -(_last_follow_target.alt - _local_pos.ref_alt);
			_target_vel = new_target_vel;

			_target_pos_valid = true;
			_target_source = TargetSource::GPS_FOLLOW_TARGET;
			_last_target_update = now;
			new_sample_processed = true;
		}
	}

	// 2. Optical Seeker direct lock: active whenever camera detects target within visual range
	if (_visual_contact && _target_detection.range_valid
	    && PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && PX4_ISFINITE(_local_pos.z)) {
		const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
		const matrix::Vector3f los_b(_target_detection.los_body[0], _target_detection.los_body[1], _target_detection.los_body[2]);
		const matrix::Quatf q_att(_vehicle_attitude.q);
		const matrix::Vector3f los_ned = q_att.rotateVector(los_b);
		const matrix::Vector3f vis_target_pos = self_pos + los_ned * _target_detection.range_m;

		_target_pos = vis_target_pos;
		_target_pos_valid = true;
		_target_source = TargetSource::VISION;
		_last_target_update = now;

	} else if (!new_sample_processed) {
		const bool follow_target_fresh = _has_follow_target && (hrt_elapsed_time(&_last_target_rx_time) < 1_s);

		if (follow_target_fresh && _target_pos_valid && dt > 0.001f && dt < 0.05f) {
			// 100 Hz kinematic dead-reckoning extrapolation between target telemetry frames
			_target_pos += _target_vel * dt + _target_accel * (0.5f * dt * dt);
			_target_vel += _target_accel * dt;
			_target_source = TargetSource::DEAD_RECKONING;
		}
	}

	// 3. Invalidate target if no update for more than 3 seconds
	if (_target_pos_valid && (hrt_elapsed_time(&_last_target_update) > 3_s)) {
		_target_pos_valid = false;
		_target_source = TargetSource::NONE;
		_pos_integral.zero();
		_firing_counter = 0;
	}
}

void Intercept::UpdateVisualDetection()
{
	target_detection_s td;

	if (_target_detection_sub.update(&td)) {
		if (td.detected) {
			_visual_contact = true;
			_visual_range = td.range_m;
			_target_detection = td;
			_last_visual_contact = hrt_absolute_time();
		}
	}

	if (_visual_contact && (hrt_elapsed_time(&_last_visual_contact) > 2_s)) {
		_visual_contact = false;
	}
}

void Intercept::ComputeMidcourseGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd, float dt)
{
	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const float dist_to_target = (_target_pos - self_pos).norm();

	// 1. Closing rate calculation
	if (_last_approach_dist < 0.0f) {
		_last_approach_dist = dist_to_target;
		_filt_closing_speed = 0.0f;
	} else if (dt > 0.002f) {
		const float raw_closing = (_last_approach_dist - dist_to_target) / dt;
		_filt_closing_speed = 0.85f * _filt_closing_speed + 0.15f * raw_closing;
		_last_approach_dist = dist_to_target;
	}

	// 2. Initialize continuous forward speed
	const float psi = _local_pos.heading;
	const float current_fwd_speed = _local_pos.vx * cosf(psi) + _local_pos.vy * sinf(psi);
	if (!_speed_initialized) {
		_fwd_speed_cmd = math::constrain(current_fwd_speed, 18.0f, 40.0f);
		_fwd_accel_cmd = 0.0f;
		_speed_initialized = true;
	}

	const float target_speed = _target_vel.norm();
	matrix::Vector3f target_dir(1.0f, 0.0f, 0.0f);
	if (target_speed > 1.0f) {
		target_dir = _target_vel / target_speed;
	}

	const float d_offset = _param_int_basket_d.get();

	// 3. Smooth approach speed profile:
	// When far (> 50m): sprint towards target_speed + 12 m/s
	// When approaching (50m down to 18m): smoothly decelerate towards target_speed + 2.5 m/s
	const float dist_to_basket = math::max(0.0f, dist_to_target - d_offset);
	const float desired_closing = math::constrain(sqrtf(2.0f * 1.5f * dist_to_basket), 2.0f, 14.0f);
	_fwd_speed_cmd = math::constrain(target_speed + desired_closing, 15.0f, 44.0f);

	// 4. Virtual aim point at the basket with 2nd-order turn acceleration compensation
	const matrix::Vector3f aim_point = _target_pos - target_dir * d_offset;
	const matrix::Vector3f to_aim = aim_point - self_pos;
	const float dist_to_aim = to_aim.norm();
	const float time_to_aim = dist_to_aim / math::max(_fwd_speed_cmd, 10.0f);

	const float t_acc = math::min(time_to_aim, 2.0f);
	const matrix::Vector3f intercept_point = aim_point + _target_vel * time_to_aim + _target_accel * (0.5f * t_acc * t_acc);
	const matrix::Vector3f to_intercept = intercept_point - self_pos;

	// 5. Horizontal velocity setpoint in inertial NED with dynamic agile turn acceleration limiting
	const matrix::Vector2f to_intercept_xy(to_intercept(0), to_intercept(1));
	const float dist_xy = to_intercept_xy.norm();
	float desired_vx{0.0f};
	float desired_vy{0.0f};

	if (dist_xy > 0.5f) {
		const matrix::Vector2f vel_xy = (to_intercept_xy / dist_xy) * _fwd_speed_cmd;
		desired_vx = vel_xy(0);
		desired_vy = vel_xy(1);
	} else {
		desired_vx = target_dir(0) * _fwd_speed_cmd;
		desired_vy = target_dir(1) * _fwd_speed_cmd;
	}

	const float max_acc_hor = math::max(_param_mpc_acc_hor.get(), 20.0f);
	const float max_dvh = max_acc_hor * dt;
	if (_last_cmd_valid && dt > 0.005f) {
		vel_cmd(0) = math::constrain(desired_vx, _last_vel_cmd(0) - max_dvh, _last_vel_cmd(0) + max_dvh);
		vel_cmd(1) = math::constrain(desired_vy, _last_vel_cmd(1) - max_dvh, _last_vel_cmd(1) + max_dvh);
	} else {
		vel_cmd(0) = desired_vx;
		vel_cmd(1) = desired_vy;
	}

	// 6. Altitude tracking in Approach mode (Extended to +/-7.0 m/s to match rapid dives and zoom climbs)
	const float delta_z = intercept_point(2) - self_pos(2);
	const float target_vz = _target_vel(2);
	const float K_z_p = _param_int_kp_z_app.get();
	const float vz_desired = math::constrain(target_vz + K_z_p * delta_z, -7.0f, 7.0f);

	const float max_dvz = 4.0f * dt;
	if (_last_cmd_valid && dt > 0.005f) {
		vel_cmd(2) = math::constrain(vz_desired, _last_vel_cmd(2) - max_dvz, _last_vel_cmd(2) + max_dvz);
	} else {
		vel_cmd(2) = vz_desired;
	}

	// 7. Yaw setpoint towards target
	const float dx = _target_pos(0) - self_pos(0);
	const float dy = _target_pos(1) - self_pos(1);
	const float desired_yaw = atan2f(dy, dx);

	if (_last_cmd_valid && dt > 0.005f) {
		const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
		const float dyaw = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
		yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(dyaw, -max_yaw_rate * dt, max_yaw_rate * dt));
	} else {
		yaw_cmd = desired_yaw;
	}
}

void Intercept::ComputeBasketGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd, float dt)
{
	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const matrix::Vector3f self_vel(_local_pos.vx, _local_pos.vy, _local_pos.vz);
	const float target_speed = _target_vel.norm();
	matrix::Vector3f target_dir(1.0f, 0.0f, 0.0f);
	if (target_speed > 1.0f) {
		target_dir = _target_vel / target_speed;
	}

	const float d_offset = _param_int_basket_d.get(); // 3.5m safe standoff distance
	const matrix::Vector3f p_v = _target_pos - (target_dir * d_offset);
	const matrix::Vector3f r_v = p_v - self_pos;
	const matrix::Vector3f v_rel = self_vel - _target_vel; // relative velocity (drone - target)

	// Decompose position error into along-track (parallel to target_dir) and cross-track
	const float r_along = r_v.dot(target_dir); // > 0 means drone is behind basket
	const matrix::Vector3f r_cross = r_v - target_dir * r_along;

	// Integrate cross-track position error with anti-windup (steady-state wind drift)
	_pos_integral(0) = math::constrain(_pos_integral(0) + r_cross(0) * dt, -1.5f, 1.5f);
	_pos_integral(1) = math::constrain(_pos_integral(1) + r_cross(1) * dt, -1.5f, 1.5f);
	_pos_integral(2) = math::constrain(_pos_integral(2) + r_v(2) * dt, -1.5f, 1.5f);

	// Along-track: smooth, critically damped approach to basket with sqrt deceleration curve
	float v_along_corr = 0.0f;
	const float v_along_rel = v_rel.dot(target_dir); // current closing rate

	if (r_along > 0.0f) {
		// Behind basket: smoothly approach with strong damping on closing rate to prevent overshoot
		const float v_close_des = math::min(4.5f, sqrtf(2.0f * 1.5f * r_along));
		const float v_err = v_close_des - v_along_rel;
		v_along_corr = math::constrain(1.2f * v_err, -3.0f, 5.0f);
	} else {
		// In front of basket: brake immediately to restore safe standoff distance!
		v_along_corr = math::constrain(1.5f * r_along - 0.8f * v_along_rel, -6.0f, 0.0f);
	}

	// Cross-track and vertical PD tracking with turn acceleration feedforward
	const float tau_lead = 0.40f;
	const matrix::Vector3f v_cross_corr(
		1.5f * r_cross(0) - 0.7f * (v_rel(0) - target_dir(0) * v_along_rel) + 0.15f * _pos_integral(0) + _target_accel(0) * tau_lead,
		1.5f * r_cross(1) - 0.7f * (v_rel(1) - target_dir(1) * v_along_rel) + 0.15f * _pos_integral(1) + _target_accel(1) * tau_lead,
		1.4f * r_v(2) - 0.6f * v_rel(2) + 0.15f * _pos_integral(2) + _target_accel(2) * 0.30f
	);

	const matrix::Vector3f v_corr = target_dir * v_along_corr + v_cross_corr;
	const matrix::Vector3f v_rdv = _target_vel + v_corr;

	const float max_acc_hor = math::max(_param_mpc_acc_hor.get(), 20.0f);
	const float max_dvh = max_acc_hor * dt;
	if (_last_cmd_valid && dt > 0.005f) {
		vel_cmd(0) = math::constrain(v_rdv(0), _last_vel_cmd(0) - max_dvh, _last_vel_cmd(0) + max_dvh);
		vel_cmd(1) = math::constrain(v_rdv(1), _last_vel_cmd(1) - max_dvh, _last_vel_cmd(1) + max_dvh);
		vel_cmd(2) = math::constrain(v_rdv(2), _last_vel_cmd(2) - 3.5f * dt, _last_vel_cmd(2) + 3.5f * dt);
	} else {
		vel_cmd = v_rdv;
	}

	// Yaw aligns with target direction
	const float target_yaw = atan2f(target_dir(1), target_dir(0));
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
		const float dyaw = matrix::wrap_pi(target_yaw - _last_yaw_cmd);
		yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(dyaw, -max_yaw_rate * dt, max_yaw_rate * dt));
	} else {
		yaw_cmd = target_yaw;
	}
}

void Intercept::ComputeGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd, float dt)
{
	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const matrix::Vector3f self_vel(_local_pos.vx, _local_pos.vy, _local_pos.vz);

	const float target_speed = _target_vel.norm();
	matrix::Vector3f target_dir(1.0f, 0.0f, 0.0f);
	if (target_speed > 1.0f) {
		target_dir = _target_vel / target_speed;
	}

	const float d_offset = _param_int_basket_d.get(); // 3.5m safe standoff distance
	const matrix::Vector3f p_v = _target_pos - (target_dir * d_offset);
	_p_v = p_v;
	_v_v = _target_vel;

	const matrix::Vector3f r_v = p_v - self_pos;
	const float R_v = r_v.norm();
	const matrix::Vector3f v_rel = _target_vel - self_vel;

	_last_R_v = R_v;
	_last_r_v = r_v;
	_last_V_c = -(r_v.dot(v_rel)) / math::max(R_v, 0.01f);

	if (_net_fired) {
		// Target successfully captured: do NOT ram into falling target!
		// Maintain level altitude and smoothly back off to observe capture safely
		vel_cmd = target_dir * math::max(target_speed - 5.0f, 10.0f);
		vel_cmd(2) = 0.0f;
		yaw_cmd = atan2f(target_dir(1), target_dir(0));
		return;
	}

	matrix::Vector3f vel_midcourse;
	float yaw_midcourse{_local_pos.heading};
	ComputeMidcourseGuidance(vel_midcourse, yaw_midcourse, dt);

	matrix::Vector3f vel_basket;
	float yaw_basket{_local_pos.heading};
	ComputeBasketGuidance(vel_basket, yaw_basket, dt);

	// Smooth blending between Midcourse Lead-Pursuit (Rv > 18m) and Basket Seating (Rv <= 6m)
	const float w = math::constrain((R_v - 6.0f) / 12.0f, 0.0f, 1.0f);
	vel_cmd = (vel_midcourse * w) + (vel_basket * (1.0f - w));

	const float dyaw = matrix::wrap_pi(yaw_basket - yaw_midcourse);
	yaw_cmd = matrix::wrap_pi(yaw_midcourse + dyaw * (1.0f - w));

	if (R_v > 18.0f) {
		_guidance_phase = GuidancePhase::APPROACH;
	} else if (R_v > 6.0f) {
		_guidance_phase = GuidancePhase::BLENDING;
	} else {
		_guidance_phase = GuidancePhase::RENDEZVOUS;
	}

	CheckFiringEnvelope(r_v, self_vel, target_dir);
}

void Intercept::CheckFiringEnvelope(const matrix::Vector3f &r_v, const matrix::Vector3f &self_vel, const matrix::Vector3f &u_t)
{
	const matrix::Vector3f self_fwd(cosf(_local_pos.heading), sinf(_local_pos.heading), 0.0f);
	const matrix::Vector3f u_t_horiz = matrix::Vector3f(u_t(0), u_t(1), 0.0f).normalized();

	const float rv_dist = r_v.norm();
	const float vel_diff = (self_vel - _target_vel).norm();
	const float align_dot = self_fwd.dot(u_t_horiz);

	// Safe Tactical Two-Tier Firing Envelope with d_offset = 3.5m:
	// Tier 1 (Basket Seating): Rv <= 2.2m (meaning 1.3m to 5.7m from target), vel match <= 3.0 m/s, forward alignment >= 0.85
	const bool matched_seating = (rv_dist <= 2.2f) && (vel_diff <= 3.0f) && (align_dot >= 0.85f);

	// Tier 2 (Flyby Pass): Rv <= 1.2m, forward alignment >= 0.80
	const bool close_flyby = (rv_dist <= 1.2f) && (align_dot >= 0.80f);

	if (matched_seating || close_flyby) {
		_firing_counter++;

		if (_firing_counter >= 4 && !_net_fired) {
			_net_fired = true;
			_net_fire_time = hrt_absolute_time();
			PX4_INFO(">>> NET GUN FIRED! TARGET CAPTURED! <<<");

			// Trigger net launcher servo signal
			actuator_servos_s servos{};
			servos.timestamp = hrt_absolute_time();
			servos.control[0] = 1.0f;
			_actuator_servos_pub.publish(servos);
		}

	} else if (!_net_fired) {
		if (_firing_counter > 0) {
			_firing_counter--;
		}
	}
}

void Intercept::PublishGimbalTracking(const matrix::Vector3f &self_pos)
{
	const matrix::Vector3f r_target = _target_pos - self_pos;
	const float r_xy = sqrtf(r_target(0) * r_target(0) + r_target(1) * r_target(1));

	// World NED reference frame: Roll=0.0f (horizon-stabilized), Pitch and Yaw
	const float roll_ned = 0.0f;
	const float pitch_ned = atan2f(-r_target(2), math::max(r_xy, 0.01f));
	const float yaw_ned = atan2f(r_target(1), r_target(0));

	const matrix::Quatf q_gimbal(matrix::Eulerf(roll_ned, pitch_ned, yaw_ned));

	gimbal_device_set_attitude_s gimbal_cmd{};
	gimbal_cmd.timestamp = hrt_absolute_time();
	gimbal_cmd.target_system = 0;
	gimbal_cmd.target_component = 0;
	gimbal_cmd.flags = gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_ROLL_LOCK |
			   gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_PITCH_LOCK |
			   gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_LOCK;

	q_gimbal.copyTo(gimbal_cmd.q);
	gimbal_cmd.angular_velocity_x = NAN;
	gimbal_cmd.angular_velocity_y = NAN;
	gimbal_cmd.angular_velocity_z = NAN;

	_gimbal_device_set_attitude_pub.publish(gimbal_cmd);
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

	// 4. Respond to all arming checks from Commander
	arming_check_request_s req;
	while (_arming_check_request_sub.update(&req)) {
		ReplyToArmingCheck(req.request_id);
	}

	// 5. Update local position, attitude, and target kinematics
	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.010f; // 100 Hz nominal step

	if (_last_loop_time > 0 && now > _last_loop_time) {
		dt = math::constrain((float)(now - _last_loop_time) * 1e-6f, 0.002f, 0.05f);
	}
	_last_loop_time = now;

	_vehicle_local_position_sub.update(&_local_pos);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	UpdateTarget(dt);
	UpdateVisualDetection();

	// Update 3-axis Storm32 gimbal tracking: continuously keep camera locked on target in world NED frame
	// Active at all times (including on the ground before takeoff) as long as target GPS telemetry is available
	if (_target_pos_valid) {
		const matrix::Vector3f self_pos(
			PX4_ISFINITE(_local_pos.x) ? _local_pos.x : 0.0f,
			PX4_ISFINITE(_local_pos.y) ? _local_pos.y : 0.0f,
			PX4_ISFINITE(_local_pos.z) ? _local_pos.z : 0.0f
		);
		PublishGimbalTracking(self_pos);
	}

	// 6. Check if Intercept is currently the active navigation mode
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_is_active = (vehicle_status.nav_state == _mode_id);
	}

	// 7. Mode execution
	if (_is_active) {
		if (!_was_active) {
			PX4_INFO("Intercept mode activated: initiating Lead-Pursuit & Basket Rendezvous");
			_was_active = true;
			_hold_valid = false;
			_guidance_phase = GuidancePhase::APPROACH;
			_pos_integral.zero();
			_firing_counter = 0;
			_net_fired = false;
			_last_cmd_valid = false;
			_speed_initialized = false;
			_last_approach_dist = -1.0f;
			_filt_closing_speed = 0.0f;
			_last_approach_time = 0;
		}

		if (_target_pos_valid) {
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};

			// Low-altitude guard: if vehicle is below 2.5m altitude, perform vertical climb-out before forward tilt
			const bool low_altitude_takeoff = (_local_pos.z_valid && _local_pos.z > -2.5f);

			if (low_altitude_takeoff) {
				vel_cmd = matrix::Vector3f(0.0f, 0.0f, -2.5f);
				yaw_cmd = _local_pos.heading;
			} else {
				// Compute Option A Hybrid Guidance: Lead-Pursuit Midcourse + 3D Basket Seating
				ComputeGuidance(vel_cmd, yaw_cmd, dt);
				_last_vel_cmd = vel_cmd;
				_last_yaw_cmd = yaw_cmd;
				_last_cmd_valid = true;
			}

			// Publish pure velocity setpoints. With acceleration=NAN, PX4 pos-controller creates smooth, stable tilt!
			trajectory_setpoint_s sp{};
			sp.timestamp = now;
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
			// Fallback: if target fix lost or not yet acquired, hold station
			if (!_hold_valid && _local_pos.xy_valid && _local_pos.z_valid) {
				_hold_position(0) = _local_pos.x;
				_hold_position(1) = _local_pos.y;
				_hold_position(2) = _local_pos.z;
				_hold_yaw = _local_pos.heading;
				_hold_valid = true;
			}

			if (_hold_valid) {
				trajectory_setpoint_s sp{};
				sp.timestamp = now;
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
			_guidance_phase = GuidancePhase::STANDBY;
			_pos_integral.zero();
			_firing_counter = 0;
			_last_cmd_valid = false;
			_speed_initialized = false;
			_last_approach_dist = -1.0f;
			_filt_closing_speed = 0.0f;
			_last_approach_time = 0;
		}
	}
}

int Intercept::print_status()
{
	PX4_INFO("Running: %s (100 Hz)", is_running() ? "yes" : "no");
	PX4_INFO("Registered: %s (mode_id=%d, arming_check_id=%d)",
		 (_mode_id != -1) ? "yes" : "no", _mode_id, _arming_check_id);
	PX4_INFO("Active: %s", _is_active ? "yes" : "no");

	const char *phase_str = "STANDBY";
	switch (_guidance_phase) {
	case GuidancePhase::APPROACH:   phase_str = "APPROACH (Lead-Pursuit Midcourse)"; break;
	case GuidancePhase::BLENDING:   phase_str = "BLENDING (Handover to Basket)"; break;
	case GuidancePhase::RENDEZVOUS: phase_str = "RENDEZVOUS (Precision 3D Basket Seating)"; break;
	case GuidancePhase::NET_FIRED:  phase_str = "NET_FIRED (TARGET CAPTURED!)"; break;
	default: phase_str = "STANDBY"; break;
	}
	PX4_INFO("Guidance Phase: %s", phase_str);

	const char *src_str = "NONE";
	switch (_target_source) {
	case TargetSource::GPS_FOLLOW_TARGET: src_str = "GPS (follow_target)"; break;
	case TargetSource::VISION:            src_str = "VISION (target_detection)"; break;
	case TargetSource::DEAD_RECKONING:    src_str = "DEAD_RECKONING (kinematic extrapolation)"; break;
	default:                              src_str = "NONE"; break;
	}
	PX4_INFO("Target Source: %s, Fix Valid: %s", src_str, _target_pos_valid ? "YES" : "NO");

	if (_target_pos_valid) {
		const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
		const matrix::Vector3f rel_pos = _target_pos - self_pos;
		const float dist_tgt = rel_pos.norm();
		const float age_s = (float)(hrt_elapsed_time(&_last_target_update)) * 1e-6f;

		PX4_INFO("Target Pos: [%.1f, %.1f, %.1f] m, Vel: [%.1f, %.1f, %.1f] m/s, Acc: [%.2f, %.2f, %.2f] m/s^2",
			 (double)_target_pos(0), (double)_target_pos(1), (double)_target_pos(2),
			 (double)_target_vel(0), (double)_target_vel(1), (double)_target_vel(2),
			 (double)_target_accel(0), (double)_target_accel(1), (double)_target_accel(2));

		PX4_INFO("Target Distance: %.1f m (age: %.2fs), Basket Dist R_v: %.2f m, V_c: %.1f m/s",
			 (double)dist_tgt, (double)age_s, (double)_last_R_v, (double)_last_V_c);

		PX4_INFO("Basket Vector r_v: [%.2f, %.2f, %.2f] m",
			 (double)_last_r_v(0), (double)_last_r_v(1), (double)_last_r_v(2));

		PX4_INFO("Command Vel: [%.2f, %.2f, %.2f] m/s, Yaw Cmd: %.1f deg",
			 (double)_last_vel_cmd(0), (double)_last_vel_cmd(1), (double)_last_vel_cmd(2),
			 (double)math::degrees(_last_yaw_cmd));

		PX4_INFO("Command Accel: [0.00, 0.00, 0.00] m/s^2, Yaw Cmd: %.1f deg",
			 (double)math::degrees(_last_yaw_cmd));

		PX4_INFO("Net Gun Status: %s (Envelope Counter: %u/10)",
			 _net_fired ? "FIRED!" : "READY", _firing_counter);

		const float r_xy = sqrtf(rel_pos(0) * rel_pos(0) + rel_pos(1) * rel_pos(1));
		PX4_INFO("Storm32 Gimbal Lock: Pitch: %.1f deg, Yaw: %.1f deg (NED Horizon-Stabilized)",
			 (double)math::degrees(atan2f(-rel_pos(2), math::max(r_xy, 0.01f))),
			 (double)math::degrees(atan2f(rel_pos(1), rel_pos(0))));

	} else {
		PX4_INFO("Target Fix: NO VALID DATA");
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
3D Blended APN and Virtual Capture Basket Rendezvous Intercept Flight Mode.
- 3D Augmented Proportional Navigation (APN) for closing pursuit (R_v > 5m)
- Precision Capture Basket Seating PID (R_v <= 5m) with velocity & acceleration matching
- Firing envelope detection with automatic net launcher deployment
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
