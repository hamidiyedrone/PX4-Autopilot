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
	ScheduleOnInterval(10_ms); // 100 Hz guidance loop
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

	_has_fresh_visual = false;

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
			_has_fresh_visual = true;
		}
	}

	if (_visual_contact && (hrt_elapsed_time(&_last_visual_contact) > 1500_ms)) {
		PX4_WARN("Visual contact lost (>1.5s)");
		_visual_contact = false;
	}
}

void Intercept::ComputeMidcourseGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd)
{
	// 1. Aim point: 60m behind the target and 20m below the target (NED: +20m is down)
	const float target_speed = _target_vel.norm();
	matrix::Vector3f target_dir(1.f, 0.f, 0.f);

	if (target_speed > 1.f) {
		target_dir = _target_vel / target_speed;
	}

	const float dist_behind = 35.f;
	const float dist_below = 12.f;
	const matrix::Vector3f aim_point = _target_pos - target_dir * dist_behind + matrix::Vector3f(0.f, 0.f, dist_below);

	// 2. Intercept prediction based on high-speed pursuit cruise speed (38 m/s)
	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const matrix::Vector3f to_aim = aim_point - self_pos;
	const float dist_to_aim = to_aim.norm();

	const float cruise_speed = 38.f; // m/s (enables strong 74-76 deg tilt and rapid closure)
	const float time_to_intercept = dist_to_aim / cruise_speed;

	// Predicted position of the aim point at estimated arrival time
	const matrix::Vector3f intercept_point = aim_point + _target_vel * time_to_intercept;

	// 3. Direction and velocity command towards intercept point
	const matrix::Vector3f to_intercept = intercept_point - self_pos;
	const float dist_to_intercept = to_intercept.norm();

	if (dist_to_intercept > 10.f) {
		vel_cmd = (to_intercept / dist_to_intercept) * cruise_speed;
	} else {
		// Maintain a 31 m/s closing speed near the aim point so the drone keeps forward tilt
		vel_cmd = _target_vel + (to_intercept / math::max(dist_to_intercept, 0.1f)) * 6.0f;
	}

	// 4. Yaw command points camera/nose horizontally towards current target position
	const float dx = _target_pos(0) - _local_pos.x;
	const float dy = _target_pos(1) - _local_pos.y;
	yaw_cmd = atan2f(dy, dx);
}

void Intercept::ComputeTerminalVisualGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd)
{
	const hrt_abstime now = hrt_absolute_time();

	const float cx = _target_detection.bbox[0]; // [0.0, 1.0], center is 0.50
	const float cy = _target_detection.bbox[1]; // [0.0, 1.0], center is 0.50
	const float w  = _target_detection.bbox[2]; // Target bounding box width
	const float h  = _target_detection.bbox[3]; // Target bounding box height

	constexpr float DESIRED_CX = 0.50f;
	const float desired_w = math::constrain(_param_int_tgt_size.get(), 0.10f, 0.70f);
	const float desired_ytop = math::constrain(_param_int_tgt_ytop.get(), 0.20f, 0.80f);

	// Compute time step for optical rate damping
	float dt = 0.033f; // default 30 Hz
	if (_last_visual_time > 0 && now > _last_visual_time) {
		dt = math::constrain((float)(now - _last_visual_time) * 1e-6f, 0.005f, 0.2f);
	}
	_last_visual_time = now;

	// 1. Adaptive Base Speed Estimation (Learns unknown target speed dynamically)
	const float psi = _local_pos.heading;
	const float cos_psi = cosf(psi);
	const float sin_psi = sinf(psi);
	const float current_fwd_speed = _local_pos.vx * cos_psi + _local_pos.vy * sin_psi;

	if (!_speed_initialized || _adaptive_base_speed < 5.0f) {
		_adaptive_base_speed = math::constrain(current_fwd_speed, 15.0f, 35.0f);
		_speed_initialized = true;
	}

	const float err_w = desired_w - w;
	const float d_w_dt = (w - _last_w) / dt;
	const float rel_expansion_rate = d_w_dt / math::max(w, 0.01f); // (V_self - V_target) / D (1/s)

	// Base speed adapts dynamically using both size error integrator and expansion damping:
	// If w > desired_w (target too big, drone too close), base speed gradually reduces!
	// If w < desired_w (target too small, drone too far), base speed gradually increases!
	constexpr float K_adapt_err = 0.8f;
	constexpr float K_adapt_exp = 2.0f;
	_adaptive_base_speed += (K_adapt_err * err_w - K_adapt_exp * rel_expansion_rate) * dt;
	_adaptive_base_speed = math::constrain(_adaptive_base_speed, 15.0f, 36.0f);

	// Standoff closing / braking adjustment
	constexpr float K_p_w = 12.0f;
	constexpr float K_d_exp = 3.5f;
	const float v_standoff_adj = math::constrain(K_p_w * err_w - K_d_exp * rel_expansion_rate, -5.5f, 4.0f);

	// 2. Horizontal: Seat Drone Strictly on the Tail Centerline (Yaw + Lateral Roll)
	// cx > 0.50: target is right in FOV -> slide right (+V_lat) & yaw right
	// cx < 0.50: target is left in FOV -> slide left (-V_lat) & yaw left
	const float err_x = cx - DESIRED_CX;
	const float d_x_dt = (cx - _last_cx) / dt;

	const float k_p_lat = math::constrain(_param_int_kp_lat.get(), 0.5f, 15.0f);
	constexpr float K_d_lat = 0.4f;
	const float v_lat = math::constrain(k_p_lat * err_x + K_d_lat * d_x_dt, -3.5f, 3.5f);

	// Yaw points camera nose directly at target azimuth
	const float visual_yaw_err = math::constrain(err_x * 1.20f, -0.40f, 0.40f);
	yaw_cmd = _local_pos.heading + visual_yaw_err;

	// 3. Vertical: Seat Top Edge of BBox on Horizontal Centerline (y = desired_ytop)
	// y_top = cy - h/2.
	// y_top < desired_ytop: target is high up in frame (drone is underneath) -> err_y_top < 0 -> V_z < 0 (CLIMB!)
	// y_top > desired_ytop: target is low down in frame (drone is above) -> err_y_top > 0 -> V_z > 0 (DESCEND!)
	const float y_top = cy - (h * 0.5f);
	const float err_y_top = y_top - desired_ytop;
	const float d_ytop_dt = (y_top - _last_y_top) / dt;

	const float k_p_z = math::constrain(_param_int_kp_z.get(), 0.5f, 15.0f);
	constexpr float K_d_z = 0.5f;
	const float v_z = math::constrain(k_p_z * err_y_top + K_d_z * d_ytop_dt, -3.5f, 3.0f);

	// 4. Dynamic Climb Priority (Zero fixed speeds):
	// If the drone is below the target (err_y_top < 0, target high in frame),
	// we dynamically reduce forward tilt demand proportionally to vertical error.
	// This frees motor thrust to climb vertically! As y_top reaches desired_ytop, reduction vanishes.
	float climb_speed_reduction = 0.0f;
	if (err_y_top < 0.0f) {
		const float k_climb = math::constrain(_param_int_k_climb.get(), 0.0f, 30.0f);
		climb_speed_reduction = math::constrain(-err_y_top * k_climb, 0.0f, 8.0f);
	}

	const float v_fwd = math::max(_adaptive_base_speed + v_standoff_adj - climb_speed_reduction, 15.0f);

	// 5. Transform Body/Track Velocity Commands to World NED
	vel_cmd(0) = v_fwd * cos_psi - v_lat * sin_psi;
	vel_cmd(1) = v_fwd * sin_psi + v_lat * cos_psi;
	vel_cmd(2) = v_z;

	// 5. Net Deployment Trigger Condition (Automatic Fire Control)
	const bool locked_x = (fabsf(err_x) < 0.05f); // within 5% of horizontal center
	const bool locked_y = (fabsf(err_y_top) < 0.05f); // top edge touches center line (+/- 5%)
	const bool locked_dist = (fabsf(err_w) < 0.04f); // target nicely sized in net window

	if (locked_x && locked_y && locked_dist) {
		if (_lock_start_time == 0) {
			_lock_start_time = now;
		} else if (!_net_deployed && (hrt_elapsed_time(&_lock_start_time) > 400_ms)) {
			_net_deployed = true;
			_target_locked = true;
			PX4_WARN("=================================================");
			PX4_WARN(">>> TARGET LOCKED IN NET CONE! NET DEPLOYED! <<<");
			PX4_WARN("Intercept speed: ~%.1f m/s, cx: %.2f, y_top: %.2f, bbox_w: %.2f",
				 (double)current_fwd_speed, (double)cx, (double)y_top, (double)w);
			PX4_WARN("=================================================");
		}
	} else {
		_lock_start_time = 0;
		_target_locked = false;
	}

	_last_cx = cx;
	_last_cy = cy;
	_last_y_top = y_top;
	_last_w  = w;
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
			PX4_INFO("Intercept mode activated: pursuing target for visual acquisition");
			_was_active = true;
			_hold_valid = false;
			_guidance_state = GuidanceState::MIDCOURSE_GPS;
			_last_cmd_valid = false;
			_last_visual_time = 0;
			_lock_start_time = 0;
			_target_locked = false;
		}

		// State transitions: Midcourse GPS <-> Terminal Visual Handover
		if (_guidance_state == GuidanceState::MIDCOURSE_GPS) {
			const float dist_to_tgt = (_target_pos_valid) ?
				matrix::Vector2f(_target_pos(0) - _local_pos.x, _target_pos(1) - _local_pos.y).norm() : 999.f;

			// Hand over ONLY when visual contact is confirmed AND vehicle has entered the 40m approach funnel
			// (guarantees the drone has descended to the correct rear-lower altitude before visual lock)
			if (_visual_contact && _target_detection.detected && (dist_to_tgt < 40.0f)) {
				_guidance_state = GuidanceState::TERMINAL_VISUAL;
				_terminal_start_time = hrt_absolute_time();
				_last_visual_time = 0;
				_lock_start_time = 0;
				_target_locked = false;

				// Initialize baseline speed from drone's current forward ground speed
				const float psi = _local_pos.heading;
				const float current_fwd_speed = _local_pos.vx * cosf(psi) + _local_pos.vy * sinf(psi);
				_adaptive_base_speed = math::constrain(current_fwd_speed, 15.0f, 35.0f);
				_speed_initialized = true;

				_last_w = _target_detection.bbox[2];
				_last_cx = _target_detection.bbox[0];
				_last_cy = _target_detection.bbox[1];
				_last_y_top = _target_detection.bbox[1] - (_target_detection.bbox[3] * 0.5f);

				PX4_INFO("Handover: switched to TERMINAL_VISUAL (PURE OPTICAL) guidance (init speed: %.1f m/s, bbox_w: %.3f)",
					 (double)_adaptive_base_speed, (double)_target_detection.bbox[2]);
			}

		} else if (_guidance_state == GuidanceState::TERMINAL_VISUAL) {
			if (!_visual_contact) {
				_guidance_state = GuidanceState::MIDCOURSE_GPS;
				_last_cmd_valid = false;
				_last_visual_time = 0;
				_lock_start_time = 0;
				_target_locked = false;
				_speed_initialized = false;
				PX4_WARN("Visual contact lost (>1.5s), reverting to MIDCOURSE_GPS");
			}
		}

		if (_guidance_state == GuidanceState::TERMINAL_VISUAL) {
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};

			if (_has_fresh_visual && _target_detection.detected && (_target_detection.bbox[2] > 0.005f)) {
				matrix::Vector3f prev_vel = _last_vel_cmd;
				float prev_yaw = _last_yaw_cmd;
				hrt_abstime now = hrt_absolute_time();

				ComputeTerminalVisualGuidance(vel_cmd, yaw_cmd);

				// Compute command derivatives for intermediate 100 Hz predictive extrapolation
				if (_last_cmd_valid && _last_fresh_visual_time > 0) {
					float dt_frame = math::constrain((float)(now - _last_fresh_visual_time) * 1e-6f, 0.005f, 0.1f);
					_vel_cmd_dot = (vel_cmd - prev_vel) / dt_frame;
					_yaw_cmd_dot = matrix::wrap_pi(yaw_cmd - prev_yaw) / dt_frame;

					// Clamp derivative limits for safety (max 20 m/s^2 accel, max 3.0 rad/s yaw rate)
					for (int i = 0; i < 3; i++) {
						_vel_cmd_dot(i) = math::constrain(_vel_cmd_dot(i), -20.f, 20.f);
					}
					_yaw_cmd_dot = math::constrain(_yaw_cmd_dot, -3.0f, 3.0f);
				} else {
					_vel_cmd_dot.zero();
					_yaw_cmd_dot = 0.f;
				}

				_last_vel_cmd = vel_cmd;
				_last_yaw_cmd = yaw_cmd;
				_last_fresh_visual_time = now;
				_last_cmd_valid = true;

			} else if (_last_cmd_valid) {
				// Intermediate 100 Hz cycle (between camera frames):
				// Damped first-order predictive extrapolation based on command trend
				hrt_abstime now = hrt_absolute_time();
				float tau = (float)(now - _last_fresh_visual_time) * 1e-6f;
				float damping = expf(-tau / 0.040f); // 40ms decay constant

				vel_cmd = _last_vel_cmd + (_vel_cmd_dot * tau) * damping;
				yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + (_yaw_cmd_dot * tau) * damping);
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
			// Midcourse guidance: fly towards rear-lower intercept point and point nose at target
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};
			ComputeMidcourseGuidance(vel_cmd, yaw_cmd);

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
			_guidance_state = GuidanceState::MIDCOURSE_GPS;
			_last_cmd_valid = false;
			_last_visual_time = 0;
			_lock_start_time = 0;
			_target_locked = false;
			_speed_initialized = false;
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

	PX4_INFO("Guidance state: %s", (_guidance_state == GuidanceState::TERMINAL_VISUAL) ? "TERMINAL_VISUAL (PURE OPTICAL)" : "MIDCOURSE_GPS");

	if (_visual_contact) {
		PX4_INFO("Visual contact: YES (range: %.1f m, bbox: [%.2f, %.2f, %.2f, %.2f])",
			 (double)_visual_range, (double)_target_detection.bbox[0], (double)_target_detection.bbox[1],
			 (double)_target_detection.bbox[2], (double)_target_detection.bbox[3]);
		PX4_INFO("Fire Control: Locked=%s, Net Deployed=%s", _target_locked ? "YES" : "NO", _net_deployed ? "YES" : "NO");
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
Target intercept flight mode for autonomous drone interception.
Registers an external flight mode 'Intercept' with Commander.
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
