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

	if (_has_follow_target && _map_ref.isInitialized()
	    && PX4_ISFINITE(_last_follow_target.lat) && PX4_ISFINITE(_last_follow_target.lon)) {

		if (_last_follow_target.timestamp != _last_target_rx_time) {
			_last_target_rx_time = _last_follow_target.timestamp;

			float x{0.f};
			float y{0.f};
			_map_ref.project(_last_follow_target.lat, _last_follow_target.lon, x, y);

			const hrt_abstime now = hrt_absolute_time();
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

	if (!new_sample_processed) {
		const bool follow_target_fresh = _has_follow_target && (hrt_elapsed_time(&_last_target_rx_time) < 1_s);

		if (_visual_contact && _target_detection.range_valid
		    && PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && PX4_ISFINITE(_local_pos.z)
		    && (!follow_target_fresh || _target_detection.range_m < 20.0f)) {
			// Visual tracking active (primary in terminal range <20m or when GPS follow_target is stale/unavailable)
			const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
			const matrix::Vector3f los_b(_target_detection.los_body[0], _target_detection.los_body[1], _target_detection.los_body[2]);
			const matrix::Quatf q_att(_vehicle_attitude.q);
			const matrix::Vector3f los_ned = q_att.rotateVector(los_b);
			const matrix::Vector3f vis_target_pos = self_pos + los_ned * _target_detection.range_m;

			const hrt_abstime now = hrt_absolute_time();
			if (_last_target_update > 0 && now > _last_target_update) {
				const float dt_meas = (float)(now - _last_target_update) * 1e-6f;
				if (dt_meas > 0.005f && dt_meas < 1.0f) {
					const matrix::Vector3f raw_vel = (vis_target_pos - _target_pos) / dt_meas;
					_target_vel = _target_vel * 0.7f + raw_vel * 0.3f;
				}
			}

			_target_pos = vis_target_pos;
			_target_pos_valid = true;
			_target_source = TargetSource::VISION;
			_last_target_update = now;

		} else if (follow_target_fresh && _target_pos_valid && dt > 0.001f && dt < 0.05f) {
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

void Intercept::ComputeBlendedAPNGuidance(matrix::Vector3f &a_cmd, float &yaw_cmd, float dt)
{
	// -------------------------------------------------------------------------
	// A. Hedef Yönelim ve Sanal Sepet (Capture Basket Geometry)
	// -------------------------------------------------------------------------
	matrix::Vector3f u_t;
	const float target_speed = _target_vel.norm();

	if (target_speed > 1.0f) {
		u_t = _target_vel.normalized();
	} else {
		u_t = matrix::Vector3f(1.0f, 0.0f, 0.0f);
	}

	// Interceptor hedef noktası: Talon'un d_offset = 2.5m tam arkasındaki sepet
	const float d_offset = _param_int_basket_d.get(); // 2.5m
	const matrix::Vector3f p_v = _target_pos - (u_t * d_offset);
	const matrix::Vector3f v_v = _target_vel;
	const matrix::Vector3f a_v = _target_accel;
	_p_v = p_v;
	_v_v = v_v;

	// -------------------------------------------------------------------------
	// B. Bağıl Kinematik ve Görüş Hattı (LOS Kinematics)
	// -------------------------------------------------------------------------
	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const matrix::Vector3f self_vel(_local_pos.vx, _local_pos.vy, _local_pos.vz);

	const matrix::Vector3f r_v = p_v - self_pos;
	const float R_v = r_v.norm();
	const matrix::Vector3f v_rel = v_v - self_vel;

	_last_R_v = R_v;

	_last_r_v = r_v;

	const matrix::Vector3f u_los = r_v / math::max(R_v, 0.01f);
	const float V_c = - (r_v.dot(v_rel)) / math::max(R_v, 0.01f);
	_last_V_c = V_c;

	// Regularized line-of-sight angular rate (core radius R_core = 2.5m prevents close-range singularity & fishtailing)
	const matrix::Vector3f omega_los = r_v.cross(v_rel) / math::max(R_v * R_v + 6.25f, 0.01f);

	// -------------------------------------------------------------------------
	// C. 3D Blended Güdüm Kanunu (APN + Rendezvous PID)
	// -------------------------------------------------------------------------
	// 1. Uzak Faz: 3D Augmented Proportional Navigation (R_v > 5.0m)
	const float N = _param_int_apn_n.get(); // 3.8
	const matrix::Vector3f a_t_perp = a_v - (u_los * a_v.dot(u_los));

	// Lateral APN steering acceleration (perpendicular to LOS: nullifies LOS rate omega_los)
	const matrix::Vector3f a_cmd_apn_lat = (omega_los.cross(u_los) * (N * math::max(V_c, 2.0f))) +
					       (a_t_perp * (0.5f * N));

	// Desired pursuit velocity: sprint closing velocity when far, smooth deceleration approaching basket
	const float V_c_des = math::constrain(0.70f * R_v, 0.0f, 15.0f);
	const matrix::Vector3f v_des_apn = v_v + (u_los * V_c_des);

	// 2. Yakın Faz: Sepete Oturma / Hız Eşitleme PID (R_v <= 5.0m)
	_pos_integral(0) = math::constrain(_pos_integral(0) + r_v(0) * dt, -5.0f, 5.0f);
	_pos_integral(1) = math::constrain(_pos_integral(1) + r_v(1) * dt, -5.0f, 5.0f);
	_pos_integral(2) = math::constrain(_pos_integral(2) + r_v(2) * dt, -5.0f, 5.0f);

	const matrix::Vector3f v_des_rdv = v_v + (r_v * 0.8f);
	const matrix::Vector3f a_cmd_rendezvous = a_v +
		matrix::Vector3f(6.5f * r_v(0), 6.5f * r_v(1), 7.0f * r_v(2)) +
		matrix::Vector3f(4.2f * v_rel(0), 4.2f * v_rel(1), 4.5f * v_rel(2)) +
		matrix::Vector3f(0.8f * _pos_integral(0), 0.8f * _pos_integral(1), 1.0f * _pos_integral(2));

	// 3. Yumuşak Geçiş (Blending Filter: 2.0m ile 6.0m arası)
	const float w = math::constrain((R_v - 2.0f) / 4.0f, 0.0f, 1.0f);
	const matrix::Vector3f v_des = (v_des_apn * w) + (v_des_rdv * (1.0f - w));

	_sp_vel = v_des;

	// Blended feedforward acceleration
	a_cmd = (a_cmd_apn_lat * w) + (a_cmd_rendezvous * (1.0f - w));

	// Guidance Phase Update
	if (R_v > 6.0f) {
		_guidance_phase = GuidancePhase::APN_CRUISE;
	} else if (R_v > 2.0f) {
		_guidance_phase = GuidancePhase::BLENDING;
	} else {
		_guidance_phase = _net_fired ? GuidancePhase::NET_FIRED : GuidancePhase::RENDEZVOUS;
	}

	// 4. İvme Doyumu ve Yumuşatma (Jerk Limiter)
	const float a_max = _param_int_acc_max.get(); // 55.0 m/s^2 (~5.6g)
	const float a_cmd_norm = a_cmd.norm();

	if (a_cmd_norm > a_max) {
		a_cmd = (a_cmd / a_cmd_norm) * a_max;
	}

	// Jerk / slew-rate limit on commanded acceleration (prevents violent pitch snap)
	const float max_jerk = 70.0f; // 70 m/s^3
	const matrix::Vector3f da = a_cmd - _prev_a_cmd;
	const float da_norm = da.norm();

	if (da_norm > max_jerk * dt) {
		a_cmd = _prev_a_cmd + (da / da_norm) * (max_jerk * dt);
	}
	_prev_a_cmd = a_cmd;

	// 5. Takip Hız Setpointi (Trajectory Velocity Setpoint)
	_sp_vel(2) = v_v(2);
	_sp_vel_initialized = true;

	// -------------------------------------------------------------------------
	// D. Yaw Setpoint: Gövde burnu hedefin anlık gidiş vektörüne kilitlenir
	// -------------------------------------------------------------------------
	yaw_cmd = atan2f(u_t(1), u_t(0));

	// -------------------------------------------------------------------------
	// E. Ağ Atma Karar Mekanizması Kontrolü (Firing Envelope)
	// -------------------------------------------------------------------------
	CheckFiringEnvelope(r_v, self_vel, u_t);
}

void Intercept::CheckFiringEnvelope(const matrix::Vector3f &r_v, const matrix::Vector3f &self_vel, const matrix::Vector3f &u_t)
{
	const matrix::Vector3f self_fwd(cosf(_local_pos.heading), sinf(_local_pos.heading), 0.0f);
	const matrix::Vector3f u_t_horiz = matrix::Vector3f(u_t(0), u_t(1), 0.0f).normalized();

	// Firing envelope criteria:
	// 1. Sanal sepete <= 0.65m yaklaşma (R_v <= 0.65m)
	const bool pos_ok = (r_v.norm() <= 0.65f);
	// 2. Hız farkı <= 1.3 m/s
	const bool vel_ok = ((self_vel - _target_vel).norm() <= 1.3f);
	// 3. Vektörel paralellik >= 0.985 (~10 dereceden dar açı)
	const bool align_ok = (self_fwd.dot(u_t_horiz) >= 0.985f);

	if (pos_ok && vel_ok && align_ok) {
		_firing_counter++;

		if (_firing_counter >= 10 && !_net_fired) {
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

	// 4. Respond to arming checks from Commander
	if (_arming_check_request_sub.updated()) {
		arming_check_request_s req;
		_arming_check_request_sub.copy(&req);
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
	if (_target_pos_valid) {
		const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
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
			PX4_INFO("Intercept mode activated: initiating 3D Blended APN pursuit");
			_was_active = true;
			_hold_valid = false;
			_guidance_phase = GuidancePhase::APN_CRUISE;
			_pos_integral.zero();
			_firing_counter = 0;
			_net_fired = false;
			_sp_vel_initialized = false;
		}

		if (_target_pos_valid) {
			matrix::Vector3f a_cmd;
			float yaw_cmd{_local_pos.heading};

			// Compute analytical 3D Blended APN + PID Rendezvous Guidance
			ComputeBlendedAPNGuidance(a_cmd, yaw_cmd, dt);

			_last_a_cmd = a_cmd;
			_last_yaw_cmd = yaw_cmd;

			// Feedforward acceleration + smooth pursuit velocity setpoint
			trajectory_setpoint_s sp{};
			sp.timestamp = now;
			// Altitude locked firmly to target basket height across all phases (rejects aerodynamic wing lift)
			sp.position[0] = NAN;
			sp.position[1] = NAN;
			sp.position[2] = _p_v(2);
			sp.acceleration[0] = a_cmd(0);
			sp.acceleration[1] = a_cmd(1);
			sp.acceleration[2] = _target_accel(2);
			sp.velocity[0] = _sp_vel(0);
			sp.velocity[1] = _sp_vel(1);
			sp.velocity[2] = _v_v(2);
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
			_sp_vel_initialized = false;
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
	case GuidancePhase::APN_CRUISE: phase_str = "APN_CRUISE (R_v > 5m)"; break;
	case GuidancePhase::BLENDING:   phase_str = "BLENDING (2m < R_v <= 5m)"; break;
	case GuidancePhase::RENDEZVOUS: phase_str = "RENDEZVOUS (R_v <= 2m)"; break;
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

		PX4_INFO("Command Accel: [%.2f, %.2f, %.2f] m/s^2, Yaw Cmd: %.1f deg",
			 (double)_last_a_cmd(0), (double)_last_a_cmd(1), (double)_last_a_cmd(2),
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
