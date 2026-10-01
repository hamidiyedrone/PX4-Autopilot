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
				PX4_INFO("Visual contact acquired! Range: %.1f m", (double)td.range_m);
			}

			_visual_contact = true;
			_visual_range = td.range_m;
			_last_visual_contact = hrt_absolute_time();
		}
	}

	if (_visual_contact && (hrt_elapsed_time(&_last_visual_contact) > 1_s)) {
		PX4_INFO("Visual contact lost");
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

	const float dist_behind = 60.f;
	const float dist_below = 20.f;
	const matrix::Vector3f aim_point = _target_pos - target_dir * dist_behind + matrix::Vector3f(0.f, 0.f, dist_below);

	// 2. Intercept prediction based on our cruise speed (30 m/s)
	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const matrix::Vector3f to_aim = aim_point - self_pos;
	const float dist_to_aim = to_aim.norm();

	const float cruise_speed = 30.f; // m/s
	const float time_to_intercept = dist_to_aim / cruise_speed;

	// Predicted position of the aim point at estimated arrival time
	const matrix::Vector3f intercept_point = aim_point + _target_vel * time_to_intercept;

	// 3. Direction and velocity command towards intercept point
	const matrix::Vector3f to_intercept = intercept_point - self_pos;
	const float dist_to_intercept = to_intercept.norm();

	if (dist_to_intercept > 5.f) {
		vel_cmd = (to_intercept / dist_to_intercept) * cruise_speed;
	} else {
		vel_cmd = _target_vel;
	}

	// 4. Yaw command points camera/nose horizontally towards current target position
	const float dx = _target_pos(0) - _local_pos.x;
	const float dy = _target_pos(1) - _local_pos.y;
	yaw_cmd = atan2f(dy, dx);
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

	// 5. Update local position, target tracking, and visual detection
	_vehicle_local_position_sub.update(&_local_pos);
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
		}

		if (_target_pos_valid) {
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

		PX4_INFO("Target: [%.1f, %.1f, %.1f] m, vel: [%.1f, %.1f, %.1f] m/s, dist: %.1f m (age: %.2f s)",
			 (double)_target_pos(0), (double)_target_pos(1), (double)_target_pos(2),
			 (double)_target_vel(0), (double)_target_vel(1), (double)_target_vel(2),
			 (double)dist, (double)age_s);

	} else {
		PX4_INFO("Target: no valid fix");
	}

	if (_visual_contact) {
		PX4_INFO("Visual contact: YES (range: %.1f m)", (double)_visual_range);
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
