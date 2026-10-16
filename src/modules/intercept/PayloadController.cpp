/****************************************************************************
 *
 *   Copyright (c) 2026 Hamidiye Savunma Teknolojileri. All rights reserved.
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
 * 3. Neither the name Hamidiye Savunma Teknolojileri nor the names of its contributors may be
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

#include "PayloadController.hpp"
#include <px4_platform_common/log.h>

PayloadController::PayloadController(ModuleParams *parent) :
	ModuleParams(parent)
{
}

void PayloadController::reset()
{
	_trigger_active = false;
	_trigger_start_time = 0;
	_net_fired = false;
	_gmb_i_term = 0.0f;
}

void PayloadController::triggerFire()
{
	_trigger_active = true;
	_trigger_start_time = hrt_absolute_time();
	PX4_INFO("Net Gun Trigger FIRED!");
}

void PayloadController::update(float dt, bool is_active, bool is_armed, float target_dist,
			       bool has_visual, float visual_norm_y,
			       bool has_target_los, float los_pitch_deg)
{
	// 1. Check manual trigger parameter command (INT_TRIGGER=1)
	if (_param_int_trigger.get() == 1) {
		triggerFire();
		_param_int_trigger.set(0);
		_param_int_trigger.commit();
	}

	// 2. Manage trigger pulse safety timer (reset to safe after INT_TRG_PULSE seconds)
	if (_trigger_active) {
		const float pulse_duration = _param_int_trg_pulse.get();
		const float elapsed_s = (float)hrt_elapsed_time(&_trigger_start_time) * 1e-6f;

		if (elapsed_s >= pulse_duration) {
			_trigger_active = false;
		}
	}

	// Reset fired latch when vehicle disarms or mode becomes inactive
	if (!is_active || !is_armed) {
		_net_fired = false;
	}

	// 3. Compute pitch angles for turret and gimbal
	computeTurretPitch(dt, target_dist);
	computeGimbalPitch(dt, has_visual, visual_norm_y, has_target_los, los_pitch_deg);

	// 4. Check auto-fire criteria if enabled
	checkAutoFire(is_active, is_armed, target_dist, has_visual);

	// 5. Publish to actuator_servos and mount_orientation
	publishOutputs();
}

void PayloadController::computeTurretPitch(float dt, float target_dist)
{
	const float min_pitch = _param_int_trt_min_p.get();
	const float max_pitch = _param_int_trt_max_p.get();

	// Takip toleransı koridoru: INT_STANDOFF (örn. 5m) + INT_TRM_FWD_TOL (örn. 1m) -> [standoff - tol, standoff + tol]
	const float standoff = _param_int_standoff.get();
	const float fwd_tol = _param_int_trm_fwd_tol.get();
	const float max_tracking_dist = standoff + fwd_tol;
	const float min_tracking_dist = math::max(standoff - fwd_tol, 1.5f);

	const bool in_tracking_corridor = (target_dist >= min_tracking_dist && target_dist <= max_tracking_dist);

	if (_param_int_trt_mode.get() == 1 && in_tracking_corridor) {
		// Taret açısı = Gimbal açısı + Gimbal Ofset
		const float desired_pitch = _gimbal_pitch_deg + _param_int_trt_offset.get();

		if (dt > 0.005f) {
			const float alpha = math::constrain(dt / 0.10f, 0.05f, 1.0f);
			_turret_pitch_deg = math::constrain(_turret_pitch_deg + alpha * (desired_pitch - _turret_pitch_deg),
							    min_pitch, max_pitch);
		} else {
			_turret_pitch_deg = math::constrain(desired_pitch, min_pitch, max_pitch);
		}

	} else {
		// Takip toleransı dışındayken veya Manuel Modda: Manuel/stowed açıya dön (varsayılan 0°)
		const float manual_pitch = _param_int_turret_pitch.get();
		if (dt > 0.005f) {
			const float alpha = math::constrain(dt / 0.15f, 0.05f, 1.0f);
			_turret_pitch_deg = math::constrain(_turret_pitch_deg + alpha * (manual_pitch - _turret_pitch_deg),
							    min_pitch, max_pitch);
		} else {
			_turret_pitch_deg = math::constrain(manual_pitch, min_pitch, max_pitch);
		}
	}
}

void PayloadController::computeGimbalPitch(float dt, bool has_visual, float visual_norm_y,
					   bool has_target_los, float los_pitch_deg)
{
	const float min_gmb = _param_int_gmb_min_p.get();
	const float max_gmb = _param_int_gmb_max_p.get();

	if (has_visual && dt > 0.005f) {
		// Optical tracking closed-loop PI controller
		// visual_norm_y is positive when target center is below crosshair (cy > 0.5)
		const float err_y = visual_norm_y;

		// Anti-windup integration (clamped to [-1.0, 1.0])
		_gmb_i_term = math::constrain(_gmb_i_term + err_y * dt, -1.0f, 1.0f);

		const float kp = _param_int_gmb_kp.get();
		const float ki = _param_int_gmb_ki.get();
		const float pitch_rate_cmd = kp * err_y + ki * _gmb_i_term;

		// Integrate rate into gimbal pitch angle
		_gimbal_pitch_deg = math::constrain(_gimbal_pitch_deg + pitch_rate_cmd * dt, min_gmb, max_gmb);

	} else {
		// Reset integrator when visual contact is not active
		_gmb_i_term = 0.0f;

		if (has_target_los) {
			// Kinematic Line-of-Sight tracking from relative 3D position (GPS/Inertial)
			const float alpha = (dt > 0.005f) ? math::constrain(dt / 0.08f, 0.05f, 1.0f) : 1.0f;
			_gimbal_pitch_deg = math::constrain(_gimbal_pitch_deg + alpha * (los_pitch_deg - _gimbal_pitch_deg),
							    min_gmb, max_gmb);
		} else {
			// Return smoothly to level forward boresight (0 deg)
			const float alpha = (dt > 0.005f) ? math::constrain(dt / 0.25f, 0.05f, 1.0f) : 1.0f;
			_gimbal_pitch_deg = math::constrain(_gimbal_pitch_deg + alpha * (0.0f - _gimbal_pitch_deg),
							    min_gmb, max_gmb);
		}
	}
}

void PayloadController::checkAutoFire(bool is_active, bool is_armed, float target_dist, bool has_visual)
{
	if (!is_active || !is_armed || _param_int_auto_fire.get() != 1 || _net_fired) {
		return;
	}

	const float standoff = _param_int_standoff.get();
	const float fwd_tol = _param_int_trm_fwd_tol.get();
	const float max_fire_dist = standoff + fwd_tol;
	const float min_fire_dist = math::max(standoff - fwd_tol, 1.5f);

	// Takip toleransı koridorunda ve görsel temas varsa otomatik ateşle
	if (target_dist >= min_fire_dist && target_dist <= max_fire_dist && has_visual) {
		triggerFire();
		_net_fired = true;
	}
}

void PayloadController::publishOutputs()
{
	// 1. Normalize Turret Pitch to [-1.0, 1.0]
	const float trt_min = _param_int_trt_min_p.get();
	const float trt_max = _param_int_trt_max_p.get();
	const float trt_span = (trt_max > trt_min + 1.0f) ? (trt_max - trt_min) : 45.0f;
	const float trt_norm = math::constrain((_turret_pitch_deg - trt_min) / trt_span * 2.0f - 1.0f, -1.0f, 1.0f);

	// 2. Normalize Gimbal Pitch to [-1.0, 1.0]
	const float gmb_min = _param_int_gmb_min_p.get();
	const float gmb_max = _param_int_gmb_max_p.get();
	const float gmb_span = (gmb_max > gmb_min + 1.0f) ? (gmb_max - gmb_min) : 120.0f;
	const float gmb_norm = math::constrain((_gimbal_pitch_deg - gmb_min) / gmb_span * 2.0f - 1.0f, -1.0f, 1.0f);

	// 3. Trigger Output: +1.0 (Fire) or -1.0 (Safe)
	const float trg_norm = _trigger_active ? 1.0f : -1.0f;

	// 4. Publish actuator_servos
	actuator_servos_s servos{};
	servos.timestamp = hrt_absolute_time();
	servos.timestamp_sample = servos.timestamp;

	for (int i = 0; i < 8; ++i) {
		servos.control[i] = NAN;
	}

	const int gmb_idx = _param_int_gmb_servo.get() - 1;
	if (gmb_idx >= 0 && gmb_idx < 8) {
		servos.control[gmb_idx] = gmb_norm;
	}

	const int trt_idx = _param_int_trt_servo.get() - 1;
	if (trt_idx >= 0 && trt_idx < 8) {
		servos.control[trt_idx] = trt_norm;
	}

	const int trg_idx = _param_int_trg_servo.get() - 1;
	if (trg_idx >= 0 && trg_idx < 8) {
		servos.control[trg_idx] = trg_norm;
	}

	_actuator_servos_pub.publish(servos);

	// 5. Publish mount_orientation for GCS HUD telemetry
	mount_orientation_s mount{};
	mount.timestamp = servos.timestamp;
	mount.attitude_euler_angle[0] = 0.0f;
	mount.attitude_euler_angle[1] = math::radians(_turret_pitch_deg);
	mount.attitude_euler_angle[2] = 0.0f;
	_mount_orientation_pub.publish(mount);
}
