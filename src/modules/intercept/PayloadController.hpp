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

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/mount_orientation.h>

class PayloadController : public ModuleParams
{
public:
	explicit PayloadController(ModuleParams *parent);
	~PayloadController() = default;

	/**
	 * Main payload update function called cyclically from Intercept::Run()
	 *
	 * @param dt                 Time step in seconds
	 * @param is_active          Whether Intercept flight mode is active
	 * @param is_armed           Whether vehicle is armed
	 * @param target_dist        Distance to target in meters
	 * @param has_visual         Visual contact active
	 * @param visual_norm_y      Normalized optical vertical error [-1.0, 1.0]
	 * @param has_target_los     Target line-of-sight vector available (from GPS/Inertial)
	 * @param los_pitch_deg      Target line-of-sight elevation angle in body frame (degrees)
	 */
	void update(float dt, bool is_active, bool is_armed, float target_dist,
		    bool has_visual, float visual_norm_y,
		    bool has_target_los, float los_pitch_deg);

	void triggerFire();
	void reset();

	float getTurretPitchDeg() const { return _turret_pitch_deg; }
	float getGimbalPitchDeg() const { return _gimbal_pitch_deg; }
	bool isTriggerActive() const { return _trigger_active; }
	bool isNetFired() const { return _net_fired; }

private:
	void computeTurretPitch(float dt, float target_dist);
	void computeGimbalPitch(float dt, bool has_visual, float visual_norm_y, bool has_target_los, float los_pitch_deg);
	void checkAutoFire(bool is_active, bool is_armed, float target_dist, bool has_visual);
	void publishOutputs();

	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<mount_orientation_s> _mount_orientation_pub{ORB_ID(mount_orientation)};

	float _turret_pitch_deg{0.f};
	float _gimbal_pitch_deg{0.f};
	float _gmb_i_term{0.f};

	bool _trigger_active{false};
	hrt_abstime _trigger_start_time{0};
	bool _net_fired{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::INT_GMB_SERVO>) _param_int_gmb_servo,
		(ParamInt<px4::params::INT_TRT_SERVO>) _param_int_trt_servo,
		(ParamInt<px4::params::INT_TRG_SERVO>) _param_int_trg_servo,
		(ParamFloat<px4::params::INT_TRT_MIN_P>) _param_int_trt_min_p,
		(ParamFloat<px4::params::INT_TRT_MAX_P>) _param_int_trt_max_p,
		(ParamFloat<px4::params::INT_GMB_MIN_P>) _param_int_gmb_min_p,
		(ParamFloat<px4::params::INT_GMB_MAX_P>) _param_int_gmb_max_p,
		(ParamFloat<px4::params::INT_GMB_KP>) _param_int_gmb_kp,
		(ParamFloat<px4::params::INT_GMB_KI>) _param_int_gmb_ki,
		(ParamInt<px4::params::INT_TRT_MODE>) _param_int_trt_mode,
		(ParamFloat<px4::params::INT_TRT_OFFSET>) _param_int_trt_offset,
		(ParamFloat<px4::params::INT_TURRET_PITCH>) _param_int_turret_pitch,
		(ParamInt<px4::params::INT_AUTO_FIRE>) _param_int_auto_fire,
		(ParamFloat<px4::params::INT_FIRE_DIST>) _param_int_fire_dist,
		(ParamFloat<px4::params::INT_FIRE_TOL>) _param_int_fire_tol,
		(ParamFloat<px4::params::INT_TRG_PULSE>) _param_int_trg_pulse,
		(ParamInt<px4::params::INT_TRIGGER>) _param_int_trigger,
		(ParamFloat<px4::params::INT_STANDOFF>) _param_int_standoff,
		(ParamFloat<px4::params::INT_TRM_FWD_TOL>) _param_int_trm_fwd_tol
	)
};
