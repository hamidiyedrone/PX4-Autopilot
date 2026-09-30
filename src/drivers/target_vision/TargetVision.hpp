/*
 * target_vision: link to the vision computer that detects the target, see
 * target_vision_protocol.h for the wire format.
 */

#pragma once

#include "target_vision_protocol.h"

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/target_detection.h>

using namespace time_literals;

class TargetVision : public ModuleBase<TargetVision>, public ModuleParams
{
public:
	TargetVision(const char *device, unsigned baudrate, int udp_port);
	~TargetVision() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TargetVision *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	bool openTransport();
	void closeTransport();

	/** blocking read of up to length bytes, at most TRANSPORT_TIMEOUT_MS. < 0 on error */
	int readTransport(uint8_t *buffer, size_t length);

	/** find complete packets in the received byte stream and hand them to handlePacket() */
	void parse(const uint8_t *data, size_t length);

	void handlePacket(const target_vision_detection_s &packet);

	/** camera optical frame -> body FRD, from the mount angles */
	void updateMountRotation();

	static constexpr int TRANSPORT_TIMEOUT_MS{100};

	char _device[32] {};
	unsigned _baudrate{0};
	device::Serial *_uart{nullptr};

	int _udp_port{-1};
	int _udp_fd{-1};

	matrix::Dcmf _R_body_cam{};

	uint8_t _buffer[2 * TARGET_VISION_PACKET_SIZE] {};
	size_t _buffer_len{0};

	// statistics, for print_status()
	uint32_t _packets{0};
	uint32_t _detections{0};
	uint32_t _crc_errors{0};
	uint32_t _format_errors{0};
	bool _format_warned{false};
	uint32_t _dropped_frames{0};
	uint32_t _last_frame_id{0};
	bool _frame_id_valid{false};
	uint32_t _last_latency_us{0};
	hrt_abstime _last_packet{0};

	uORB::Publication<target_detection_s> _target_detection_pub{ORB_ID(target_detection)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TGV_MNT_ROLL>)  _param_tgv_mnt_roll,
		(ParamFloat<px4::params::TGV_MNT_PITCH>) _param_tgv_mnt_pitch,
		(ParamFloat<px4::params::TGV_MNT_YAW>)   _param_tgv_mnt_yaw
	)
};
