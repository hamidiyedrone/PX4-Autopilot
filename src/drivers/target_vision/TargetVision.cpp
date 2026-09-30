/*
 * target_vision: link to the vision computer that detects the target.
 *
 * The vision computer (on the vehicle a Jetson running YOLO + PnP, in SITL
 * Tools/simulation/interceptor_sim/tools/sim_detector.py) sends one binary packet per
 * processed camera frame, over UART on the vehicle and over UDP in SITL. This driver
 * validates the packets (see target_vision_protocol.h), rotates the line of sight and the
 * target attitude from the camera optical frame into the body FRD frame with the mount
 * angles (TGV_MNT_*) and publishes target_detection.
 *
 * The line of sight starts at the camera and is published as such, without correcting to
 * the centre of gravity: the net launcher fires through the nose next to the camera, so
 * that is the reference the guidance wants.
 *
 *   target_vision start -d /dev/ttyS3 -b 921600    # vehicle
 *   target_vision start -u 15600                   # SITL
 */

#include "TargetVision.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

#include <errno.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

#if defined(__PX4_POSIX)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

static_assert(TARGET_VISION_PACKET_SIZE == 70, "unexpected packet size, the sender uses the documented layout");
static_assert(TARGET_VISION_PAYLOAD_SIZE == 64, "unexpected payload size");

TargetVision::TargetVision(const char *device, unsigned baudrate, int udp_port) :
	ModuleParams(nullptr),
	_baudrate(baudrate),
	_udp_port(udp_port)
{
	if (device != nullptr) {
		strncpy(_device, device, sizeof(_device) - 1);
	}
}

TargetVision::~TargetVision()
{
	closeTransport();
}

void TargetVision::updateMountRotation()
{
	// camera optical frame (x right in the image, y down, z along the optical axis) of a
	// forward looking camera, expressed in the body FRD frame
	const float forward[9] = {
		0.f, 0.f, 1.f,
		1.f, 0.f, 0.f,
		0.f, 1.f, 0.f
	};

	const matrix::Eulerf mount(math::radians(_param_tgv_mnt_roll.get()),
				   math::radians(_param_tgv_mnt_pitch.get()),
				   math::radians(_param_tgv_mnt_yaw.get()));

	_R_body_cam = matrix::Dcmf(mount) * matrix::Dcmf(forward);
}

bool TargetVision::openTransport()
{
	if (_udp_port >= 0) {
#if defined(__PX4_POSIX)
		_udp_fd = ::socket(AF_INET, SOCK_DGRAM, 0);

		if (_udp_fd < 0) {
			PX4_ERR("socket failed (%i)", errno);
			return false;
		}

		struct sockaddr_in addr {};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(_udp_port);

		if (::bind(_udp_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			PX4_ERR("bind to udp %i failed (%i)", _udp_port, errno);
			closeTransport();
			return false;
		}

		PX4_INFO("listening on udp %i", _udp_port);
		return true;
#else
		PX4_ERR("no udp on this platform");
		return false;
#endif
	}

	_uart = new device::Serial(_device, _baudrate);

	if (_uart == nullptr) {
		PX4_ERR("alloc failed");
		return false;
	}

	if (!_uart->open()) {
		PX4_ERR("opening %s failed", _device);
		return false;
	}

	PX4_INFO("%s open at %u baud", _device, _baudrate);
	return true;
}

void TargetVision::closeTransport()
{
	if (_uart != nullptr) {
		_uart->close();
		delete _uart;
		_uart = nullptr;
	}

#if defined(__PX4_POSIX)

	if (_udp_fd >= 0) {
		::close(_udp_fd);
		_udp_fd = -1;
	}

#endif
}

int TargetVision::readTransport(uint8_t *buffer, size_t length)
{
	if (_uart != nullptr) {
		return _uart->readAtLeast(buffer, length, 1, TRANSPORT_TIMEOUT_MS);
	}

#if defined(__PX4_POSIX)

	if (_udp_fd >= 0) {
		struct pollfd fds {};
		fds.fd = _udp_fd;
		fds.events = POLLIN;

		const int ret = poll(&fds, 1, TRANSPORT_TIMEOUT_MS);

		if (ret == 0) {
			return 0;	// timeout, no sender
		}

		if (ret < 0) {
			return (errno == EINTR) ? 0 : -1;
		}

		return (int)::recvfrom(_udp_fd, buffer, length, 0, nullptr, nullptr);
	}

#endif

	return -1;
}

void TargetVision::parse(const uint8_t *data, size_t length)
{
	for (size_t i = 0; i < length; i++) {
		if (_buffer_len == sizeof(_buffer)) {
			// nothing usable in a full buffer: drop the older half and keep going
			memmove(_buffer, _buffer + TARGET_VISION_PACKET_SIZE, sizeof(_buffer) - TARGET_VISION_PACKET_SIZE);
			_buffer_len -= TARGET_VISION_PACKET_SIZE;
		}

		_buffer[_buffer_len++] = data[i];
	}

	size_t consumed = 0;

	while (_buffer_len - consumed >= TARGET_VISION_PACKET_SIZE) {
		const uint8_t *candidate = _buffer + consumed;

		if (candidate[0] != TARGET_VISION_MAGIC0 || candidate[1] != TARGET_VISION_MAGIC1) {
			consumed++;
			continue;
		}

		if (candidate[2] != TARGET_VISION_VERSION || candidate[3] != TARGET_VISION_PAYLOAD_SIZE) {
			if (!_format_warned) {
				PX4_WARN("sender speaks version %u length %u, expected %u/%u", (unsigned)candidate[2], (unsigned)candidate[3],
					 (unsigned)TARGET_VISION_VERSION, TARGET_VISION_PAYLOAD_SIZE);
				_format_warned = true;
			}

			_format_errors++;
			consumed++;
			continue;
		}

		uint16_t crc_pkt;
		memcpy(&crc_pkt, candidate + TARGET_VISION_OFF_CRC, sizeof(crc_pkt));

		const uint16_t crc = target_vision_crc16(candidate + TARGET_VISION_CRC_OFFSET,
							 TARGET_VISION_CRC_LENGTH);

		if (crc != crc_pkt) {
			_crc_errors++;
			consumed++;	// not a packet start after all, resync from the next byte
			continue;
		}

		target_vision_detection_s packet;
		target_vision_decode(candidate, &packet);

		handlePacket(packet);
		consumed += TARGET_VISION_PACKET_SIZE;
	}

	if (consumed > 0) {
		memmove(_buffer, _buffer + consumed, _buffer_len - consumed);
		_buffer_len -= consumed;
	}
}

void TargetVision::handlePacket(const target_vision_detection_s &packet)
{
	const hrt_abstime now = hrt_absolute_time();

	_packets++;
	_last_packet = now;
	_last_latency_us = packet.latency_us;

	if (_frame_id_valid && packet.frame_id > _last_frame_id + 1) {
		_dropped_frames += packet.frame_id - _last_frame_id - 1;
	}

	_last_frame_id = packet.frame_id;
	_frame_id_valid = true;

	target_detection_s detection{};
	detection.timestamp = now;
	detection.timestamp_sample = (now > packet.latency_us) ? now - packet.latency_us : now;
	detection.frame_id = packet.frame_id;
	detection.latency_us = packet.latency_us;
	detection.detected = packet.flags & TARGET_VISION_FLAG_DETECTED;
	detection.confidence = packet.confidence / 255.f;

	if (detection.detected) {
		_detections++;

		matrix::Vector3f los_cam(packet.los);

		if (los_cam.longerThan(FLT_EPSILON)) {
			los_cam.normalize();
			const matrix::Vector3f los_body = _R_body_cam * los_cam;
			los_body.copyTo(detection.los_body);

		} else {
			detection.detected = false;	// a detection without a direction is of no use
		}

		detection.range_valid = (packet.flags & TARGET_VISION_FLAG_RANGE) && PX4_ISFINITE(packet.range_m);

		if (detection.range_valid) {
			detection.range_m = packet.range_m;
			detection.range_sigma_m = packet.range_sigma_m;
		}

		detection.attitude_valid = packet.flags & TARGET_VISION_FLAG_ATTITUDE;

		if (detection.attitude_valid) {
			matrix::Quatf q_body_target = matrix::Quatf(_R_body_cam) * matrix::Quatf(packet.q);
			q_body_target.normalize();
			q_body_target.copyTo(detection.q_target);
		}

		memcpy(detection.bbox, packet.bbox, sizeof(detection.bbox));
	}

	_target_detection_pub.publish(detection);
}

void TargetVision::run()
{
	updateMountRotation();

	if (!openTransport()) {
		return;
	}

	uint8_t buffer[4 * TARGET_VISION_PACKET_SIZE];

	while (!should_exit()) {
		const int bytes = readTransport(buffer, sizeof(buffer));

		if (bytes > 0) {
			parse(buffer, bytes);

		} else if (bytes < 0) {
			px4_usleep(10000);	// broken transport: do not spin
		}

		if (_parameter_update_sub.updated()) {
			parameter_update_s parameter_update;
			_parameter_update_sub.copy(&parameter_update);
			ModuleParams::updateParams();
			updateMountRotation();
		}
	}

	closeTransport();
}

int TargetVision::print_status()
{
	if (_udp_port >= 0) {
		PX4_INFO("udp %i", _udp_port);

	} else {
		PX4_INFO("%s at %u baud", _device, _baudrate);
	}

	if (_last_packet == 0) {
		PX4_INFO("no packet received yet");

	} else {
		PX4_INFO("last packet %.3f s ago, frame %u, sender latency %.1f ms",
			 (double)hrt_elapsed_time(&_last_packet) * 1e-6, _last_frame_id, (double)_last_latency_us * 1e-3);
	}

	PX4_INFO("packets %u, detections %u, dropped frames %u, crc errors %u, format errors %u",
		 _packets, _detections, _dropped_frames, _crc_errors, _format_errors);

	PX4_INFO("camera optical axis in body FRD: [%.2f %.2f %.2f]",
		 (double)_R_body_cam(0, 2), (double)_R_body_cam(1, 2), (double)_R_body_cam(2, 2));

	return 0;
}

int TargetVision::task_spawn(int argc, char *argv[])
{
	const int task_id = px4_task_spawn_cmd("target_vision", SCHED_DEFAULT, SCHED_PRIORITY_SLOW_DRIVER,
					       PX4_STACK_ADJUSTED(2200), (px4_main_t)&run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	_task_id = task_id;
	return 0;
}

TargetVision *TargetVision::instantiate(int argc, char *argv[])
{
	const char *device = nullptr;
	int baudrate = 921600;
	int udp_port = -1;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool error_flag = false;

	while ((ch = px4_getopt(argc, argv, "d:b:u:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}

			break;

		case 'u':
			udp_port = strtol(myoptarg, nullptr, 10);
			break;

		default:
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	if ((device == nullptr) == (udp_port < 0)) {
		print_usage("either a serial device (-d) or a udp port (-u) is required");
		return nullptr;
	}

	TargetVision *instance = new TargetVision(device, baudrate, udp_port);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int TargetVision::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int TargetVision::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Receives target detections from the vision computer and publishes them as the
target_detection uORB topic, one message per camera frame the sender processed.

The sender is a Jetson running the detector on the vehicle (UART) and
Tools/simulation/interceptor_sim/tools/sim_detector.py in SITL (UDP). The packet format is
documented in src/drivers/target_vision/target_vision_protocol.h.

The line of sight and the target attitude arrive in the camera optical frame and are
rotated into the body FRD frame with the mount angles TGV_MNT_ROLL/PITCH/YAW: they turn the
camera away from looking forward, so an interceptor whose camera looks along the thrust axis
(straight up in hover) has TGV_MNT_PITCH = 90.

### Examples
Vehicle, Jetson on TELEM2:
$ target_vision start -d /dev/ttyS2 -b 921600

SITL:
$ target_vision start -u 15600
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("target_vision", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 921600, 9600, 3000000, "Baudrate of the serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('u', -1, 1, 65535, "Listen on this udp port instead of a serial device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int target_vision_main(int argc, char *argv[])
{
	return TargetVision::main(argc, argv);
}
