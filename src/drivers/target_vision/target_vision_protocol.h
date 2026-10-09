/*
 * Wire format between the vision computer and the target_vision driver.
 *
 * One packet per processed camera frame, 70 bytes, little endian, sent over UART on the
 * vehicle and over UDP in SITL. The sender must also send a packet when it found nothing
 * (flag DETECTED cleared), so that the receiver can tell "no target" from "no detector".
 *
 *   offset size type      field
 *   0      2    uint8[2]  magic 'T', 'V'
 *   2      1    uint8     version
 *   3      1    uint8     payload length in bytes (TARGET_VISION_PAYLOAD_SIZE)
 *   4      4    uint32    frame_id, frame counter, gaps tell the receiver about dropped frames
 *   8      4    uint32    latency_us, capture to send, measured by the sender
 *   12     1    uint8     flags, see below
 *   13     1    uint8     confidence, 0..255 maps to 0..1
 *   14     2    uint16    reserved, 0
 *   16     12   float[3]  los, unit vector camera to target in the camera optical frame
 *   28     4    float     range_m, camera to target
 *   32     4    float     range_sigma_m, 1 sigma of range_m
 *   36     16   float[4]  q, target attitude in the camera optical frame (w, x, y, z)
 *   52     16   float[4]  bbox, centre x, centre y, width, height, normalized to the image
 *   68     2    uint16    CRC-16-CCITT (init 0xffff) over offset 2 up to the CRC
 *
 * Frames:
 *   camera optical frame: x right in the image, y down in the image, z along the optical axis.
 *   target frame:         x out of the nose, y right wing, z down (FRD).
 *
 * There is no absolute time in the packet: the two clocks are not synchronised. The sender
 * reports how old the frame is instead (latency_us, capture to send), and the receiver
 * subtracts that from its own time.
 *
 * Mirrored in Tools/simulation/interceptor_sim/tools/sim_detector.py.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#define TARGET_VISION_MAGIC0		'T'
#define TARGET_VISION_MAGIC1		'V'
#define TARGET_VISION_VERSION		1

#define TARGET_VISION_FLAG_DETECTED	(1 << 0)	// a target was found, the fields below are valid
#define TARGET_VISION_FLAG_RANGE	(1 << 1)	// range_m, range_sigma_m are valid
#define TARGET_VISION_FLAG_ATTITUDE	(1 << 2)	// q is valid
#define TARGET_VISION_FLAG_BODY_LOS	(1 << 3)	// los and q are already expressed in vehicle body FRD frame (e.g. from gimballed seeker)

#define TARGET_VISION_PACKET_SIZE	70u
#define TARGET_VISION_PAYLOAD_SIZE	64u
#define TARGET_VISION_CRC_OFFSET	2u		// first byte covered by the CRC
#define TARGET_VISION_CRC_LENGTH	66u		// version up to the end of the payload

#define TARGET_VISION_OFF_FRAME_ID	4u
#define TARGET_VISION_OFF_LATENCY	8u
#define TARGET_VISION_OFF_FLAGS		12u
#define TARGET_VISION_OFF_CONFIDENCE	13u
#define TARGET_VISION_OFF_LOS		16u
#define TARGET_VISION_OFF_RANGE		28u
#define TARGET_VISION_OFF_RANGE_SIGMA	32u
#define TARGET_VISION_OFF_Q		36u
#define TARGET_VISION_OFF_BBOX		52u
#define TARGET_VISION_OFF_CRC		68u

/** the packet fields, unpacked into host layout */
struct target_vision_detection_s {
	uint32_t	frame_id;
	uint32_t	latency_us;
	uint8_t		flags;
	uint8_t		confidence;
	float		los[3];
	float		range_m;
	float		range_sigma_m;
	float		q[4];
	float		bbox[4];
};

/** decode a validated packet. The fields are read byte wise, the buffer needs no alignment */
static inline void target_vision_decode(const uint8_t *buffer, struct target_vision_detection_s *out)
{
	memcpy(&out->frame_id, buffer + TARGET_VISION_OFF_FRAME_ID, sizeof(out->frame_id));
	memcpy(&out->latency_us, buffer + TARGET_VISION_OFF_LATENCY, sizeof(out->latency_us));
	out->flags = buffer[TARGET_VISION_OFF_FLAGS];
	out->confidence = buffer[TARGET_VISION_OFF_CONFIDENCE];
	memcpy(out->los, buffer + TARGET_VISION_OFF_LOS, sizeof(out->los));
	memcpy(&out->range_m, buffer + TARGET_VISION_OFF_RANGE, sizeof(out->range_m));
	memcpy(&out->range_sigma_m, buffer + TARGET_VISION_OFF_RANGE_SIGMA, sizeof(out->range_sigma_m));
	memcpy(out->q, buffer + TARGET_VISION_OFF_Q, sizeof(out->q));
	memcpy(out->bbox, buffer + TARGET_VISION_OFF_BBOX, sizeof(out->bbox));
}

/** CRC-16-CCITT (init 0xffff, poly 0x1021) */
static inline uint16_t target_vision_crc16(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xffff;

	for (size_t i = 0; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;

		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			} else {
				crc = (uint16_t)(crc << 1);
			}
		}
	}

	return crc;
}
