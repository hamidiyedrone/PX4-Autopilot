# Target detection from the vision computer (src/drivers/target_vision).
#
# One message per camera frame the sender processed, also when it found nothing
# (detected = false), so that the consumer sees the detector rate and its dropouts.
# The line of sight starts at the camera, not at the vehicle centre of gravity.

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# capture time of the frame, sender reported latency subtracted (microseconds)

float32[3] los_body		# unit vector from the camera to the target, body FRD frame
float32[4] q_target		# target attitude relative to the body frame (w, x, y, z), target frame is FRD: x out of the nose, y right wing
float32[4] bbox			# detection box in the image, normalized: centre x, centre y, width, height

float32 range_m			# distance from the camera to the target [m]
float32 range_sigma_m		# 1 sigma uncertainty of range_m [m]
float32 confidence		# detector confidence, 0..1

uint32 frame_id			# frame counter of the sender, gaps mean dropped frames
uint32 latency_us		# capture to send latency reported by the sender (microseconds)

bool detected			# false: frame processed, no target found. The fields below are then invalid
bool range_valid		# range_m and range_sigma_m are usable
bool attitude_valid		# q_target is usable
