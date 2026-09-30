px4_add_module(
	MODULE drivers__target_vision
	MAIN target_vision
	SRCS
		TargetVision.cpp
		TargetVision.hpp
		target_vision_protocol.h
	MODULE_CONFIG
		module.yaml
	)
