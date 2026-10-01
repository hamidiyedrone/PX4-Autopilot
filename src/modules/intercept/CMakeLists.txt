px4_add_module(
	MODULE modules__intercept
	MAIN intercept
	SRCS
		Intercept.cpp
		Intercept.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		geo
	)
