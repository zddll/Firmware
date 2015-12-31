include(nuttx/px4_impl_nuttx)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/px4io
	drivers/boards/px4fmu-v2
	drivers/rgbled
	drivers/mpu6000
	drivers/mpu9250
	drivers/lsm303d
	drivers/l3gd20
	drivers/hmc5883
	drivers/ms5611
	drivers/gps
	modules/sensors

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver

	#
	# General system control
	#
	modules/commander
	modules/mavlink
	modules/uavcan

	#
	# Estimation modules
	#
	modules/attitude_estimator_q

	#
	# Vehicle Control
	#
	modules/sat_att_control

	#
	# Logging
	#
	modules/sdlog2

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/controllib
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/mathlib
	lib/mathlib/math/filter
	lib/external_lgpl
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/conversion
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common 
	platforms/nuttx/px4_layer

)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_io_board
	px4io-v2
	)

set(config_extra_libs
	${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM4lf_math.a
	uavcan
	uavcan_stm32_driver
	)

set(config_io_extra_libs
	#${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM3l_math.a
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
