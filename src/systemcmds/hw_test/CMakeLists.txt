add_library(hw_test_runner STATIC
	hw_test_runner.cpp
	hw_test_uart.cpp
	hw_test_can.cpp
	hw_test_spi.cpp
	hw_test_i2c.cpp
	hw_test_sd.cpp
	hw_test_pwm.cpp
)

add_dependencies(hw_test_runner uorb_headers)

target_include_directories(hw_test_runner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(hw_test_runner PRIVATE MODULE_NAME="hw_test")

target_link_libraries(hw_test_runner PRIVATE
	prebuild_targets
	px4_platform
	systemlib
	perf
	events_interface
	parameters_interface
	px4_layer
	uORB
)

if(CONFIG_BOARD_FLYCORE_Q_HW_TEST)
	set(flycore_protocol_dir "${PX4_SOURCE_DIR}/protocol/flycore_incoming_test")
	set(flycore_mavlink_dir "${PX4_SOURCE_DIR}/src/modules/mavlink/mavlink")
	set(flycore_mavlink_output "${CMAKE_CURRENT_BINARY_DIR}/flycore-mavlink")
	set(flycore_mavlink_header "${flycore_mavlink_output}/flycore_test/mavlink.h")
	px4_add_git_submodule(TARGET git_flycore_mavlink PATH "${flycore_mavlink_dir}")
	file(GLOB flycore_upstream_definitions CONFIGURE_DEPENDS "${flycore_mavlink_dir}/message_definitions/v1.0/*.xml")
	add_custom_command(
		OUTPUT "${flycore_mavlink_header}"
		COMMAND ${PYTHON_EXECUTABLE} "${flycore_protocol_dir}/generate.py"
			--definitions "${flycore_mavlink_dir}/message_definitions/v1.0"
			--pymavlink-root "${flycore_mavlink_dir}"
			--output "${flycore_mavlink_output}"
		DEPENDS git_flycore_mavlink
			"${flycore_protocol_dir}/generate.py"
			"${flycore_protocol_dir}/flycore_test.xml"
			${flycore_upstream_definitions}
		VERBATIM
	)
	add_custom_target(flycore_mavlink_c_generate DEPENDS "${flycore_mavlink_header}")
	add_dependencies(hw_test_runner flycore_mavlink_c_generate)
	target_include_directories(hw_test_runner SYSTEM PRIVATE "${flycore_mavlink_output}")
	target_sources(hw_test_runner PRIVATE hw_test_flycore_mavlink.cpp)
	set(hw_test_module_config flycore_q.yaml)
else()
	target_sources(hw_test_runner PRIVATE hw_test_um982.cpp)
endif()

px4_add_module(
	MODULE systemcmds__hw_test
	MAIN hw_test
	MODULE_CONFIG ${hw_test_module_config}
	COMPILE_FLAGS
	SRCS
		hw_test_main.cpp
	DEPENDS
		hw_test_runner
	)

px4_add_module(
	MODULE systemcmds__hw_test_all
	MAIN hw_test_all
	COMPILE_FLAGS
	SRCS
		hw_test_all_main.cpp
	DEPENDS
		hw_test_runner
	)
