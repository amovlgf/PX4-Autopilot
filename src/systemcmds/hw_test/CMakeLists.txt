add_library(hw_test_runner STATIC
	hw_test_runner.cpp
	hw_test_uart.cpp
	hw_test_can.cpp
	hw_test_spi.cpp
	hw_test_i2c.cpp
	hw_test_sd.cpp
	hw_test_pwm.cpp
	hw_test_um982.cpp
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

px4_add_module(
	MODULE systemcmds__hw_test
	MAIN hw_test
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
