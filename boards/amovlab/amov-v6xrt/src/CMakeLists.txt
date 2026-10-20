############################################################################
#
#   Copyright (c) 2016, 2019, 2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(board_sources
	init.c
	usb.c
	imxrt_clockconfig.c
	imxrt_flexspi_nor_boot.c
	imxrt_flexspi_nor_flash.c
)
if("${PX4_BOARD_LABEL}" STREQUAL "bootloader")
	add_compile_definitions(BOOTLOADER)
	add_library(drivers_board ${board_sources} bootloader_main.c)
	target_link_libraries(drivers_board PRIVATE nuttx_arch nuttx_drivers px4_layer arch_board_romapi bootloader)
	target_include_directories(drivers_board PRIVATE ${PX4_SOURCE_DIR}/platforms/nuttx/src/bootloader/common)
else()
	# Commander uses these parameter definitions even without the navigation
	# and sensor tasks. Keep their upstream owners; do not duplicate parameters.
	set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES
		${PX4_SOURCE_DIR}/src/modules/navigator/geofence_params.yaml
		${PX4_SOURCE_DIR}/src/modules/navigator/rtl_params.yaml
		${PX4_SOURCE_DIR}/src/modules/sensors/module.yaml
		${PX4_SOURCE_DIR}/src/modules/sensors/sensor_params.yaml
		${PX4_SOURCE_DIR}/src/modules/sensors/sensor_params_mag.yaml
		${PX4_SOURCE_DIR}/src/modules/sensors/vehicle_gps_position/module.yaml
	)
	px4_add_library(drivers_board ${board_sources} led.c imxrt_ocram_initialize.c)
	set_source_files_properties(imxrt_ocram_initialize.c PROPERTIES COMPILE_FLAGS -fno-builtin)
	target_link_libraries(drivers_board PRIVATE nuttx_arch nuttx_drivers px4_layer arch_board_romapi drivers__led)
endif()
