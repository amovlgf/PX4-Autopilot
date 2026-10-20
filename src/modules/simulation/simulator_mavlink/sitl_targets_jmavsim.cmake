find_program(JAVA_ANT_PATH "ant")
find_package(Java COMPONENTS Runtime Development)

if(JAVA_ANT_PATH AND Java_JAVAC_EXECUTABLE AND Java_JAVA_EXECUTABLE)

	set(JMAVSIM_DIR "${PX4_SOURCE_DIR}/Tools/simulation/jmavsim")
	set(JMAVSIM_BUILD_DIR "${PX4_BINARY_DIR}/jmavsim")
	set(JMAVSIM_JAR "${JMAVSIM_BUILD_DIR}/out/production/jmavsim_run.jar")
	set(JMAVSIM_SCHEMA_DIR "${PX4_SOURCE_DIR}/src/modules/mavlink/mavlink/message_definitions/v1.0")

	px4_add_git_submodule(TARGET git_jmavsim PATH "${JMAVSIM_DIR}/jMAVSim")
	file(GLOB JMAVSIM_PATCHES CONFIGURE_DEPENDS "${JMAVSIM_DIR}/patches/*.patch")
	file(GLOB JMAVSIM_CONFIGS CONFIGURE_DEPENDS "${JMAVSIM_DIR}/config/*.properties")

	# Export pinned clean sources and apply parent-repository patches before
	# compiling. No generated classes, resources or jars enter the submodule.
	add_custom_command(
		OUTPUT "${JMAVSIM_JAR}"
		COMMAND ${PYTHON_EXECUTABLE} "${JMAVSIM_DIR}/prepare_jmavsim.py"
			--source "${JMAVSIM_DIR}/jMAVSim"
			--build-dir "${JMAVSIM_BUILD_DIR}"
			--mavlink-dir "${JMAVSIM_SCHEMA_DIR}"
		COMMAND "${JAVA_ANT_PATH}" -f "${JMAVSIM_BUILD_DIR}/build.xml" create_run_jar copy_res
		DEPENDS
			git_jmavsim
			"${JMAVSIM_DIR}/prepare_jmavsim.py"
			"${JMAVSIM_SCHEMA_DIR}/common.xml"
			"${JMAVSIM_SCHEMA_DIR}/minimal.xml"
			"${JMAVSIM_SCHEMA_DIR}/standard.xml"
			${JMAVSIM_PATCHES}
			${JMAVSIM_CONFIGS}
		USES_TERMINAL
		COMMENT "building jMAVSim in the build directory"
	)
	add_custom_target(build_jmavsim DEPENDS "${JMAVSIM_JAR}")

	add_custom_target(test_jmavsim_sensors
		COMMAND ${CMAKE_COMMAND} -E make_directory "${JMAVSIM_BUILD_DIR}/out/tests"
		COMMAND "${Java_JAVAC_EXECUTABLE}"
			-cp "${JMAVSIM_BUILD_DIR}/out/production/jMAVSim:${JMAVSIM_BUILD_DIR}/lib/*"
			-d "${JMAVSIM_BUILD_DIR}/out/tests"
			"${JMAVSIM_DIR}/tests/SensorModelTest.java"
		COMMAND "${Java_JAVA_EXECUTABLE}" -Djava.awt.headless=true
			"-Djmavsim.sensorConfig=${JMAVSIM_BUILD_DIR}/config/position_source_test.properties"
			-cp "${JMAVSIM_BUILD_DIR}/out/tests:${JMAVSIM_BUILD_DIR}/out/production/jMAVSim:${JMAVSIM_BUILD_DIR}/lib/*"
			me.drton.jmavsim.SensorModelTest "${JMAVSIM_BUILD_DIR}/mavlink/message_definitions/common.xml"
		DEPENDS build_jmavsim "${JMAVSIM_DIR}/tests/SensorModelTest.java"
		USES_TERMINAL
		COMMENT "checking jMAVSim sensor geometry, timing and wire format"
	)

	add_custom_target(jmavsim_run_symlink
		COMMAND ${CMAKE_COMMAND} -E make_directory "${PX4_BINARY_DIR}/rootfs"
		COMMAND ${CMAKE_COMMAND} -E create_symlink "${JMAVSIM_DIR}/jmavsim_run.sh" "${PX4_BINARY_DIR}/rootfs/jmavsim_run.sh"
		BYPRODUCTS "${PX4_BINARY_DIR}/rootfs/jmavsim_run.sh"
	)

	add_custom_target(jmavsim_iris
		COMMAND ${CMAKE_COMMAND} -E env
			PX4_SYS_AUTOSTART=10017 PX4_SIMULATOR=jmavsim PX4_SIM_MODEL=jmavsim_iris
			"JMAVSIM_BUILD_DIR=${JMAVSIM_BUILD_DIR}" "JMAVSIM_JAVA=${Java_JAVA_EXECUTABLE}"
			${PYTHON_EXECUTABLE} "${JMAVSIM_DIR}/run_jmavsim_sitl.py" $<TARGET_FILE:px4>
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 build_jmavsim jmavsim_run_symlink
			"${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10017_jmavsim_iris"
		COMMENT "launching px4 jmavsim_iris (SYS_AUTOSTART=10017)"
	)
	add_custom_target(jmavsim DEPENDS jmavsim_iris)
endif()
