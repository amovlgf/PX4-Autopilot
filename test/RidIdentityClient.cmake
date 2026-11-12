# Exercise the actual RID CAN client with native libuavcan and a simulated bus.
set(RID_CLIENT_LIBUAVCAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan/libuavcan)
set(RID_CLIENT_DSDL_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan/dsdl/uavcan)
set(RID_CLIENT_DSDL_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/rid_client_dsdl)

# Initialize inputs before the configure-time glob, including nonrecursive test checkouts.
# Reuse the driver's targets when UAVCAN is also enabled.
if(NOT TARGET git_uavcan_dsdl)
    px4_add_git_submodule(TARGET git_uavcan_dsdl PATH ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan/dsdl)
endif()
if(NOT TARGET git_uavcan_pydronecan)
    px4_add_git_submodule(TARGET git_uavcan_pydronecan PATH ${RID_CLIENT_LIBUAVCAN_DIR}/dsdl_compiler/pydronecan)
endif()

file(GLOB_RECURSE RID_CLIENT_DSDL_INPUTS CONFIGURE_DEPENDS ${RID_CLIENT_DSDL_DIR}/*.uavcan)
file(GLOB_RECURSE RID_CLIENT_LIBUAVCAN_SRCS CONFIGURE_DEPENDS ${RID_CLIENT_LIBUAVCAN_DIR}/src/*.cpp)
add_custom_command(
    OUTPUT ${RID_CLIENT_DSDL_OUTPUT}/generation.stamp
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RID_CLIENT_DSDL_OUTPUT}
    COMMAND ${PYTHON_EXECUTABLE} ${RID_CLIENT_LIBUAVCAN_DIR}/dsdl_compiler/libuavcan_dsdlc
        --outdir ${RID_CLIENT_DSDL_OUTPUT} ${RID_CLIENT_DSDL_DIR}
    COMMAND ${CMAKE_COMMAND} -E touch ${RID_CLIENT_DSDL_OUTPUT}/generation.stamp
    DEPENDS ${RID_CLIENT_DSDL_INPUTS} ${RID_CLIENT_LIBUAVCAN_DIR}/dsdl_compiler/libuavcan_dsdlc
    COMMENT "Generating native RID client DroneCAN types"
)
add_custom_target(rid_client_dsdl DEPENDS ${RID_CLIENT_DSDL_OUTPUT}/generation.stamp)
add_dependencies(rid_client_dsdl git_uavcan_dsdl git_uavcan_pydronecan)
px4_add_functional_gtest(
    SRC ${PX4_SOURCE_DIR}/src/drivers/uavcan/RidIdentityClientTest.cpp
    EXTRA_SRCS ${PX4_SOURCE_DIR}/src/drivers/uavcan/RidIdentityClient.cpp ${RID_CLIENT_LIBUAVCAN_SRCS}
    INCLUDES ${RID_CLIENT_LIBUAVCAN_DIR}/include ${RID_CLIENT_DSDL_OUTPUT} ${RID_CLIENT_LIBUAVCAN_DIR}/test
    COMPILE_FLAGS -DUAVCAN_CPP_VERSION=UAVCAN_CPP03 -DUAVCAN_EXCEPTIONS=1 -frtti
        -Wno-cast-align -Wno-deprecated-copy
)
if(TARGET functional-RidIdentityClient)
    add_dependencies(functional-RidIdentityClient rid_client_dsdl uorb_headers)
    target_link_options(functional-RidIdentityClient PRIVATE -Wl,--wrap=hrt_absolute_time)
endif()
