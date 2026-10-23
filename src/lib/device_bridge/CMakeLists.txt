############################################################################
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
############################################################################
px4_add_library(device_bridge DeviceBridgeQueue.cpp)
target_link_libraries(device_bridge PUBLIC uORB)
px4_add_functional_gtest(SRC DeviceBridgeQueueTest.cpp LINKLIBS device_bridge)
