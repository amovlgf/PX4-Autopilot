/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_platform_common/px4_config.h>
#include "mavlink_bridge_header.h"
#include <lib/device_bridge/DeviceBridge.hpp>
#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
#include <lib/device_bridge/DeviceBridgeQueue.hpp>
#include <uORB/Subscription.hpp>
#include <pthread.h>
#endif

class Mavlink;

class MavlinkDeviceBridge
{
public:
	explicit MavlinkDeviceBridge(Mavlink &mavlink);
	~MavlinkDeviceBridge();
	void handle(const mavlink_message_t &message);
	void update();
private:
	void send(const device_bridge_response_s &response);
	Mavlink &_mavlink;
#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
	const uint64_t _route_generation;
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	device_bridge::PendingRequest _pending;
	uORB::Subscription _response_sub{ORB_ID(device_bridge_response)};
#endif
};
