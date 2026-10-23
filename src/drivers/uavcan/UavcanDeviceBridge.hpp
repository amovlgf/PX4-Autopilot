/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <amov/transport/DeviceManagement.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/uavcan.hpp>
#include <uORB/Publication.hpp>
#include <lib/device_bridge/DeviceBridgeQueue.hpp>

class UavcanDeviceBridge
{
public:
	UavcanDeviceBridge(uavcan::INode &node, const uavcan::NodeStatusMonitor &monitor);
	void update();
private:
	using Service = amov::transport::DeviceManagement;
	using Callback = uavcan::MethodBinder<UavcanDeviceBridge *, void (UavcanDeviceBridge::*)(const uavcan::ServiceCallResult<Service> &)>;
	void response(const uavcan::ServiceCallResult<Service> &result);
	void reply(const device_bridge_request_s &request, uint8_t status);
	uavcan::INode &_node;
	const uavcan::NodeStatusMonitor &_monitor;
	uavcan::ServiceClient<Service, Callback> _client;
	device_bridge::PendingRequest _pending;
	uORB::Publication<device_bridge_response_s> _response_pub{ORB_ID(device_bridge_response)};
};
