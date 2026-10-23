/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "MavlinkDeviceBridge.hpp"
#include "mavlink_main.h"
#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <string.h>

#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
namespace
{
uint64_t nextRouteGeneration()
{
	static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
	static uint64_t generation = 0;
	LockGuard lock(mutex);
	return ++generation;
}
}
#endif

MavlinkDeviceBridge::MavlinkDeviceBridge(Mavlink &mavlink) : _mavlink(mavlink)
#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
	, _route_generation(nextRouteGeneration())
#endif
{
}

MavlinkDeviceBridge::~MavlinkDeviceBridge()
{
#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
	pthread_mutex_destroy(&_mutex);
#endif
}

void MavlinkDeviceBridge::handle(const mavlink_message_t &message)
{
	mavlink_amov_device_request_t input{};
	mavlink_msg_amov_device_request_decode(&message, &input);

	if (input.target_system != _mavlink.get_system_id() || input.target_component != _mavlink.get_component_id()) {
		return;
	}

	device_bridge_request_s request{};
	request.timestamp = hrt_absolute_time();
	request.session_id = input.session_id;
	request.request_id = input.request_id;
	request.mavlink_instance = _mavlink.get_instance_id();
	request.source_system = message.sysid;
	request.source_component = message.compid;
	request.protocol_version = input.protocol_version;
	request.operation = input.operation;
	request.node_id = input.node_id;
	request.payload_length = input.payload_length;

	if (input.protocol_version != device_bridge_request_s::PROTOCOL_VERSION) {
		send(device_bridge::responseFor(request, device_bridge_response_s::STATUS_UNSUPPORTED, request.timestamp));
		return;
	}

	if (!device_bridge::valid(request)) {
		send(device_bridge::responseFor(request, device_bridge_response_s::STATUS_INVALID, request.timestamp));
		return;
	}

#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
	LockGuard lock(_mutex);

	if (_pending.active()) {
		send(device_bridge::responseFor(request, device_bridge_response_s::STATUS_BUSY, request.timestamp));
		return;
	}

	request.route_generation = _route_generation;
	memcpy(request.payload, input.payload, request.payload_length);
	device_bridge::discardResponses(_response_sub);
	_pending.start(request);
	const uint8_t status = device_bridge::requestQueue().enqueue(request);

	if (status != device_bridge_response_s::STATUS_OK) {
		_pending.clear();
		send(device_bridge::responseFor(request, status, request.timestamp));
	}

#else
	send(device_bridge::responseFor(request, device_bridge_response_s::STATUS_UNSUPPORTED, request.timestamp));
#endif
}

void MavlinkDeviceBridge::update()
{
#if defined(CONFIG_UAVCAN_DEVICE_BRIDGE)
	LockGuard lock(_mutex);
	device_bridge_response_s response{};

	if (device_bridge::takeResponse(_pending, _response_sub, hrt_absolute_time(), response)) {
		send(response);
	}

#endif
}

void MavlinkDeviceBridge::send(const device_bridge_response_s &response)
{
	mavlink_amov_device_response_t output{};
	output.session_id = response.session_id;
	output.request_id = response.request_id;
	output.target_system = response.target_system;
	output.target_component = response.target_component;
	output.protocol_version = device_bridge_request_s::PROTOCOL_VERSION;
	output.operation = response.operation;
	output.node_id = response.node_id;
	output.status = response.status;
	output.payload_length = response.payload_length;
	memcpy(output.payload, response.payload, response.payload_length);
	_mavlink.lock_send();
	mavlink_msg_amov_device_response_send_struct(_mavlink.get_channel(), &output);
	_mavlink.unlock_send();
}
