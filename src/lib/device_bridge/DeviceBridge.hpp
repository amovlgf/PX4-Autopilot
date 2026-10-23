/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <uORB/topics/device_bridge_request.h>
#include <uORB/topics/device_bridge_response.h>
#include <stdint.h>

namespace device_bridge
{
// Expire requests waiting for the CAN worker before starting a new transfer.
static constexpr uint64_t QUEUE_TIMEOUT_US = 1000000;
static constexpr uint64_t RESPONSE_TIMEOUT_US = 2500000;
static constexpr unsigned SERVICE_TIMEOUT_MS = 1000;
static constexpr uint8_t MAX_NODE_ID = 127;

inline bool valid(const device_bridge_request_s &request)
{
	if (request.protocol_version != device_bridge_request_s::PROTOCOL_VERSION
	    || request.session_id == 0 || request.request_id == 0
	    || request.payload_length > sizeof(request.payload)) {
		return false;
	}

	if (request.operation == device_bridge_request_s::OPERATION_LIST_NODES) {
		return request.node_id == 0 && request.payload_length == 0;
	}

	return request.operation == device_bridge_request_s::OPERATION_TRANSFER
	       && request.node_id > 0 && request.node_id <= MAX_NODE_ID;
}

inline bool expired(uint64_t started, uint64_t now, uint64_t timeout)
{
	return started == 0 || now < started || now - started >= timeout;
}

inline device_bridge_response_s responseFor(const device_bridge_request_s &request, uint8_t status, uint64_t now)
{
	device_bridge_response_s response{};
	response.timestamp = now;
	response.request_timestamp = request.timestamp;
	response.route_generation = request.route_generation;
	response.session_id = request.session_id;
	response.request_id = request.request_id;
	response.mavlink_instance = request.mavlink_instance;
	response.target_system = request.source_system;
	response.target_component = request.source_component;
	response.operation = request.operation;
	response.node_id = request.node_id;
	response.status = status;
	return response;
}

// The owner serializes receive/update access. Internal routing never enters the device payload.
class PendingRequest
{
public:
	bool start(const device_bridge_request_s &request)
	{
		if (_active) { return false; }

		_request = request;
		_active = true;
		return true;
	}
	bool active() const { return _active; }
	const device_bridge_request_s &request() const { return _request; }
	void clear() { _active = false; }
	bool matches(const device_bridge_response_s &response) const
	{
		return _active && response.request_timestamp == _request.timestamp
		       && response.route_generation == _request.route_generation
		       && response.mavlink_instance == _request.mavlink_instance
		       && response.session_id == _request.session_id && response.request_id == _request.request_id
		       && response.target_system == _request.source_system && response.target_component == _request.source_component
		       && response.operation == _request.operation && response.node_id == _request.node_id;
	}
private:
	device_bridge_request_s _request{};
	bool _active{false};
};

// A complete node list fits in one response, independently of uORB multi-instance limits.
inline void appendOnlineNode(device_bridge_response_s &response, uint8_t node_id, uint8_t local_node_id, bool offline)
{
	if (!offline && node_id > 0 && node_id <= MAX_NODE_ID && node_id != local_node_id
	    && response.payload_length < sizeof(response.payload)) {
		response.payload[response.payload_length++] = node_id;
	}
}
} // namespace device_bridge
