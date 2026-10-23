/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "UavcanDeviceBridge.hpp"
#include <drivers/drv_hrt.h>
#include <string.h>

UavcanDeviceBridge::UavcanDeviceBridge(uavcan::INode &node, const uavcan::NodeStatusMonitor &monitor) :
	_node(node), _monitor(monitor), _client(node)
{
	_client.setCallback(Callback(this, &UavcanDeviceBridge::response));
	_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(device_bridge::SERVICE_TIMEOUT_MS));
}

void UavcanDeviceBridge::reply(const device_bridge_request_s &request, uint8_t status)
{
	_response_pub.publish(device_bridge::responseFor(request, status, hrt_absolute_time()));
}

void UavcanDeviceBridge::update()
{
	device_bridge_request_s request{};

	for (unsigned i = 0; i < device_bridge_request_s::ORB_QUEUE_LENGTH && device_bridge::requestQueue().dequeue(request); ++i) {
		if (!device_bridge::valid(request)) {
			reply(request, device_bridge_response_s::STATUS_INVALID);
			continue;
		}

		if (device_bridge::expired(request.timestamp, hrt_absolute_time(), device_bridge::QUEUE_TIMEOUT_US)) {
			reply(request, device_bridge_response_s::STATUS_TIMEOUT);
			continue;
		}

		if (_pending.active()) {
			reply(request, device_bridge_response_s::STATUS_BUSY);
			continue;
		}

		if (request.operation == device_bridge_request_s::OPERATION_LIST_NODES) {
			auto output = device_bridge::responseFor(request, device_bridge_response_s::STATUS_OK, hrt_absolute_time());
			_monitor.forEachNode([this, &output](uavcan::NodeID id, uavcan::NodeStatusMonitor::NodeStatus status) {
				device_bridge::appendOnlineNode(output, id.get(), _node.getNodeID().get(),
								status.mode == uavcan::protocol::NodeStatus::MODE_OFFLINE);
			});
			_response_pub.publish(output);
			continue;
		}

		const uavcan::NodeID target(request.node_id);

		if (target == _node.getNodeID() || _monitor.getNodeStatus(target).mode == uavcan::protocol::NodeStatus::MODE_OFFLINE) {
			reply(request, device_bridge_response_s::STATUS_UNAVAILABLE);
			continue;
		}

		Service::Request outgoing{};

		for (unsigned j = 0; j < request.payload_length; ++j) {
			outgoing.payload.push_back(request.payload[j]);
		}

		_pending.start(request);

		if (_client.call(target, outgoing) < 0) {
			_pending.clear();
			reply(request, device_bridge_response_s::STATUS_UNAVAILABLE);
		}
	}
}

void UavcanDeviceBridge::response(const uavcan::ServiceCallResult<Service> &result)
{
	if (!_pending.active() || result.getCallID().server_node_id.get() != _pending.request().node_id) {
		return;
	}

	const auto &request = _pending.request();
	auto output = device_bridge::responseFor(request, device_bridge_response_s::STATUS_TIMEOUT, hrt_absolute_time());

	if (result.isSuccessful()) {
		const auto &payload = result.getResponse().payload;

		if (payload.size() <= sizeof(output.payload)) {
			output.status = device_bridge_response_s::STATUS_OK;
			output.payload_length = payload.size();

			for (unsigned i = 0; i < payload.size(); ++i) { output.payload[i] = payload[i]; }

		} else {
			output.status = device_bridge_response_s::STATUS_INVALID;
		}
	}

	_pending.clear();
	_response_pub.publish(output);
}
