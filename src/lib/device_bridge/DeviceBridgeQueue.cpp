/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "DeviceBridgeQueue.hpp"
#include <containers/LockGuard.hpp>

namespace device_bridge
{
namespace
{
RequestQueue queue;
}

RequestQueue &requestQueue() { return queue; }

uint8_t RequestQueue::enqueue(const device_bridge_request_s &request)
{
	LockGuard lock(_mutex);

	if (_queued == device_bridge_request_s::ORB_QUEUE_LENGTH) {
		return device_bridge_response_s::STATUS_BUSY;
	}

	// Subscribe before the first publication: a late uORB subscriber starts at
	// the newest sample, which is insufficient for a command queue.
	if (!_publication.advertise() || !_subscription.subscribe() || !_publication.publish(request)) {
		return device_bridge_response_s::STATUS_UNAVAILABLE;
	}

	++_queued;
	return device_bridge_response_s::STATUS_OK;
}

bool RequestQueue::dequeue(device_bridge_request_s &request)
{
	LockGuard lock(_mutex);

	if (_queued > 0 && _subscription.update(&request)) {
		--_queued;
		return true;
	}

	return false;
}

void discardResponses(uORB::Subscription &subscription)
{
	device_bridge_response_s discarded{};

	for (unsigned i = 0; i < device_bridge_response_s::ORB_QUEUE_LENGTH && subscription.update(&discarded); ++i) {}
}

bool takeResponse(PendingRequest &pending, uORB::Subscription &subscription, uint64_t now,
		  device_bridge_response_s &response)
{
	bool missed_response = false;
	subscription.subscribe();

	for (unsigned i = 0; i < device_bridge_response_s::ORB_QUEUE_LENGTH; ++i) {
		const unsigned generation = subscription.get_last_generation();

		if (!subscription.update(&response)) { break; }

		missed_response |= subscription.get_last_generation() - generation > 1;

		if (pending.matches(response) && response.payload_length <= sizeof(response.payload)) {
			pending.clear();
			return true;
		}
	}

	// A missing reply gives no evidence of device state. Report transport
	// TIMEOUT (unknown completion), never success and never retry the command.
	if (pending.active() && (missed_response || expired(pending.request().timestamp, now, RESPONSE_TIMEOUT_US))) {
		response = responseFor(pending.request(), device_bridge_response_s::STATUS_TIMEOUT, now);
		pending.clear();
		return true;
	}

	return false;
}
} // namespace device_bridge
