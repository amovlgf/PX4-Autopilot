/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "DeviceBridge.hpp"
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <pthread.h>

namespace device_bridge
{
// All producers share this queue. Only CAN dequeue releases capacity; a sender
// timing out or restarting must not let uORB overwrite an unread request.
class RequestQueue
{
public:
	uint8_t enqueue(const device_bridge_request_s &request);
	bool dequeue(device_bridge_request_s &request);
private:
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	unsigned _queued{0};
	uORB::Publication<device_bridge_request_s> _publication{ORB_ID(device_bridge_request)};
	uORB::Subscription _subscription{ORB_ID(device_bridge_request)};
};

// Retain the subscriber across CAN driver restarts, including queued requests.
RequestQueue &requestQueue();

void discardResponses(uORB::Subscription &subscription);
bool takeResponse(PendingRequest &pending, uORB::Subscription &subscription, uint64_t now,
		  device_bridge_response_s &response);
} // namespace device_bridge
