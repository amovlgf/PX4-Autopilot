/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include "DeviceBridgeQueue.hpp"

namespace
{
device_bridge_request_s request(unsigned id = 1)
{
	device_bridge_request_s value{};
	value.timestamp = 1000000 + id;
	value.route_generation = 7;
	value.session_id = 123;
	value.request_id = id;
	value.mavlink_instance = 2;
	value.source_system = 255;
	value.source_component = 190;
	value.protocol_version = device_bridge_request_s::PROTOCOL_VERSION;
	value.operation = device_bridge_request_s::OPERATION_TRANSFER;
	value.node_id = 125;
	return value;
}

class DeviceBridgeQueue : public ::testing::Test
{
protected:
	void SetUp() override
	{
		device_bridge_request_s discarded{};

		while (device_bridge::requestQueue().dequeue(discarded)) {}

		ASSERT_TRUE(_responses.advertise());
	}
	void TearDown() override
	{
		device_bridge_request_s discarded{};

		while (device_bridge::requestQueue().dequeue(discarded)) {}
	}
	uORB::Publication<device_bridge_response_s> _responses{ORB_ID(device_bridge_response)};
};
}

TEST_F(DeviceBridgeQueue, SenderTimeoutCannotReleaseUnreadQueueCapacity)
{
	auto &queue = device_bridge::requestQueue();
	uORB::Subscription responses{ORB_ID(device_bridge_response)};
	device_bridge::discardResponses(responses);

	// The same link repeatedly times out while the CAN consumer is stalled.
	for (unsigned id = 1; id <= device_bridge_request_s::ORB_QUEUE_LENGTH; ++id) {
		const auto input = request(id);
		device_bridge::PendingRequest pending;
		ASSERT_TRUE(pending.start(input));
		ASSERT_EQ(queue.enqueue(input), device_bridge_response_s::STATUS_OK);
		device_bridge_response_s output{};
		ASSERT_TRUE(device_bridge::takeResponse(pending, responses,
							input.timestamp + device_bridge::RESPONSE_TIMEOUT_US, output));
		ASSERT_EQ(output.status, device_bridge_response_s::STATUS_TIMEOUT);
		ASSERT_FALSE(pending.active());
	}

	EXPECT_EQ(queue.enqueue(request(9)), device_bridge_response_s::STATUS_BUSY);
	EXPECT_EQ(queue.enqueue(request(10)), device_bridge_response_s::STATUS_BUSY);

	// Late CAN startup still sees every accepted request, in order.
	for (unsigned id = 1; id <= device_bridge_request_s::ORB_QUEUE_LENGTH; ++id) {
		device_bridge_request_s output{};
		ASSERT_TRUE(queue.dequeue(output));
		EXPECT_EQ(output.request_id, id);
		EXPECT_TRUE(device_bridge::expired(output.timestamp,
						   output.timestamp + device_bridge::RESPONSE_TIMEOUT_US, device_bridge::QUEUE_TIMEOUT_US));
	}

	device_bridge_request_s output{};
	EXPECT_FALSE(queue.dequeue(output));
	EXPECT_EQ(queue.enqueue(request(11)), device_bridge_response_s::STATUS_OK);
	ASSERT_TRUE(queue.dequeue(output));
	EXPECT_EQ(output.request_id, 11);
}

TEST_F(DeviceBridgeQueue, OnlyDequeuingReleasesOneQueueSlot)
{
	auto &queue = device_bridge::requestQueue();

	for (unsigned id = 1; id <= device_bridge_request_s::ORB_QUEUE_LENGTH; ++id) {
		ASSERT_EQ(queue.enqueue(request(id)), device_bridge_response_s::STATUS_OK);
	}

	device_bridge_request_s output{};
	ASSERT_TRUE(queue.dequeue(output));
	EXPECT_EQ(output.request_id, 1);
	ASSERT_EQ(queue.enqueue(request(9)), device_bridge_response_s::STATUS_OK);
	EXPECT_EQ(queue.enqueue(request(10)), device_bridge_response_s::STATUS_BUSY);

	for (unsigned id = 2; id <= 9; ++id) {
		ASSERT_TRUE(queue.dequeue(output));
		EXPECT_EQ(output.request_id, id);
	}

	EXPECT_FALSE(queue.dequeue(output));
}

TEST_F(DeviceBridgeQueue, LostCorrelatedReplyReportsTimeoutBeforeDeadline)
{
	uORB::Subscription responses{ORB_ID(device_bridge_response)};
	device_bridge::discardResponses(responses);
	device_bridge::PendingRequest pending;
	const auto input = request();
	ASSERT_TRUE(pending.start(input));
	ASSERT_TRUE(_responses.publish(device_bridge::responseFor(input, device_bridge_response_s::STATUS_OK, input.timestamp)));

	// Other links make the actual correlated reply fall out of the uORB queue.
	for (unsigned i = 0; i < device_bridge_response_s::ORB_QUEUE_LENGTH; ++i) {
		ASSERT_TRUE(_responses.publish(device_bridge::responseFor(request(i + 2), device_bridge_response_s::STATUS_OK,
					       input.timestamp)));
	}

	device_bridge_response_s output{};
	ASSERT_TRUE(device_bridge::takeResponse(pending, responses, input.timestamp + 1, output));
	EXPECT_EQ(output.status, device_bridge_response_s::STATUS_TIMEOUT);
	EXPECT_EQ(output.request_id, input.request_id);
	EXPECT_FALSE(pending.active());
}

TEST_F(DeviceBridgeQueue, SurvivingCorrelatedReplyCanCompleteDespiteUnrelatedLoss)
{
	uORB::Subscription responses{ORB_ID(device_bridge_response)};
	device_bridge::discardResponses(responses);
	device_bridge::PendingRequest pending;
	const auto input = request();
	ASSERT_TRUE(pending.start(input));

	for (unsigned i = 0; i < device_bridge_response_s::ORB_QUEUE_LENGTH; ++i) {
		ASSERT_TRUE(_responses.publish(device_bridge::responseFor(request(i + 2), device_bridge_response_s::STATUS_OK,
					       input.timestamp)));
	}

	auto expected = device_bridge::responseFor(input, device_bridge_response_s::STATUS_OK, input.timestamp);
	expected.payload_length = 2;
	expected.payload[0] = 255;
	expected.payload[1] = 0; // Opaque device failure/success data is not interpreted.
	ASSERT_TRUE(_responses.publish(expected));
	device_bridge_response_s output{};
	ASSERT_TRUE(device_bridge::takeResponse(pending, responses, input.timestamp + 1, output));
	EXPECT_EQ(output.status, device_bridge_response_s::STATUS_OK);
	EXPECT_EQ(output.payload_length, 2);
	EXPECT_EQ(output.payload[0], 255);
	EXPECT_EQ(output.payload[1], 0);
	EXPECT_FALSE(pending.active());
}

TEST_F(DeviceBridgeQueue, LateSubscriberStillTimesOutWhenOwnReplyIsGone)
{
	device_bridge::PendingRequest pending;
	const auto input = request();
	ASSERT_TRUE(pending.start(input));
	ASSERT_TRUE(_responses.publish(device_bridge::responseFor(input, device_bridge_response_s::STATUS_OK, input.timestamp)));

	for (unsigned i = 0; i < device_bridge_response_s::ORB_QUEUE_LENGTH; ++i) {
		ASSERT_TRUE(_responses.publish(device_bridge::responseFor(request(i + 2), device_bridge_response_s::STATUS_OK,
					       input.timestamp)));
	}

	uORB::Subscription responses{ORB_ID(device_bridge_response)};
	device_bridge_response_s output{};
	EXPECT_FALSE(device_bridge::takeResponse(pending, responses, input.timestamp + 1, output));
	ASSERT_TRUE(device_bridge::takeResponse(pending, responses,
						input.timestamp + device_bridge::RESPONSE_TIMEOUT_US, output));
	EXPECT_EQ(output.status, device_bridge_response_s::STATUS_TIMEOUT);
	EXPECT_FALSE(pending.active());
}

TEST_F(DeviceBridgeQueue, ReusedRequestIdDoesNotAcceptLateReply)
{
	uORB::Subscription responses{ORB_ID(device_bridge_response)};
	device_bridge::discardResponses(responses);
	const auto old = request();
	auto input = old;
	input.timestamp++;
	device_bridge::PendingRequest pending;
	ASSERT_TRUE(pending.start(input));
	ASSERT_TRUE(_responses.publish(device_bridge::responseFor(old, device_bridge_response_s::STATUS_OK, input.timestamp)));
	device_bridge_response_s output{};
	EXPECT_FALSE(device_bridge::takeResponse(pending, responses, input.timestamp + 1, output));
	EXPECT_TRUE(pending.active());
	ASSERT_TRUE(_responses.publish(device_bridge::responseFor(input, device_bridge_response_s::STATUS_OK, input.timestamp)));
	ASSERT_TRUE(device_bridge::takeResponse(pending, responses, input.timestamp + 1, output));
	EXPECT_EQ(output.status, device_bridge_response_s::STATUS_OK);
	EXPECT_EQ(output.request_timestamp, input.timestamp);
	EXPECT_FALSE(pending.active());
}
