/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include <string.h>
#include <lib/device_bridge/DeviceBridge.hpp>

namespace
{
device_bridge_request_s request()
{
	device_bridge_request_s value{};
	value.timestamp = 1000000;
	value.route_generation = 7;
	value.session_id = 123;
	value.request_id = 9;
	value.mavlink_instance = 2;
	value.source_system = 255;
	value.source_component = 190;
	value.protocol_version = device_bridge_request_s::PROTOCOL_VERSION;
	value.operation = device_bridge_request_s::OPERATION_TRANSFER;
	value.node_id = 125;
	return value;
}
}

TEST(DeviceBridge, AcceptsOpaquePayloadWithoutApplicationValidation)
{
	auto input = request();
	input.payload_length = sizeof(input.payload);

	for (unsigned i = 0; i < sizeof(input.payload); ++i) { input.payload[i] = i; }

	ASSERT_TRUE(device_bridge::valid(input));
	device_bridge::PendingRequest pending;
	ASSERT_TRUE(pending.start(input));
	EXPECT_EQ(memcmp(input.payload, pending.request().payload, sizeof(input.payload)), 0);
	input.payload_length = 0;
	EXPECT_TRUE(device_bridge::valid(input));
}

TEST(DeviceBridge, RejectsMalformedRoutingWithoutInspectingApplicationData)
{
	auto input = request();
	input.payload_length = 193;
	EXPECT_FALSE(device_bridge::valid(input));
	input = request(); input.node_id = 0;
	EXPECT_FALSE(device_bridge::valid(input));
	input = request(); input.node_id = 128;
	EXPECT_FALSE(device_bridge::valid(input));
	input = request(); input.protocol_version = 2;
	EXPECT_FALSE(device_bridge::valid(input));
	input = request(); input.session_id = 0;
	EXPECT_FALSE(device_bridge::valid(input));
	input = request(); input.request_id = 0;
	EXPECT_FALSE(device_bridge::valid(input));
	input = request(); input.operation = 2;
	EXPECT_FALSE(device_bridge::valid(input));
}

TEST(DeviceBridge, NodeDiscoveryHasNoTargetOrBody)
{
	auto input = request();
	input.operation = device_bridge_request_s::OPERATION_LIST_NODES;
	EXPECT_FALSE(device_bridge::valid(input));
	input.node_id = 0;
	EXPECT_TRUE(device_bridge::valid(input));
	input.payload_length = 1;
	EXPECT_FALSE(device_bridge::valid(input));
}

TEST(DeviceBridge, BusyDoesNotReplaceAnInflightTransaction)
{
	device_bridge::PendingRequest pending;
	auto first = request();
	ASSERT_TRUE(pending.start(first));
	auto second = first;
	second.request_id++;
	second.node_id++;
	EXPECT_FALSE(pending.start(second));
	EXPECT_EQ(pending.request().request_id, first.request_id);
	EXPECT_EQ(pending.request().node_id, first.node_id);
	pending.clear();
	EXPECT_TRUE(pending.start(second));
}

TEST(DeviceBridge, ResponseIsBoundToOriginalLinkSenderSessionAndNode)
{
	device_bridge::PendingRequest pending;
	const auto input = request();
	ASSERT_TRUE(pending.start(input));
	const auto expected = device_bridge::responseFor(input, device_bridge_response_s::STATUS_OK, 1000010);
	ASSERT_TRUE(pending.matches(expected));
	auto changed = expected; changed.route_generation++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.mavlink_instance++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.target_system++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.target_component++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.session_id++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.request_id++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.request_timestamp++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.node_id++;
	EXPECT_FALSE(pending.matches(changed));
	changed = expected; changed.operation++;
	EXPECT_FALSE(pending.matches(changed));
}

TEST(DeviceBridge, CompletedAndReusedRequestIdsDoNotConsumeLateReplies)
{
	device_bridge::PendingRequest pending;
	auto input = request();
	ASSERT_TRUE(pending.start(input));
	const auto late = device_bridge::responseFor(input, device_bridge_response_s::STATUS_OK, 1000010);
	pending.clear();
	EXPECT_FALSE(pending.matches(late));
	input.timestamp++;
	ASSERT_TRUE(pending.start(input));
	EXPECT_FALSE(pending.matches(late));
}

TEST(DeviceBridge, FullNodeDiscoveryExcludesLocalOfflineAndInvalidNodes)
{
	device_bridge_response_s output{};

	for (unsigned i = 1; i <= device_bridge::MAX_NODE_ID; ++i) {
		device_bridge::appendOnlineNode(output, i, 42, false);
	}

	ASSERT_EQ(output.payload_length, 126);
	EXPECT_EQ(output.payload[0], 1);
	EXPECT_EQ(output.payload[40], 41);
	EXPECT_EQ(output.payload[41], 43);
	EXPECT_EQ(output.payload[125], 127);
	device_bridge::appendOnlineNode(output, 0, 42, false);
	device_bridge::appendOnlineNode(output, 128, 42, false);
	device_bridge::appendOnlineNode(output, 17, 42, true);
	EXPECT_EQ(output.payload_length, 126);
}

TEST(DeviceBridge, QueuedRequestsExpireBeforeExecution)
{
	const auto started = request().timestamp;
	EXPECT_FALSE(device_bridge::expired(started, started + device_bridge::QUEUE_TIMEOUT_US - 1,
					    device_bridge::QUEUE_TIMEOUT_US));
	EXPECT_TRUE(device_bridge::expired(started, started + device_bridge::QUEUE_TIMEOUT_US,
					   device_bridge::QUEUE_TIMEOUT_US));
	EXPECT_TRUE(device_bridge::expired(started, started - 1, device_bridge::QUEUE_TIMEOUT_US));
	EXPECT_TRUE(device_bridge::expired(0, started, device_bridge::QUEUE_TIMEOUT_US));
}
