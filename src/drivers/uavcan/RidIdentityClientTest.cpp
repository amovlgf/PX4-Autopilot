/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include <iostream>
#include "RidIdentityClient.hpp"
#include "node/test_node.hpp"

namespace
{
// Only this test thread uses controlled time; PX4 background workers retain their real clock.
thread_local bool controlled_time = false;
thread_local hrt_abstime current_time = 1000000;
}

extern "C" hrt_abstime __real_hrt_absolute_time();
extern "C" hrt_abstime __wrap_hrt_absolute_time()
{
	return controlled_time ? current_time++ : __real_hrt_absolute_time();
}

namespace
{
class RidIdentityClientTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		current_time = 1000000;
		controlled_time = true;
		_clock.monotonic = current_time;
		ASSERT_EQ(_client.init(), 0);
		ASSERT_TRUE(_client.acceptArmStatus(124, current_time));
		rid_identity_request_s request{};
		request.timestamp = current_time;
		request.boot_id = 123;
		request.session_id = 456;
		request.request_id = 1;
		request.target_system = 255;
		request.target_component = 190;
		request.protocol_version = 2;
		request.flags = 1;
		ASSERT_TRUE(_requests.publish(request));
	}

	void TearDown() override
	{
		controlled_time = false;
	}

	SystemClockMock _clock;
	CanDriverMock _can{1, _clock};
	TestNode _node{_can, _clock, 1};
	RidIdentityClient _client{_node};
	uORB::Publication<rid_identity_request_s> _requests{ORB_ID(rid_identity_request)};
};
}

TEST_F(RidIdentityClientTest, AdvancingClockStillStartsTheFirstNodeInfoRequest)
{
	_client.update();

	// update() samples time before starting a cycle. A later clock read must not
	// turn that new cycle into an already expired snapshot before any CAN call.
	ASSERT_FALSE(_can.ifaces[0].tx.empty());
	uavcan::Frame request;
	ASSERT_TRUE(request.parse(_can.ifaces[0].tx.front().frame));
	EXPECT_EQ(request.getTransferType(), uavcan::TransferTypeServiceRequest);
	EXPECT_EQ(request.getDataTypeID().get(), uavcan::protocol::GetNodeInfo::DefaultDataTypeID);
	EXPECT_EQ(request.getDstNodeID().get(), 124);
}

TEST_F(RidIdentityClientTest, TimedOutServiceRetryStartsANewNodeInfoRequest)
{
	_client.update();
	ASSERT_FALSE(_can.ifaces[0].tx.empty());

	while (!_can.ifaces[0].tx.empty()) { _can.ifaces[0].tx.pop(); }

	// Let libuavcan deliver its real request-timeout callback, then reach the
	// client's retry time while the RID ArmStatus is still fresh.
	current_time += 700000;
	_clock.monotonic = current_time;
	_node.getScheduler().spinOnce();
	current_time += 300000;
	_clock.monotonic = current_time;
	_client.update();
	ASSERT_FALSE(_can.ifaces[0].tx.empty());
}

TEST_F(RidIdentityClientTest, TransactionStillExpiresAfterTwentySeconds)
{
	uORB::Subscription identity{ORB_ID(rid_identity)};
	_client.update();
	ASSERT_FALSE(_can.ifaces[0].tx.empty());

	// Keep the CAN heartbeat fresh and leave the service callback pending, so
	// update() itself must enforce the transaction deadline before waiting.
	current_time += 20000001;
	_clock.monotonic = current_time;
	ASSERT_TRUE(_client.acceptArmStatus(124, current_time));
	_client.update();

	rid_identity_s result{};
	ASSERT_TRUE(identity.copy(&result));
	EXPECT_EQ(result.result, rid_identity_s::RESULT_TIMEOUT);
	EXPECT_STREQ(result.error, "RID transaction deadline");
	EXPECT_EQ(result.rid_node_id, 124);
	EXPECT_EQ(result.request_id, 1u);
	EXPECT_EQ(result.session_id, 456u);
}
