/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include "RidProtocolPublications.hpp"
#include <uORB/Subscription.hpp>

namespace
{
rid_identity_request_s identityRequest(uint32_t request_id = 1)
{
	rid_identity_request_s request{};
	request.timestamp = 1000000 + request_id;
	request.boot_id = 123;
	request.session_id = 456;
	request.request_id = request_id;
	request.target_system = 255;
	request.target_component = 190;
	request.protocol_version = 2;
	request.flags = 1;
	return request;
}
}

TEST(RidProtocolPublications, ExistingConsumerReadsRequestAfterPublishReturns)
{
	RidProtocolPublications publications;
	uORB::Subscription subscription{ORB_ID(rid_identity_request)};
	const auto expected = identityRequest();
	ASSERT_TRUE(publications.publishIdentityRequest(expected));

	// The CAN work queue consumes the request after the MAVLink handler returns.
	EXPECT_EQ(orb_exists(ORB_ID(rid_identity_request), 0), PX4_OK);
	rid_identity_request_s actual{};
	ASSERT_TRUE(subscription.update(&actual));
	EXPECT_EQ(actual.request_id, expected.request_id);
	EXPECT_EQ(actual.boot_id, expected.boot_id);
	EXPECT_EQ(actual.session_id, expected.session_id);
	EXPECT_EQ(actual.target_system, expected.target_system);
	EXPECT_EQ(actual.target_component, expected.target_component);
	EXPECT_EQ(actual.protocol_version, expected.protocol_version);
}

TEST(RidProtocolPublications, ConsumerStartingAfterRequestCanReadIt)
{
	RidProtocolPublications publications;
	const auto expected = identityRequest(2);
	ASSERT_TRUE(publications.publishIdentityRequest(expected));

	uORB::Subscription subscription{ORB_ID(rid_identity_request)};
	rid_identity_request_s actual{};
	ASSERT_TRUE(subscription.copy(&actual));
	EXPECT_EQ(actual.request_id, expected.request_id);
	EXPECT_EQ(actual.session_id, expected.session_id);
}

TEST(RidProtocolPublications, RepeatedRequestsRemainVisibleToDelayedConsumer)
{
	RidProtocolPublications publications;
	uORB::Subscription subscription{ORB_ID(rid_identity_request)};
	ASSERT_TRUE(publications.publishIdentityRequest(identityRequest(3)));
	const auto latest = identityRequest(4);
	ASSERT_TRUE(publications.publishIdentityRequest(latest));

	rid_identity_request_s actual{};
	ASSERT_TRUE(subscription.update(&actual));
	EXPECT_EQ(actual.request_id, latest.request_id);
	EXPECT_EQ(actual.timestamp, latest.timestamp);
	EXPECT_FALSE(subscription.updated());
}

TEST(RidProtocolPublications, ReadinessAndOwnerInvalidationRemainReadable)
{
	RidProtocolPublications publications;
	uORB::Subscription subscription{ORB_ID(rid_uom_status)};
	rid_uom_status_s ready{};
	ready.timestamp = 1000000;
	ready.boot_id = 123;
	ready.session_id = 456;
	ready.sequence = 1;
	ready.protocol_version = 2;
	ready.local_ready = 1;
	ready.state = rid_uom_status_s::STATE_READY;
	ASSERT_TRUE(publications.publishUomStatus(ready));

	rid_uom_status_s actual{};
	ASSERT_TRUE(subscription.update(&actual));
	EXPECT_EQ(actual.session_id, ready.session_id);
	EXPECT_EQ(actual.local_ready, 1);
	EXPECT_EQ(actual.state, ready.state);

	// A new reporting owner invalidates readiness before requesting its identity.
	ASSERT_TRUE(publications.publishUomStatus(rid_uom_status_s{}));
	EXPECT_EQ(orb_exists(ORB_ID(rid_uom_status), 0), PX4_OK);
	ASSERT_TRUE(subscription.update(&actual));
	EXPECT_EQ(actual.timestamp, 0);
	EXPECT_EQ(actual.boot_id, 0);
	EXPECT_EQ(actual.session_id, 0);
	EXPECT_EQ(actual.local_ready, 0);

	uORB::Subscription new_subscription{ORB_ID(rid_uom_status)};
	ASSERT_TRUE(new_subscription.copy(&actual));
	EXPECT_EQ(actual.session_id, 0);
	EXPECT_EQ(actual.local_ready, 0);
}
