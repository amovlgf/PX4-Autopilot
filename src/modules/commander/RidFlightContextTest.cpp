/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include <lib/remote_id/RidFlightContext.hpp>

TEST(RidFlightContext, LocksActualTakeoffUtcAndKeepsContextAfterLanding)
{
	rid::FlightContext context;
	sensor_gps_s gps{};
	gps.timestamp = 10000000;
	gps.timestamp_time_relative = -200000;
	gps.time_utc_usec = 1793764800000000ULL;
	gps.fix_type = 3;
	EXPECT_EQ(context.state().flight_sequence, 0u);
	context.takeoff(10100000, gps);
	EXPECT_EQ(context.state().flight_sequence, 1u);
	EXPECT_EQ(context.state().takeoff_utc_us, gps.time_utc_usec + 300000);
	EXPECT_EQ(context.state().flags, 7);
	const auto first = context.state();
	context.takeoff(10200000, gps);
	EXPECT_EQ(context.state().takeoff_utc_us, first.takeoff_utc_us);
	EXPECT_EQ(context.state().flight_sequence, 1u);
	context.land(11000000);
	EXPECT_EQ(context.state().flags, rid_flight_status_s::FLAG_TAKEOFF_UTC_VALID);
	EXPECT_EQ(context.state().takeoff_utc_us, first.takeoff_utc_us);
	gps.timestamp = 12000000;
	context.takeoff(12100000, gps);
	EXPECT_EQ(context.state().flight_sequence, 2u);
	EXPECT_EQ(context.state().flags, 7);
}

TEST(RidFlightContext, MissingUtcAtTakeoffIsNeverBackfilled)
{
	rid::FlightContext context;
	sensor_gps_s gps{};
	context.takeoff(10000000, gps);
	EXPECT_EQ(context.state().takeoff_utc_us, 0u);
	EXPECT_EQ(context.state().flags, 3);
	gps.timestamp = 10100000;
	gps.time_utc_usec = 1793764800000000ULL;
	gps.fix_type = 3;
	context.takeoff(10200000, gps);
	EXPECT_EQ(context.state().takeoff_utc_us, 0u);
	context.land(11000000);
	context.takeoff(12000000, gps); // A stale GNSS sample cannot timestamp a new flight.
	EXPECT_EQ(context.state().flight_sequence, 2u);
	EXPECT_EQ(context.state().takeoff_utc_us, 0u);
	context.land(13000000);
	gps.timestamp = 14000000;
	gps.timestamp_time_relative = 1;
	context.takeoff(14000000, gps); // Reject a future-dated UTC sample.
	EXPECT_EQ(context.state().takeoff_utc_us, 0u);
	context.land(15000000);
	gps.timestamp = UINT64_MAX;
	gps.timestamp_time_relative = INT32_MIN;
	context.takeoff(16000000, gps);
	EXPECT_EQ(context.state().takeoff_utc_us, 0u);
}
