/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once
#include "RidState.hpp"
#include <uORB/topics/rid_flight_status.h>
#include <uORB/topics/sensor_gps.h>

namespace rid
{
class FlightContext
{
public:
	void takeoff(uint64_t now, const sensor_gps_s &gps)
	{
		if (_state.flags & rid_flight_status_s::FLAG_ACTIVE) { return; }

		_state.timestamp = now;

		if (++_state.flight_sequence == 0) { ++_state.flight_sequence; }

		_state.takeoff_utc_us = 0;
		_state.flags = rid_flight_status_s::FLAG_ACTIVE | rid_flight_status_s::FLAG_AIRBORNE;
		// Use the GNSS UTC sample's own monotonic time at the actual event only.
		// An unavailable UTC must never be reconstructed from a later clock fix.
		static constexpr uint64_t EARLIEST_UTC_US = 1577836800000000ULL;
		const int64_t sample = gps.timestamp <= uint64_t(INT64_MAX - INT32_MAX)
				       ? int64_t(gps.timestamp) + gps.timestamp_time_relative : 0;

		if (gps.fix_type >= 3 && gps.time_utc_usec >= EARLIEST_UTC_US && sample > 0
		    && recent(gps.timestamp, now, SYSTEM_RECEIPT_TIMEOUT_US)
		    && recent(uint64_t(sample), now, SYSTEM_RECEIPT_TIMEOUT_US)
		    && gps.time_utc_usec <= UINT64_MAX - (now - uint64_t(sample))) {
			_state.takeoff_utc_us = gps.time_utc_usec + now - uint64_t(sample);
			_state.flags |= rid_flight_status_s::FLAG_TAKEOFF_UTC_VALID;
		}
	}
	void land(uint64_t now)
	{
		_state.timestamp = now;
		_state.flags &= rid_flight_status_s::FLAG_TAKEOFF_UTC_VALID;
	}
	const rid_flight_status_s &state() const { return _state; }
private:
	rid_flight_status_s _state{};
};
}
