/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#ifndef AMOV_UOM_FLIGHT_STATUS_HPP
#define AMOV_UOM_FLIGHT_STATUS_HPP
#include "../RidProtocol.hpp"
class MavlinkStreamAmovUomFlightStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAmovUomFlightStatus(mavlink); }
	static constexpr const char *get_name_static() { return "AMOV_UOM_FLIGHT_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AMOV_UOM_FLIGHT_STATUS; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }
	unsigned get_size() override { return MAVLINK_MSG_ID_AMOV_UOM_FLIGHT_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }
private:
	explicit MavlinkStreamAmovUomFlightStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	bool send() override
	{
		mavlink_amov_uom_flight_status_t out{};

		if (!fillRidFlightStatus(out)) { return false; }

		mavlink_msg_amov_uom_flight_status_send_struct(_mavlink->get_channel(), &out);
		return true;
	}
};
#endif
