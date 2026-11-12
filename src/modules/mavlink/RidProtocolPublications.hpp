/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <uORB/Publication.hpp>
#include <uORB/topics/rid_identity_request.h>
#include <uORB/topics/rid_uom_status.h>

class RidProtocolPublications
{
public:
	bool publishIdentityRequest(const rid_identity_request_s &request)
	{
		return _identity_request_pub.publish(request);
	}

	bool publishUomStatus(const rid_uom_status_s &status)
	{
		return _uom_status_pub.publish(status);
	}
private:
	uORB::Publication<rid_identity_request_s> _identity_request_pub{ORB_ID(rid_identity_request)};
	uORB::Publication<rid_uom_status_s> _uom_status_pub{ORB_ID(rid_uom_status)};
};
