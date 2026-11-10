/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "RidProtocol.hpp"
#include "mavlink_main.h"
#include <lib/remote_id/RidState.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rid_identity_request.h>
#include <uORB/topics/rid_flight_status.h>
#include <uORB/topics/open_drone_id_arm_status.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <parameters/param.h>

namespace
{
pthread_mutex_t session_mutex = PTHREAD_MUTEX_INITIALIZER;
uint64_t boot_id = 0;
rid_identity_request_s owner{};
rid::Session reporting_session;

bool isTarget(uint8_t system, uint8_t component)
{
	return system == mavlink_system.sysid && component == mavlink_system.compid;
}

uint64_t bootId()
{
	if (boot_id == 0) {
		const int fd = open("/dev/urandom", O_RDONLY);

		if (fd >= 0) {
			uint64_t value = 0;
			const ssize_t count = read(fd, &value, sizeof(value));
			close(fd);

			if (count == sizeof(value)) { boot_id = value; }
		}
	}

	return boot_id;
}

void failure(Mavlink &mavlink, const mavlink_message_t &message, const mavlink_amov_rid_identity_request_t &request,
	     uint8_t result, const char *reason)
{
	mavlink_amov_rid_identity_t response{};
	response.boot_id = boot_id;
	response.session_id = request.session_id;
	response.request_id = request.request_id;
	response.protocol_version = request.protocol_version == 2 ? 2 : 1;
	response.capabilities = AMOV_UOM_CAPABILITY_LOCAL_READINESS_V2 | AMOV_UOM_CAPABILITY_FLIGHT_CONTEXT;
	response.target_system = message.sysid;
	response.target_component = message.compid;
	response.result = result;
	strncpy(response.error, reason, sizeof(response.error) - 1);
	mavlink.lock_send();
	mavlink_msg_amov_rid_identity_send_struct(mavlink.get_channel(), &response);
	mavlink.unlock_send();
}
}

void handleRidIdentityRequest(Mavlink &mavlink, const mavlink_message_t &message)
{
	mavlink_amov_rid_identity_request_t request{};
	mavlink_msg_amov_rid_identity_request_decode(&message, &request);

	if (!isTarget(request.target_system, request.target_component)) { return; }

	pthread_mutex_lock(&session_mutex);
	const uint64_t now = hrt_absolute_time();

	if ((request.protocol_version != 1 && request.protocol_version != 2) || request.session_id == 0 || request.request_id == 0
	    || (request.flags & ~1U)) {
		failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_UNSUPPORTED, "Unsupported identity request");
		pthread_mutex_unlock(&session_mutex); return;
	}

	if (bootId() == 0) {
		failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_UNSUPPORTED, "Boot entropy unavailable");
		pthread_mutex_unlock(&session_mutex); return;
	}

#if defined(CONFIG_UAVCAN_REMOTEID_CONTROLLER)
	int32_t can_enabled = 0;

	if (param_get(param_find("UAVCAN_ENABLE"), &can_enabled) != PX4_OK || can_enabled == 0) {
		failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_NOT_FOUND, "DroneCAN is disabled");
		pthread_mutex_unlock(&session_mutex); return;
	}

#else
	failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_UNSUPPORTED, "DroneCAN RID controller not built");
	pthread_mutex_unlock(&session_mutex); return;
#endif
	const bool same_owner = owner.session_id == request.session_id && owner.target_system == message.sysid
				&& owner.target_component == message.compid;

	if (same_owner && owner.protocol_version != request.protocol_version) {
		failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_CONFLICT, "Protocol change requires a new session");
		pthread_mutex_unlock(&session_mutex); return;
	}

	uORB::Subscription current_sub{ORB_ID(rid_identity)};
	rid_identity_s current{};
	const bool have_identity = current_sub.copy(&current);

	if (same_owner && have_identity && current.session_id == owner.session_id && current.request_id == 0
	    && current.result != rid_identity_s::RESULT_OK) {
		// Repeat a lost invalidation notification instead of reviving the old session.
		request.request_id = 0;
		failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_READING, "RID changed; create a new reporting session");
		pthread_mutex_unlock(&session_mutex); return;
	}

	const bool identity_valid = have_identity && current.result == rid_identity_s::RESULT_OK
				    && current.session_id == owner.session_id && rid::recent(current.timestamp, now, rid::IDENTITY_TIMEOUT_US);

	if (!reporting_session.claim(request.session_id, message.sysid, message.compid, now, identity_valid, request.protocol_version)) {
		failure(mavlink, message, request, AMOV_RID_IDENTITY_RESULT_CONFLICT, "Reporting session is owned by another GCS");
		pthread_mutex_unlock(&session_mutex); return;
	}

	if (!same_owner) {
		uORB::Publication<rid_uom_status_s> status_pub{ORB_ID(rid_uom_status)};
		status_pub.publish(rid_uom_status_s{});
	}

	{
		const bool duplicate = same_owner && owner.request_id == request.request_id;
		owner = {};
		owner.timestamp = now;
		owner.boot_id = boot_id;
		owner.session_id = request.session_id;
		owner.request_id = request.request_id;
		owner.target_system = message.sysid;
		owner.target_component = message.compid;
		owner.flags = duplicate ? 0 : request.flags;
		owner.protocol_version = request.protocol_version;
		uORB::Publication<rid_identity_request_s> pub{ORB_ID(rid_identity_request)};
		pub.publish(owner);
	}

	pthread_mutex_unlock(&session_mutex);
	mavlink.configure_stream_threadsafe("AMOV_RID_IDENTITY", 5.f);
	mavlink.configure_stream_threadsafe("AMOV_UOM_FLIGHT_STATUS", request.protocol_version == 2 ? 5.f : 0.f);
}

void handleRidUomStatus(const mavlink_message_t &message)
{
	mavlink_amov_uom_status_t input{};
	mavlink_msg_amov_uom_status_decode(&message, &input);

	if (!isTarget(input.target_system, input.target_component) || (input.protocol_version != 1 && input.protocol_version != 2)
	    || input.environment > AMOV_UOM_ENVIRONMENT_OFFICIAL || input.state > AMOV_UOM_STATE_FAILED
	    || (input.flags & ~15U) || input.local_ready > 1 || input.provider_state > AMOV_UOM_PROVIDER_STATE_FAILED) { return; }

	pthread_mutex_lock(&session_mutex);
	uORB::Subscription identity_sub{ORB_ID(rid_identity)};
	rid_identity_s identity{};
	const uint64_t now = hrt_absolute_time();
	rid_uom_status_s status{};
	status.timestamp = now;
	status.boot_id = input.boot_id;
	status.session_id = input.session_id;
	status.sequence = input.sequence;
	status.identity_revision = input.identity_revision;
	status.rid_boot_id = input.rid_boot_id;
	status.last_ack_age_ms = input.last_ack_age_ms;
	status.protocol_version = input.protocol_version;
	status.local_ready = input.local_ready;
	status.provider_state = input.provider_state;
	status.local_error = input.local_error;
	status.last_sample_age_ms = input.last_sample_age_ms;
	status.error_code = input.error_code;
	status.source_system = message.sysid;
	status.source_component = message.compid;
	status.environment = input.environment;
	status.state = input.state;
	status.flags = input.flags;
	memcpy(status.identity_hash, input.identity_hash, sizeof(status.identity_hash));

	if (input.protocol_version == owner.protocol_version && boot_id != 0 && input.boot_id == boot_id && input.session_id == owner.session_id
	    && message.sysid == owner.target_system && message.compid == owner.target_component
	    && identity_sub.copy(&identity) && identity.result == rid_identity_s::RESULT_OK
	    && rid::recent(identity.timestamp, now, rid::IDENTITY_TIMEOUT_US) && rid::matches(identity, status)
	    && reporting_session.accept(input.session_id, message.sysid, message.compid, input.sequence, now, input.protocol_version)) {
		uORB::Publication<rid_uom_status_s> pub{ORB_ID(rid_uom_status)};
		pub.publish(status);
	}

	pthread_mutex_unlock(&session_mutex);
}

bool ridSystemSourceAllowed(uint8_t system, uint8_t component)
{
	pthread_mutex_lock(&session_mutex);
	const bool allowed = reporting_session.sourceAllowed(system, component, hrt_absolute_time());
	pthread_mutex_unlock(&session_mutex);
	return allowed;
}

// Called by the stream under its own scheduling; ownership is shared across MAVLink links.
bool fillRidFlightStatus(mavlink_amov_uom_flight_status_t &out)
{
	pthread_mutex_lock(&session_mutex);
	uORB::Subscription identity_sub{ORB_ID(rid_identity)};
	rid_identity_s identity{};
	const uint64_t now = hrt_absolute_time();
	const bool valid = owner.protocol_version == 2 && boot_id != 0 && identity_sub.copy(&identity)
			   && identity.result == rid_identity_s::RESULT_OK && identity.boot_id == boot_id
			   && identity.session_id == owner.session_id && identity.protocol_version == 2
			   && identity.target_system == owner.target_system && identity.target_component == owner.target_component
			   && rid::recent(identity.timestamp, now, rid::IDENTITY_TIMEOUT_US);

	if (valid) {
		uORB::Subscription flight_sub{ORB_ID(rid_flight_status)};
		rid_flight_status_s flight{};
		flight_sub.copy(&flight);
		out.boot_id = boot_id;
		out.session_id = owner.session_id;
		out.takeoff_utc_us = flight.takeoff_utc_us;
		out.time_boot_us = now;
		out.flight_sequence = flight.flight_sequence;
		out.target_system = owner.target_system;
		out.target_component = owner.target_component;
		out.protocol_version = 2;
		out.flags = flight.flags;
		uORB::Subscription arm_sub{ORB_ID(open_drone_id_arm_status)};
		open_drone_id_arm_status_s arm{};

		if (rid::identityReady(identity, now) && arm_sub.copy(&arm) && arm.node_id == identity.rid_node_id
		    && arm.status == 0 && rid::recent(arm.timestamp, now, rid::HEARTBEAT_TIMEOUT_US)) {
			out.flags |= AMOV_UOM_FLIGHT_FLAGS_BROADCAST_READY;
		}

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
		out.flags |= AMOV_UOM_FLIGHT_FLAGS_SIMULATION;
#endif
	}

	pthread_mutex_unlock(&session_mutex);
	return valid;
}
