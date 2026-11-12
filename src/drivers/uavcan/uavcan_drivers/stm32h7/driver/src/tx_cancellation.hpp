/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <stdint.h>

namespace uavcan_stm32h7
{
namespace tx_cancellation
{

/** Caller must serialize access to the software pending flag and the controller. */
inline bool cancelPending(bool &pending, uint8_t index, uint32_t hardware_pending,
			  volatile uint32_t &cancel_request, bool requested)
{
	const uint32_t mask = uint32_t(1) << index;

	if (pending && (hardware_pending & mask) != 0 && requested) {
		cancel_request = mask;
		// Stop software tracking once requested; TXFQS still controls hardware slot reuse.
		pending = false;
		return true;
	}

	return false;
}

inline bool cancelTimedOut(bool &pending, uint8_t index, uint32_t hardware_pending,
			   volatile uint32_t &cancel_request, uint64_t deadline_usec, uint64_t current_time_usec)
{
	return cancelPending(pending, index, hardware_pending, cancel_request, deadline_usec < current_time_usec);
}

inline bool cancelOnError(bool &pending, uint8_t index, uint32_t hardware_pending,
			  volatile uint32_t &cancel_request, bool abort_on_error)
{
	return cancelPending(pending, index, hardware_pending, cancel_request, abort_on_error);
}

} // namespace tx_cancellation
} // namespace uavcan_stm32h7
