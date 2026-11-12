/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include "../src/tx_cancellation.hpp"

using uavcan_stm32h7::tx_cancellation::cancelOnError;
using uavcan_stm32h7::tx_cancellation::cancelTimedOut;

TEST(STM32H7TxCancellation, UnusedSlotsCannotCancelAnActiveSlot)
{
	bool pending[32] = {};
	pending[0] = true;
	volatile uint32_t cancel_request = 0;

	for (uint8_t index = 1; index < 32; ++index) {
		EXPECT_FALSE(cancelTimedOut(pending[index], index, 1, cancel_request, 0, 100));
	}

	EXPECT_EQ(cancel_request, 0u);
	EXPECT_TRUE(pending[0]);
}

TEST(STM32H7TxCancellation, InactiveSlotIsIgnoredEvenWhenHardwareBitIsSet)
{
	bool pending = false;
	volatile uint32_t cancel_request = 0;
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, 0, 100));
	EXPECT_FALSE(cancelOnError(pending, 0, 1, cancel_request, true));
	EXPECT_EQ(cancel_request, 0u);
}

TEST(STM32H7TxCancellation, RequiresMatchingHardwarePendingBit)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	EXPECT_FALSE(cancelTimedOut(pending, 0, 2, cancel_request, 0, 100));
	EXPECT_FALSE(cancelOnError(pending, 0, 2, cancel_request, true));
	EXPECT_TRUE(pending);
	EXPECT_EQ(cancel_request, 0u);
}

TEST(STM32H7TxCancellation, DeadlineComparisonIsStrict)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, 100, 99));
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, 100, 100));
	EXPECT_TRUE(pending);
	EXPECT_EQ(cancel_request, 0u);
	EXPECT_TRUE(cancelTimedOut(pending, 0, 1, cancel_request, 100, 101));
	EXPECT_FALSE(pending);
	EXPECT_EQ(cancel_request, 1u);
}

TEST(STM32H7TxCancellation, DeadlineRetains64BitPrecision)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	const uint64_t deadline = uint64_t(1) << 40;
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, deadline, deadline));
	EXPECT_TRUE(cancelTimedOut(pending, 0, 1, cancel_request, deadline, deadline + 1));
}

TEST(STM32H7TxCancellation, ErrorCancellationRequiresAbortFlag)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	EXPECT_FALSE(cancelOnError(pending, 0, 1, cancel_request, false));
	EXPECT_TRUE(pending);
	EXPECT_EQ(cancel_request, 0u);
	EXPECT_TRUE(cancelOnError(pending, 0, 1, cancel_request, true));
	EXPECT_FALSE(pending);
	EXPECT_EQ(cancel_request, 1u);
}

TEST(STM32H7TxCancellation, EveryMailboxUsesItsOwnUnsignedMask)
{
	for (uint8_t index = 0; index < 32; ++index) {
		bool pending = true;
		volatile uint32_t cancel_request = 0;
		const uint32_t expected_mask = uint32_t(1) << index;
		EXPECT_TRUE(cancelTimedOut(pending, index, UINT32_MAX, cancel_request, 0, 100));
		EXPECT_EQ(cancel_request, expected_mask);
		EXPECT_FALSE(pending);
		pending = true;
		cancel_request = 0;
		EXPECT_TRUE(cancelOnError(pending, index, UINT32_MAX, cancel_request, true));
		EXPECT_EQ(cancel_request, expected_mask);
		EXPECT_FALSE(pending);
	}
}

TEST(STM32H7TxCancellation, TimeoutIsRequestedOnlyOnceWhileHardwareRemainsPending)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	EXPECT_TRUE(cancelTimedOut(pending, 0, 1, cancel_request, 0, 100));
	cancel_request = 0;
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, 0, 200));
	EXPECT_FALSE(cancelOnError(pending, 0, 1, cancel_request, true));
	EXPECT_EQ(cancel_request, 0u);
}

TEST(STM32H7TxCancellation, ErrorAbortIsRequestedOnlyOnceWhileHardwareRemainsPending)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	EXPECT_TRUE(cancelOnError(pending, 0, 1, cancel_request, true));
	cancel_request = 0;
	EXPECT_FALSE(cancelOnError(pending, 0, 1, cancel_request, true));
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, 0, 100));
	EXPECT_EQ(cancel_request, 0u);
}

TEST(STM32H7TxCancellation, ReusedSlotUsesTheNewDeadlineAndAbortFlag)
{
	bool pending = true;
	volatile uint32_t cancel_request = 0;
	ASSERT_TRUE(cancelTimedOut(pending, 0, 1, cancel_request, 0, 100));
	// Model send() tracking a new frame after the hardware makes the slot available.
	pending = true;
	cancel_request = 0;
	EXPECT_FALSE(cancelTimedOut(pending, 0, 1, cancel_request, 300, 200));
	EXPECT_FALSE(cancelOnError(pending, 0, 1, cancel_request, false));
	EXPECT_TRUE(pending);
	EXPECT_EQ(cancel_request, 0u);
	EXPECT_TRUE(cancelTimedOut(pending, 0, 1, cancel_request, 300, 301));
}

TEST(STM32H7TxCancellation, InterfaceCancellationStateIsIndependent)
{
	bool pending[2] = {true, true};
	volatile uint32_t cancel_request[2] = {0, 0};
	EXPECT_TRUE(cancelTimedOut(pending[0], 31, UINT32_MAX, cancel_request[0], 0, 100));
	EXPECT_FALSE(cancelTimedOut(pending[1], 31, UINT32_MAX, cancel_request[1], 200, 100));
	EXPECT_FALSE(pending[0]);
	EXPECT_TRUE(pending[1]);
	EXPECT_EQ(cancel_request[0], 0x80000000u);
	EXPECT_EQ(cancel_request[1], 0u);
}

TEST(STM32H7TxCancellation, MixedPendingMailboxesCancelOnlyExpiredActiveFrames)
{
	bool pending[32] = {};
	uint64_t deadlines[32] = {};
	pending[0] = pending[1] = pending[31] = true;
	deadlines[0] = 300;
	deadlines[1] = 99;
	const uint32_t hardware_pending = 0x80000083u;
	unsigned cancellation_count = 0;

	for (uint8_t index = 0; index < 32; ++index) {
		volatile uint32_t cancel_request = 0;
		const bool cancelled = cancelTimedOut(pending[index], index, hardware_pending, cancel_request, deadlines[index], 100);
		const bool expected = index == 1 || index == 31;
		EXPECT_EQ(cancelled, expected);
		EXPECT_EQ(cancel_request, expected ? uint32_t(1) << index : 0u);
		cancellation_count += cancelled;
	}

	EXPECT_EQ(cancellation_count, 2u);
	EXPECT_TRUE(pending[0]);
	EXPECT_FALSE(pending[1]);
	EXPECT_FALSE(pending[7]);
	EXPECT_FALSE(pending[31]);
}
