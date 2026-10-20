/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include "SimulatorSensorControl.hpp"
#include "SimulatorOdometry.hpp"

using namespace simulator_sensor_control;

TEST(SimulatorSensorControl, LastAvailableImu)
{
	EXPECT_FALSE(valid_imu_mask(0, 3));
	EXPECT_FALSE(valid_imu_mask(4, 3)); // Selecting an absent third IMU must not stop estimation.
	EXPECT_FALSE(valid_imu_mask(-1, 3));
	EXPECT_FALSE(valid_imu_mask(8, 3));
	EXPECT_TRUE(valid_imu_mask(1, 3));
	EXPECT_TRUE(valid_imu_mask(2, 3));
	EXPECT_TRUE(valid_imu_mask(3, 3));
	EXPECT_FALSE(valid_imu_mask(2, 1)); // IMU 2 must have been received before selecting it alone.
	EXPECT_TRUE(valid_imu_mask(7, 0)); // Discovery has not started yet.
	EXPECT_FALSE(valid_imu_mask(0, 0));
}

TEST(SimulatorSensorControl, IndependentGnssPositionAndHeading)
{
	EXPECT_TRUE(enabled(Gnss, 3, 1 << (Gnss - Baro)));
	EXPECT_FALSE(enabled(GnssYaw, 3, 1 << (Gnss - Baro)));
	EXPECT_FALSE(enabled(Gnss, 3, 1 << (GnssYaw - Baro)));
	EXPECT_TRUE(enabled(GnssYaw, 3, 1 << (GnssYaw - Baro)));
	EXPECT_FALSE(enabled(Imu1, 2, ALL_SENSORS));
	EXPECT_TRUE(enabled(Imu2, 2, 0));
}

TEST(SimulatorSensorControl, GnssHeadingProtocolBoundaries)
{
	EXPECT_TRUE(std::isnan(gnss_heading(0)));
	EXPECT_TRUE(std::isnan(gnss_heading(36001)));
	EXPECT_TRUE(std::isnan(gnss_heading(UINT16_MAX)));
	EXPECT_FLOAT_EQ(gnss_heading(36000), 0.f);
	EXPECT_NEAR(gnss_heading(9000), 1.5707963268f, 1e-6f);
	EXPECT_GT(gnss_heading(1), 0.f);
	EXPECT_LT(gnss_heading(35999), 6.2831853072f);
}

TEST(SimulatorOdometry, WorldAngularVelocityIsRotatedIntoBody)
{
	const matrix::Quatf attitude(matrix::Eulerf(0.4f, -0.3f, 1.5707963268f));
	const matrix::Vector3f body_rate(0.7f, -0.2f, 0.4f);
	const matrix::Vector3f local_rate = matrix::Dcmf(attitude) * body_rate;
	const matrix::Vector3f recovered = simulator_odometry::angular_velocity_to_body(attitude, local_rate);

	for (unsigned i = 0; i < 3; ++i) {
		EXPECT_NEAR(recovered(i), body_rate(i), 1.e-6f);
	}
}

TEST(SimulatorOdometry, InvalidAttitudeDoesNotInventBodyRates)
{
	const matrix::Vector3f local_rate(1.f, 2.f, 3.f);
	EXPECT_FALSE(simulator_odometry::angular_velocity_to_body(matrix::Quatf(0.f, 0.f, 0.f, 0.f), local_rate).isAllFinite());
	EXPECT_FALSE(simulator_odometry::angular_velocity_to_body(matrix::Quatf(NAN, 0.f, 0.f, 0.f), local_rate).isAllFinite());
}
