/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>

namespace simulator_sensor_control
{

static constexpr int32_t ALL_IMUS = 7;
static constexpr int32_t ALL_SENSORS = 63;

enum Sensor : unsigned {
	Imu1,
	Imu2,
	Imu3,
	Baro,
	Mag,
	Gnss,
	GnssYaw,
	Range,
	Vision,
	Count
};

inline bool valid_imu_mask(int32_t requested, uint8_t available)
{
	return requested > 0 && (requested & ~ALL_IMUS) == 0 && (available == 0 || (requested & available) != 0);
}

inline bool enabled(Sensor sensor, int32_t imu_mask, int32_t sensor_mask)
{
	return sensor < Baro ? (imu_mask & (1 << sensor)) != 0 : (sensor_mask & (1 << (sensor - Baro))) != 0;
}

// HIL_GPS uses 0 for unavailable and 36000 for north. Other values above 36000 are invalid.
inline float gnss_heading(uint16_t centidegrees)
{
	static constexpr float CENTIDEGREES_TO_RADIANS = 3.14159265358979323846f / 18000.f;
	return centidegrees == 0 || centidegrees > 36000 ? NAN :
	       (centidegrees == 36000 ? 0.f : centidegrees * CENTIDEGREES_TO_RADIANS);
}

} // namespace simulator_sensor_control
