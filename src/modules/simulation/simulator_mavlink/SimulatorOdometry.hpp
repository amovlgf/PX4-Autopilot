/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include <matrix/math.hpp>

namespace simulator_odometry
{

inline matrix::Vector3f angular_velocity_to_body(const matrix::Quatf &body_to_local,
		const matrix::Vector3f &angular_velocity_local)
{
	static constexpr float QUATERNION_NORM_TOLERANCE = 1.e-3f;

	if (!body_to_local.isAllFinite() || !angular_velocity_local.isAllFinite()
	    || fabsf(body_to_local.norm() - 1.f) > QUATERNION_NORM_TOLERANCE) {
		return matrix::Vector3f{NAN, NAN, NAN};
	}

	matrix::Quatf normalized(body_to_local);
	normalized.normalize();
	return matrix::Dcmf(normalized).transpose() * angular_velocity_local;
}

} // namespace simulator_odometry
