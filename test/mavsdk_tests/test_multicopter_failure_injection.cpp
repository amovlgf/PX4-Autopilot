/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <thread>
#include <chrono>

#include "autopilot_tester_failure.h"

TEST_CASE("Failure Injection - Reject mid-air when it is disabled", "[multicopter]")
{
	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_param_int("SYS_FAILURE_EN", 0);
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, 1,
			      mavsdk::Failure::Result::Disabled);
	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

TEST_CASE("Failure Injection - Quad single motor failure lands", "[multicopter]")
{
	const float flight_altitude = 2.5f;
	const float hover_speed_tolerance = 1.0f;

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	tester.set_param_sys_failure_en(true);
	tester.set_param_fd_act_en(false);
	tester.set_param_mc_airmode(1);
	tester.set_param_ca_failure_mode(1);
	tester.set_param_com_act_fail_act(2);
	tester.set_param_com_fail_act_t(5.f);
	tester.set_takeoff_altitude(flight_altitude);
	tester.enable_actuator_output_status();
	tester.sleep_for(std::chrono::seconds(1));

	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	const int motor_instance = 1;
	const unsigned num_motors = 4;
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, motor_instance,
			      mavsdk::Failure::Result::Success);
	tester.sleep_for(std::chrono::seconds(1));
	tester.ensure_motor_stopped(motor_instance - 1, num_motors);

	tester.wait_until_landing(std::chrono::seconds(3));
	tester.wait_until_on_ground(std::chrono::seconds(60));
	tester.wait_until_disarmed(std::chrono::seconds(30));
}
