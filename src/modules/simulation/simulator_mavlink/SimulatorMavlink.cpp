/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (c) 2016 Anton Matosov. All rights reserved.
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "SimulatorMavlink.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/tasks.h>
#include <lib/geo/geo.h>
#include <drivers/device/Device.hpp>
#include <drivers/drv_pwm_output.h>
#include <conversion/rotation.h>
#include <mathlib/mathlib.h>
#include <lib/drivers/device/Device.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <termios.h>

#include <limits>
#include "SimulatorOdometry.hpp"

static int _fd;
static unsigned char _buf[2048];
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);

const unsigned mode_flag_armed = 128;
const unsigned mode_flag_custom = 1;

using namespace time_literals;
using namespace simulator_sensor_control;

static px4_task_t g_sim_task = -1;

SimulatorMavlink *SimulatorMavlink::_instance = nullptr;

static constexpr vehicle_odometry_s vehicle_odometry_empty {
	.timestamp = 0,
	.timestamp_sample = 0,
	.position = {NAN, NAN, NAN},
	.q = {NAN, NAN, NAN, NAN},
	.velocity = {NAN, NAN, NAN},
	.angular_velocity = {NAN, NAN, NAN},
	.position_variance = {NAN, NAN, NAN},
	.orientation_variance = {NAN, NAN, NAN},
	.velocity_variance = {NAN, NAN, NAN},
	.pose_frame = vehicle_odometry_s::POSE_FRAME_UNKNOWN,
	.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN,
	.reset_counter = 0,
	.quality = 0
};

SimulatorMavlink::SimulatorMavlink() :
	ModuleParams(nullptr)
{
	for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", "PWM_MAIN", "FUNC", i + 1);
		param_get(param_find(param_name), &_output_functions[i]);
	}

	_esc_status_pub.advertise();
#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	_fiducial_marker_pos_report_pub.advertise();
	_fiducial_marker_yaw_report_pub.advertise();
	_target_gnss_pub.advertise();
	_param_vte_en = param_find("VTE_EN");
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	parameters_update(true);
}

void SimulatorMavlink::parameters_update(bool force)
{
	std::lock_guard<std::mutex> lock(_sensor_mutex);

	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		validate_sensor_masks();
	}
}

void SimulatorMavlink::validate_sensor_masks()
{
	if (!valid_imu_mask(_param_sim_mav_imu.get(), _imu_present_mask)) {
		if (!valid_imu_mask(_imu_mask, _imu_present_mask)) {
			_imu_mask = _imu_present_mask ? _imu_present_mask : ALL_IMUS;
		}

		PX4_WARN("SIM_MAV_IMU=%ld invalid: keep a received IMU enabled (present 0x%x), restoring %ld",
			 (long)_param_sim_mav_imu.get(), _imu_present_mask, (long)_imu_mask);
		_param_sim_mav_imu.set(_imu_mask);
		_param_sim_mav_imu.commit();

	} else {
		if ((_imu_mask & ~_param_sim_mav_imu.get() & _imu_present_mask) != 0) {
			// The native voter needs advancing time to expire a stopped IMU. Until
			// then, its old selection may prevent new controller/actuator output.
			_imu_transition_deadline = hrt_absolute_time() + 1_s;
			_imu_recovery_notified = false;
		}

		_imu_mask = _param_sim_mav_imu.get();
	}

	if (_param_sim_mav_sens.get() < 0 || (_param_sim_mav_sens.get() & ~ALL_SENSORS) != 0) {
		PX4_WARN("SIM_MAV_SENS invalid, restoring %ld", (long)_sensor_mask);
		_param_sim_mav_sens.set(_sensor_mask);
		_param_sim_mav_sens.commit();

	} else {
		_sensor_mask = _param_sim_mav_sens.get();
	}
}

bool SimulatorMavlink::sensor_enabled(Sensor sensor) const
{
	return enabled(sensor, _imu_mask, _sensor_mask);
}

static void update_sensor_rate(hrt_abstime now, hrt_abstime &last, float &rate)
{
	if (last != 0 && now > last) {
		const float new_rate = 1.e6f / (now - last);
		rate = rate > 0.f ? 0.9f * rate + 0.1f * new_rate : new_rate;
	}

	last = now;
}

void SimulatorMavlink::sensor_received(Sensor sensor)
{
	SensorStatistics &stats = _sensor_statistics[sensor];
	stats.received++;
	update_sensor_rate(hrt_absolute_time(), stats.last_received, stats.receive_rate);
}

void SimulatorMavlink::sensor_published(Sensor sensor)
{
	SensorStatistics &stats = _sensor_statistics[sensor];
	stats.published++;
	update_sensor_rate(hrt_absolute_time(), stats.last_published, stats.publish_rate);
}

static constexpr const char *sensor_names[Count] = {
	"imu1", "imu2", "imu3", "baro", "mag", "gnss", "gnss_yaw", "range", "vision"
};

int SimulatorMavlink::sensor_command(int argc, char *argv[])
{
	parameters_update(true);
	std::lock_guard<std::mutex> lock(_sensor_mutex);

	if (argc == 1 && strcmp(argv[0], "reset") == 0) {
		_param_sim_mav_imu.set(_imu_present_mask ? _imu_present_mask : ALL_IMUS);
		_param_sim_mav_sens.set(ALL_SENSORS);
		_param_sim_mav_imu.commit();
		_param_sim_mav_sens.commit();
		validate_sensor_masks();
		PX4_INFO("All received IMUs and observations enabled");
		return PX4_OK;
	}

	if (argc != 2 || (strcmp(argv[1], "on") != 0 && strcmp(argv[1], "off") != 0)) {
		PX4_ERR("Usage: simulator_mavlink sensor {imu1|imu2|imu3|baro|mag|gnss|gnss_yaw|range|vision} {on|off}");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < Count; ++i) {
		if (strcmp(argv[0], sensor_names[i]) != 0) {
			continue;
		}

		const bool turn_on = strcmp(argv[1], "on") == 0;
		const bool imu = i < Baro;
		const int32_t bit = 1 << (imu ? i : i - Baro);
		const int32_t current = imu ? _imu_mask : _sensor_mask;
		const int32_t requested = turn_on ? current | bit : current & ~bit;

		if (imu && !valid_imu_mask(requested, _imu_present_mask)) {
			PX4_ERR("Keep at least one received IMU enabled (present 0x%x)", _imu_present_mask);
			return PX4_ERROR;
		}

		if (imu) {
			_param_sim_mav_imu.set(requested);
			_param_sim_mav_imu.commit();

		} else {
			_param_sim_mav_sens.set(requested);
			_param_sim_mav_sens.commit();
		}

		validate_sensor_masks();
		PX4_INFO("%s %s", sensor_names[i], turn_on ? "on" : "off");
		return PX4_OK;
	}

	PX4_ERR("Unknown sensor: %s", argv[0]);
	return PX4_ERROR;
}

void SimulatorMavlink::print_status()
{
	parameters_update(true);
	std::lock_guard<std::mutex> lock(_sensor_mutex);
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("running, IMU mask 0x%lx (received 0x%x), observation mask 0x%lx",
		 (unsigned long)_imu_mask, _imu_present_mask, (unsigned long)_sensor_mask);

	for (unsigned i = 0; i < Count; ++i) {
		const SensorStatistics &stats = _sensor_statistics[i];
		const bool receiving = stats.received > 0 && now - stats.last_received < 1_s;
		const bool publishing = sensor_enabled(static_cast<Sensor>(i)) && stats.published > 0
					&& now - stats.last_published < 1_s;
		PX4_INFO("%-8s %s received %s %6.1f Hz (%llu), published %s %6.1f Hz (%llu)",
			 sensor_names[i], sensor_enabled(static_cast<Sensor>(i)) ? "on " : "off",
			 receiving ? "yes" : "no ", (double)(receiving ? stats.receive_rate : 0.f),
			 (unsigned long long)stats.received,
			 publishing ? "yes" : "no ", (double)(publishing ? stats.publish_rate : 0.f),
			 (unsigned long long)stats.published);
	}
}

void SimulatorMavlink::actuator_controls_from_outputs(mavlink_hil_actuator_controls_t *msg)
{
	memset(msg, 0, sizeof(mavlink_hil_actuator_controls_t));

	msg->time_usec = hrt_absolute_time();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	if (armed) {
		for (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
			msg->controls[i] = _actuator_outputs.output[i];
		}
	}

	msg->mode = mode_flag_custom;
	msg->mode |= (armed) ? mode_flag_armed : 0;
	msg->flags = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	msg->flags |= 1;
#endif
}

void SimulatorMavlink::send_esc_telemetry(mavlink_hil_actuator_controls_t hil_act_control)
{
	esc_status_s esc_status{};
	esc_status.timestamp = hrt_absolute_time();
	const int max_esc_count = math::min(actuator_outputs_s::NUM_ACTUATOR_OUTPUTS, esc_status_s::CONNECTED_ESC_MAX);

	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	int max_esc_index = 0;

	for (int i = 0; i < max_esc_count; i++) {
		if (_output_functions[i] != 0) {
			max_esc_index = i;
		}

		esc_status.esc[i].actuator_function = _output_functions[i]; // TODO: this should be in pwm_sim...
		esc_status.esc[i].timestamp = esc_status.timestamp;
		esc_status.esc[i].esc_errorcount = 0; // TODO
		esc_status.esc[i].esc_voltage = _battery_status.voltage_v;
		esc_status.esc[i].esc_current = armed ? 1.0f + math::abs_t(hil_act_control.controls[i]) * 15.0f :
						0.0f; // TODO: magic number
		esc_status.esc[i].esc_rpm = hil_act_control.controls[i] * 6000;  // TODO: magic number
		esc_status.esc[i].esc_temperature = 20.0 + math::abs_t(hil_act_control.controls[i]) * 40.0;
	}

	esc_status.esc_count = max_esc_index + 1;
	esc_status.esc_armed_flags = (1u << esc_status.esc_count) - 1;
	esc_status.esc_online_flags = (1u << esc_status.esc_count) - 1;

	// _failure_config refreshed in updateFailureConfig(), called each loop before send_controls()
	_esc_status_pub.publish(failure_injection::process_esc(_failure_config, esc_status));
}

void SimulatorMavlink::send_controls()
{
	std::lock_guard<std::mutex> lock(_sensor_mutex);
	orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);

	if (_actuator_outputs.timestamp > 0) {
#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		// A recovery send and fresh controller output can race. jMAVSim advances
		// once per control message, so never release the same physics step twice.
		if (_last_controls_timestamp == hrt_absolute_time()) {
			return;
		}

#endif
		mavlink_hil_actuator_controls_t hil_act_control;
		actuator_controls_from_outputs(&hil_act_control);

		mavlink_message_t message{};
		mavlink_msg_hil_actuator_controls_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &hil_act_control);

		PX4_DEBUG("sending controls t=%ld (%ld)", _actuator_outputs.timestamp, hil_act_control.time_usec);

		send_mavlink_message(message);
		_last_controls_timestamp = hil_act_control.time_usec;

		send_esc_telemetry(hil_act_control);
	}
}

void SimulatorMavlink::recover_imu_transition()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	{
		std::lock_guard<std::mutex> lock(_sensor_mutex);

		if (_imu_transition_deadline == 0 || hrt_absolute_time() >= _imu_transition_deadline
		    || _actuator_outputs.timestamp == 0)
		{
			return;
		}

		if (!_imu_recovery_notified)
		{
			PX4_INFO("Holding actuator outputs while native IMU selection updates");
			_imu_recovery_notified = true;
		}
	}

	// The receive socket timed out in wall time, so the simulator is waiting for
	// controls. Hold the last command while the normal sensor voter changes IMU.
	// Only this bounded transition bypasses the normal lockstep component barrier.
	send_controls();
#endif
}

void SimulatorMavlink::update_sensors(const hrt_abstime &time, const mavlink_hil_sensor_t &sensors)
{
	const bool accel_updated = (sensors.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL;
	const bool gyro_updated = (sensors.fields_updated & SensorSource::GYRO) == SensorSource::GYRO;
	const bool imu_in_range = sensors.id < ACCEL_COUNT_MAX;
	const hrt_abstime previous_accel_timestamp = _last_accel_fifo_timestamp;
	const hrt_abstime previous_gyro_timestamp = _last_gyro_fifo_timestamp;

	if (imu_in_range && accel_updated && gyro_updated) {
		sensor_received(static_cast<Sensor>(sensors.id));
		const uint8_t present = _imu_present_mask | (1 << sensors.id);

		if (present != _imu_present_mask) {
			_imu_present_mask = present;
			validate_sensor_masks();
		}
	}

	const bool imu_enabled = imu_in_range && sensor_enabled(static_cast<Sensor>(sensors.id));

	// Track every incoming sample, even while publication is disabled. Re-enabling
	// must never integrate over the whole time the IMU was switched off.
	if (sensors.id == 0) {
		if (accel_updated) { _last_accel_fifo_timestamp = time; }

		if (gyro_updated) { _last_gyro_fifo_timestamp = time; }
	}

	// temperature only updated with baro
	if ((sensors.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		if (PX4_ISFINITE(sensors.temperature)) {
			_sensors_temperature = sensors.temperature;
		}
	}

	// accel
	if (accel_updated && (imu_enabled || !imu_in_range)) {
		if (sensors.id >= ACCEL_COUNT_MAX) {
			PX4_ERR("Number of simulated accelerometer %d out of range. Max: %d", sensors.id, ACCEL_COUNT_MAX);
			return;
		}

		if (sensors.id == 0) {
			// accel 0 is simulated FIFO
			static constexpr float ACCEL_FIFO_SCALE = CONSTANTS_ONE_G / 2048.f;
			static constexpr float ACCEL_FIFO_RANGE = 16.f * CONSTANTS_ONE_G;

			_px4_accel[sensors.id].set_scale(ACCEL_FIFO_SCALE);
			_px4_accel[sensors.id].set_range(ACCEL_FIFO_RANGE);
			_px4_accel[sensors.id].set_temperature(_sensors_temperature);

			sensor_accel_fifo_s accel_fifo{};
			accel_fifo.samples = 1;
			accel_fifo.dt = previous_accel_timestamp != 0 ? time - previous_accel_timestamp : 0;
			accel_fifo.timestamp_sample = time;
			accel_fifo.x[0] = sensors.xacc / ACCEL_FIFO_SCALE;
			accel_fifo.y[0] = sensors.yacc / ACCEL_FIFO_SCALE;
			accel_fifo.z[0] = sensors.zacc / ACCEL_FIFO_SCALE;

			_px4_accel[sensors.id].updateFIFO(accel_fifo);

		} else {
			_px4_accel[sensors.id].set_temperature(_sensors_temperature);
			_px4_accel[sensors.id].update(time, sensors.xacc, sensors.yacc, sensors.zacc);
		}
	}

	// gyro
	if (gyro_updated && (imu_enabled || !imu_in_range)) {
		if (sensors.id >= GYRO_COUNT_MAX) {
			PX4_ERR("Number of simulated gyroscope %d out of range. Max: %d", sensors.id, GYRO_COUNT_MAX);
			return;
		}

		if (sensors.id == 0) {
			// gyro 0 is simulated FIFO
			static constexpr float GYRO_FIFO_SCALE = math::radians(2000.f / 32768.f);
			static constexpr float GYRO_FIFO_RANGE = math::radians(2000.f);

			_px4_gyro[sensors.id].set_scale(GYRO_FIFO_SCALE);
			_px4_gyro[sensors.id].set_range(GYRO_FIFO_RANGE);
			_px4_gyro[sensors.id].set_temperature(_sensors_temperature);

			sensor_gyro_fifo_s gyro_fifo{};
			gyro_fifo.samples = 1;
			gyro_fifo.dt = previous_gyro_timestamp != 0 ? time - previous_gyro_timestamp : 0;
			gyro_fifo.timestamp_sample = time;
			gyro_fifo.x[0] = sensors.xgyro / GYRO_FIFO_SCALE;
			gyro_fifo.y[0] = sensors.ygyro / GYRO_FIFO_SCALE;
			gyro_fifo.z[0] = sensors.zgyro / GYRO_FIFO_SCALE;

			_px4_gyro[sensors.id].updateFIFO(gyro_fifo);

		} else {
			_px4_gyro[sensors.id].set_temperature(_sensors_temperature);
			_px4_gyro[sensors.id].update(time, sensors.xgyro, sensors.ygyro, sensors.zgyro);
		}
	}

	if (imu_enabled && accel_updated && gyro_updated
	    && _failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL, sensors.id + 1) != failure_injection::Mode::Off
	    && _failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_GYRO, sensors.id + 1) != failure_injection::Mode::Off) {
		sensor_published(static_cast<Sensor>(sensors.id));
	}

	// magnetometer
	if ((sensors.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
		if (sensors.id >= MAG_COUNT_MAX) {
			PX4_ERR("Number of simulated magnetometer %d out of range. Max: %d", sensors.id, MAG_COUNT_MAX);
			return;
		}

		sensor_received(Mag);

		if (sensor_enabled(Mag)) {
			_px4_mag[sensors.id].set_temperature(_sensors_temperature);
			_px4_mag[sensors.id].update(time, sensors.xmag, sensors.ymag, sensors.zmag);

			if (_failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, sensors.id + 1) != failure_injection::Mode::Off) {
				sensor_published(Mag);
			}
		}
	}

	// baro
	if ((sensors.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		if (sensors.id >= BARO_COUNT_MAX) {
			PX4_ERR("Number of simulated barometer %d out of range. Max: %d", sensors.id, BARO_COUNT_MAX);
			return;
		}

		sensor_received(Baro);

		if (sensor_enabled(Baro)) {
			_px4_baro[sensors.id].set_temperature(sensors.temperature);
			_px4_baro[sensors.id].update(time, sensors.abs_pressure * 100.f); // hPa to Pa

			if (_failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_BARO, sensors.id + 1) != failure_injection::Mode::Off) {
				sensor_published(Baro);
			}
		}
	}

	// differential pressure
	if ((sensors.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS && !_airspeed_disconnected) {

		const float blockage_fraction = 0.7; // defines max blockage (fully ramped)
		const float airspeed_blockage_rampup_time = 1_s; // time it takes to go max blockage, linear ramp

		float airspeed_blockage_scale = 1.f;

		if (_airspeed_blocked_timestamp > 0) {
			airspeed_blockage_scale = math::constrain(1.f - (hrt_absolute_time() - _airspeed_blocked_timestamp) /
						  airspeed_blockage_rampup_time, 1.f - blockage_fraction, 1.f);
		}

		differential_pressure_s report{};
		report.timestamp_sample = time;
		report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
		report.differential_pressure_pa = sensors.diff_pressure * 100.f * airspeed_blockage_scale; // hPa to Pa;
		report.temperature = _sensors_temperature;
		report.pitot_temperature = NAN;
		report.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(report);
	}
}

void SimulatorMavlink::handle_message(const mavlink_message_t *msg)
{
	parameters_update(false);
	std::lock_guard<std::mutex> lock(_sensor_mutex);
	updateFailureConfig();

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		handle_message_optical_flow(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		handle_message_hil_gps(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:
		handle_message_rc_channels(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR
#if defined(MAVLINK_MSG_ID_TARGET_RELATIVE)

	case MAVLINK_MSG_ID_TARGET_RELATIVE:
		handle_message_target_relative(msg);
		break;
#endif // MAVLINK_MSG_ID_TARGET_RELATIVE

#if defined(MAVLINK_MSG_ID_TARGET_ABSOLUTE)

	case MAVLINK_MSG_ID_TARGET_ABSOLUTE:
		handle_message_target_absolute(msg);
		break;
#endif // MAVLINK_MSG_ID_TARGET_ABSOLUTE
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

	case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		handle_message_hil_state_quaternion(msg);
		break;

	case MAVLINK_MSG_ID_RAW_RPM:
		mavlink_raw_rpm_t rpm_mavlink;
		mavlink_msg_raw_rpm_decode(msg, &rpm_mavlink);
		rpm_s rpm_uorb{};
		rpm_uorb.timestamp = hrt_absolute_time();
		rpm_uorb.rpm_estimate = rpm_mavlink.frequency;
		_rpm_pub.publish(rpm_uorb);
		break;
	}
}

void SimulatorMavlink::handle_message_distance_sensor(const mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist;
	mavlink_msg_distance_sensor_decode(msg, &dist);
	sensor_received(Range);

	if (sensor_enabled(Range) && publish_distance_topic(&dist) == PX4_OK) {
		sensor_published(Range);
	}
}

void SimulatorMavlink::handle_message_hil_gps(const mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	sensor_gnss_s gnss{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	device_id.devid_s.bus = 0;
	device_id.devid_s.address = hil_gps.id;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
	gnss.device_id = device_id.devid;

	gnss.latitude = hil_gps.lat / 1e7;
	gnss.longitude = hil_gps.lon / 1e7;
	gnss.altitude_msl = hil_gps.alt / 1e3;
	gnss.altitude_ellipsoid = hil_gps.alt / 1e3;

	gnss.speed_accuracy = _param_sim_gps_vel_err.get();
	gnss.course_accuracy = 0.5f;
	gnss.fix_type = hil_gps.fix_type;

	gnss.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
	gnss.epv = (float)hil_gps.epv * 1e-2f; // cm -> m

	gnss.hdop = 0; // TODO
	gnss.vdop = 0; // TODO

	gnss.noise = 0;
	gnss.automatic_gain_control = 0;
	gnss.jamming_indicator = 0;
	gnss.jamming_state = 0;
	gnss.spoofing_state = 0;

	gnss.vel_north = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
	gnss.vel_east = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
	gnss.vel_down = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
	gnss.ground_speed = (hil_gps.vel == UINT16_MAX) ? matrix::Vector2f(gnss.vel_north, gnss.vel_east).norm() :
			    (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s

	gnss.course = ((hil_gps.cog == 65535) ? NAN : matrix::wrap_2pi(math::radians(hil_gps.cog * 1e-2f))); // cdeg -> rad
	gnss.vel_ned_valid = true;

	gnss.timestamp_time_relative = 0;
	gnss.time_utc_usec = hil_gps.time_usec;

	gnss.satellites_used = hil_gps.satellites_visible;

	gnss.timestamp = hrt_absolute_time();
	sensor_received(Gnss);
	sensor_received(GnssYaw);

	// Resolve the uORB instance for this HIL_GPS id first, so the failure injection state can be
	// kept per instance. New publishers are created based on the HIL_GPS ID's being different or not.
	int instance = -1;

	for (size_t i = 0; i < sizeof(_gps_ids) / sizeof(_gps_ids[0]); i++) {
		if (_sensor_gnss_pubs[i] && _gps_ids[i] == hil_gps.id) {
			instance = i;
			break;
		}

		if (_sensor_gnss_pubs[i] == nullptr) {
			_sensor_gnss_pubs[i] = new uORB::PublicationMulti<sensor_gnss_s> {ORB_ID(sensor_gnss)};

			if (_sensor_gnss_pubs[i] == nullptr) {
				return;
			}

			_gps_ids[i] = hil_gps.id;

			instance = i;
			break;
		}
	}

	if (instance < 0) {
		return;
	}

	const bool receiver_enabled = failure_injection::process_gnss(_failure_config, (uint8_t)instance, gnss,
				      _gnss_stuck[instance]);

	if (receiver_enabled && sensor_enabled(Gnss)) {
		_sensor_gnss_pubs[instance]->publish(gnss);
		sensor_published(Gnss);
	}

	// HIL_GPS has no baseline vector or length. Leave those unknown so sensors can
	// use the configured antenna geometry without a fabricated baseline measurement.
	sensor_gnss_relative_s relative{};
	relative.timestamp = hrt_absolute_time();
	relative.device_id = device_id.devid;
	relative.time_utc_usec = hil_gps.time_usec;
	relative.heading = gnss_heading(hil_gps.yaw);
	relative.heading_accuracy = math::radians(_param_sim_gps_yaw_err.get());
	relative.position_length = NAN;
	relative.accuracy_length = NAN;

	for (unsigned i = 0; i < 3; ++i) {
		relative.position[i] = NAN;
		relative.position_accuracy[i] = NAN;
	}

	relative.gnss_fix_ok = hil_gps.fix_type >= 3;
	relative.differential_solution = hil_gps.fix_type >= 4;
	relative.carrier_solution_floating = hil_gps.fix_type == 5;
	relative.carrier_solution_fixed = hil_gps.fix_type == 6;
	relative.moving_base_mode = true;
	relative.heading_valid = PX4_ISFINITE(relative.heading) && relative.gnss_fix_ok;

	const bool heading_allowed = failure_injection::process(_failure_config, failure_injection_s::FAILURE_UNIT_SENSOR_GPS,
				     instance, relative, _gnss_relative_stuck[instance]);

	if (receiver_enabled && heading_allowed && sensor_enabled(GnssYaw)) {
		// Do not advertise an unsupported heading stream for legacy MAVLink 1 models.
		if (_sensor_gnss_relative_pubs[instance] == nullptr && hil_gps.yaw != 0) {
			_sensor_gnss_relative_pubs[instance] = new uORB::PublicationMulti<sensor_gnss_relative_s> {ORB_ID(sensor_gnss_relative)};
		}

		if (_sensor_gnss_relative_pubs[instance]) {
			_sensor_gnss_relative_pubs[instance]->publish(relative);
			sensor_published(GnssYaw);
		}
	}
}

void SimulatorMavlink::handle_message_hil_sensor(const mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	// Assume imu with id 0 is the primary imu an base lockstep based on this.
	if (imu.id == 0) {
		if (_lockstep_component == -1) {
			_lockstep_component = px4_lockstep_register_component();
		}

		struct timespec ts;

		abstime_to_ts(&ts, imu.time_usec);

		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}

	hrt_abstime now_us = hrt_absolute_time();

#if 0
	// This is just for to debug missing HIL_SENSOR messages.
	static hrt_abstime last_time = 0;
	hrt_abstime diff = now_us - last_time;
	float step = diff / 4000.0f;

	if (step > 1.1f || step < 0.9f) {
		PX4_INFO("HIL_SENSOR: imu time_usec: %lu, time_usec: %lu, diff: %lu, step: %.2f", imu.time_usec, now_us, diff, step);
	}

	last_time = now_us;
#endif

	update_sensors(now_us, imu);

	if (imu.id == 0) {
#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		if (!_has_initialized.load()) {
			_has_initialized.store(true);
		}

#endif

		px4_lockstep_progress(_lockstep_component);
	}
}

void SimulatorMavlink::handle_message_hil_state_quaternion(const mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	uint64_t timestamp = hrt_absolute_time();

	/* angular velocity */
	vehicle_angular_velocity_s hil_angular_velocity{};
	{
		hil_angular_velocity.timestamp = timestamp;

		hil_angular_velocity.xyz[0] = hil_state.rollspeed;
		hil_angular_velocity.xyz[1] = hil_state.pitchspeed;
		hil_angular_velocity.xyz[2] = hil_state.yawspeed;

		// always publish ground truth attitude message
		_vehicle_angular_velocity_ground_truth_pub.publish(hil_angular_velocity);
	}

	/* attitude */
	vehicle_attitude_s hil_attitude{};
	{
		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

		// always publish ground truth attitude message
		_attitude_ground_truth_pub.publish(hil_attitude);
	}

	/* global position */
	vehicle_global_position_s hil_gpos{};
	{
		hil_gpos.timestamp = timestamp;

		hil_gpos.lat = hil_state.lat / 1E7;//1E7
		hil_gpos.lon = hil_state.lon / 1E7;//1E7
		hil_gpos.alt = hil_state.alt / 1E3;//1E3

		// always publish ground truth attitude message
		_gpos_ground_truth_pub.publish(hil_gpos);
	}

	/* local position */
	vehicle_local_position_s hil_lpos{};
	{
		hil_lpos.timestamp = timestamp;

		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_global_local_proj_ref.isInitialized()) {
			_global_local_proj_ref.initReference(lat, lon, timestamp);
			_global_local_alt0 = hil_state.alt / 1000.f;
		}

		hil_lpos.timestamp = timestamp;
		hil_lpos.xy_valid = true;
		hil_lpos.z_valid = true;
		hil_lpos.v_xy_valid = true;
		hil_lpos.v_z_valid = true;
		_global_local_proj_ref.project(lat, lon, hil_lpos.x, hil_lpos.y);
		hil_lpos.z = _global_local_alt0 - hil_state.alt / 1000.0f;
		hil_lpos.vx = hil_state.vx / 100.0f;
		hil_lpos.vy = hil_state.vy / 100.0f;
		hil_lpos.vz = hil_state.vz / 100.0f;
		matrix::Eulerf euler = matrix::Quatf(hil_attitude.q);
		matrix::Vector3f acc(hil_state.xacc / 1000.f, hil_state.yacc / 1000.f,  hil_state.zacc / 1000.f);
		hil_lpos.ax = acc(0);
		hil_lpos.ay = acc(1);
		hil_lpos.az = acc(2);
		hil_lpos.heading = euler.psi();
		hil_lpos.xy_global = true;
		hil_lpos.z_global = true;
		hil_lpos.ref_timestamp = _global_local_proj_ref.getProjectionReferenceTimestamp();
		hil_lpos.ref_lat = _global_local_proj_ref.getProjectionReferenceLat();
		hil_lpos.ref_lon = _global_local_proj_ref.getProjectionReferenceLon();
		hil_lpos.ref_alt = _global_local_alt0;
		hil_lpos.vxy_max = std::numeric_limits<float>::infinity();
		hil_lpos.vz_max = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_min = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_max_z = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_max_xy = std::numeric_limits<float>::infinity();

		// always publish ground truth attitude message
		_lpos_ground_truth_pub.publish(hil_lpos);
	}
}

void SimulatorMavlink::handle_message_landing_target(const mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target_mavlink;
	mavlink_msg_landing_target_decode(msg, &landing_target_mavlink);

	if (landing_target_mavlink.position_valid) {
		PX4_WARN("Only landing targets relative to captured images are supported");

	} else {
		irlock_report_s report{};
		report.timestamp = hrt_absolute_time();
		report.signature = landing_target_mavlink.target_num;
		report.pos_x = landing_target_mavlink.angle_x;
		report.pos_y = landing_target_mavlink.angle_y;
		report.size_x = landing_target_mavlink.size_x;
		report.size_y = landing_target_mavlink.size_y;

		_irlock_report_pub.publish(report);
	}
}


#if defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) && CONFIG_MODULES_VISION_TARGET_ESTIMATOR

#if defined(MAVLINK_MSG_ID_TARGET_ABSOLUTE)
void
SimulatorMavlink::handle_message_target_absolute(const mavlink_message_t *msg)
{
	mavlink_target_absolute_t target_absolute;
	mavlink_msg_target_absolute_decode(msg, &target_absolute);

	target_gnss_s target_GNSS_report{};
	target_GNSS_report.timestamp = hrt_absolute_time();
	target_GNSS_report.timestamp_sample = target_absolute.timestamp;

	bool updated = false;

	// Position: bit 0
	if (!(target_absolute.sensor_capabilities & static_cast<uint8_t>(TargetAbsoluteSensorCapability::kPosition))) {
		target_GNSS_report.abs_pos_updated = false;

	} else {
		target_GNSS_report.abs_pos_updated = true;
		target_GNSS_report.latitude_deg = target_absolute.lat * 1e-7;
		target_GNSS_report.longitude_deg = target_absolute.lon * 1e-7;
		target_GNSS_report.altitude_msl_m = target_absolute.alt;
		target_GNSS_report.eph = target_absolute.position_std[0];
		target_GNSS_report.epv = target_absolute.position_std[1];
		updated = true;
	}

	// Velocity: bit 1
	if (!(target_absolute.sensor_capabilities & static_cast<uint8_t>(TargetAbsoluteSensorCapability::kVelocity))) {
		target_GNSS_report.vel_ned_updated = false;

	} else {
		const matrix::Quatf q_target(target_absolute.q_target);
		const bool target_attitude_available = (fabsf(q_target(0)) + fabsf(q_target(1)) + fabsf(q_target(2)) +
							fabsf(q_target(3))) > FLT_EPSILON;

		if (target_attitude_available) {
			const matrix::Vector3f vel_target_body(target_absolute.vel[0], target_absolute.vel[1], target_absolute.vel[2]);
			const matrix::Vector3f vel_target_ned = q_target.rotateVector(vel_target_body);

			target_GNSS_report.vel_ned_updated = true;
			target_GNSS_report.vel_n_m_s = vel_target_ned(0);
			target_GNSS_report.vel_e_m_s = vel_target_ned(1);
			target_GNSS_report.vel_d_m_s = vel_target_ned(2);
			target_GNSS_report.s_acc_m_s = fmaxf(fabsf(target_absolute.vel_std[0]),
							     fmaxf(fabsf(target_absolute.vel_std[1]),
									     fabsf(target_absolute.vel_std[2])));
			updated = true;

		} else {
			target_GNSS_report.vel_ned_updated = false;
		}
	}

	if (updated) { _target_gnss_pub.publish(target_GNSS_report); }
}
#endif // MAVLINK_MSG_ID_TARGET_ABSOLUTE


#if defined(MAVLINK_MSG_ID_TARGET_RELATIVE)
void SimulatorMavlink::handle_message_target_relative(const mavlink_message_t *msg)
{
	mavlink_target_relative_t target_relative;
	mavlink_msg_target_relative_decode(msg, &target_relative);

	vehicle_attitude_s	vehicle_attitude;
	vehicle_local_position_s vehicle_local_position;

	// Unsupported sensor frame
	if (target_relative.frame != TARGET_OBS_FRAME_LOCAL_OFFSET_NED && target_relative.frame != TARGET_OBS_FRAME_LOCAL_NED &&
	    target_relative.frame != TARGET_OBS_FRAME_BODY_FRD && target_relative.frame != TARGET_OBS_FRAME_OTHER) {
		PX4_WARN("target relative: coordinate frame %d unsupported", (int)target_relative.frame);
		return;
	}

	// Unsupported measurement type
	if (target_relative.type != LANDING_TARGET_TYPE_VISION_FIDUCIAL) {
		PX4_WARN("target relative: unsupported type %d", (int)target_relative.type);
		return;
	}

	matrix::Quatf q_sensor(target_relative.q_sensor);
	bool attitude_available = (fabsf(q_sensor(0)) + fabsf(q_sensor(1)) + fabsf(q_sensor(2)) +
				   fabsf(q_sensor(3))) > FLT_EPSILON;

	// Check if attitude can be infered based on the frame
	if (!attitude_available) {
		if (target_relative.frame == TARGET_OBS_FRAME_LOCAL_OFFSET_NED || target_relative.frame == TARGET_OBS_FRAME_LOCAL_NED) {
			// Observation already in NED, set quaternion to identity (1,0,0,0)
			q_sensor.setIdentity();
			attitude_available = true;

		} else if (target_relative.frame == TARGET_OBS_FRAME_BODY_FRD && _vehicle_attitude_sub.copy(&vehicle_attitude)) {
			// Set the quaternion to the current attitude
			const matrix::Quaternionf quat_att(&vehicle_attitude.q[0]);
			q_sensor = quat_att;
			attitude_available = true;

		} else if (target_relative.frame == TARGET_OBS_FRAME_OTHER) {
			// Target sensor frame: OTHER and no quaternion is provided
			PX4_WARN("target relative: coordinate frame %d unsupported when the q_sensor is not filled",
				 (int)target_relative.frame);
		}
	}

	// Target cannot be sent without valid attitude
	if (!attitude_available) {
		return;
	}

	// Forward target to the vision target estimator (VTE) or precland based VTE_EN
	int32_t vte_enabled = 0;

	if (_param_vte_en == PARAM_INVALID) {
		if (hrt_elapsed_time(&_vte_en_invalid_warn_last) > 20_s) {
			PX4_WARN(" Could not find VTE_EN, target_relative msg ignored.");
			_vte_en_invalid_warn_last = hrt_absolute_time();
		}

		return;

	} else {
		param_get(_param_vte_en, &vte_enabled);
	}

	if (!vte_enabled) {

		// Send the target directly to the precision landing algorithm in local NED frame
		landing_target_pose_s landing_target_pose{};
		landing_target_pose.timestamp = target_relative.timestamp;

		// Measurement already in local NED
		if (target_relative.frame == TARGET_OBS_FRAME_LOCAL_NED) {

			landing_target_pose.x_abs = target_relative.x;
			landing_target_pose.y_abs = target_relative.y;
			landing_target_pose.z_abs = target_relative.z;
			landing_target_pose.abs_pos_valid = true;

			_landing_target_pose_pub.publish(landing_target_pose);

		} else if (_vehicle_local_position_sub.copy(&vehicle_local_position)
			   && vehicle_local_position.xy_valid && vehicle_local_position.z_valid) {

			// If the local position is available, convert from sensor frame to local NED
			matrix::Vector3f target_relative_frame(target_relative.x, target_relative.y, target_relative.z);

			if (target_relative.frame == TARGET_OBS_FRAME_BODY_FRD || target_relative.frame == TARGET_OBS_FRAME_OTHER) {
				// Rotate measurement into vehicle-carried NED
				target_relative_frame = q_sensor.rotateVector(target_relative_frame);
			}

			// Convert from vehicle-carried NED to local NED
			landing_target_pose.x_abs = target_relative_frame(0) + vehicle_local_position.x;
			landing_target_pose.y_abs = target_relative_frame(1) + vehicle_local_position.y;
			landing_target_pose.z_abs = target_relative_frame(2) + vehicle_local_position.z;
			landing_target_pose.abs_pos_valid = true;

			_landing_target_pose_pub.publish(landing_target_pose);
		}

	} else {

		// Only supported type: LANDING_TARGET_TYPE_VISION_FIDUCIAL

		// Position report
		fiducial_marker_pos_report_s fiducial_marker_pos_report{};

		const hrt_abstime now = hrt_absolute_time();
		const hrt_abstime timestamp_sample = target_relative.timestamp;

		fiducial_marker_pos_report.timestamp = now;
		fiducial_marker_pos_report.timestamp_sample = timestamp_sample;
		matrix::Vector3f target_relative_frame(target_relative.x, target_relative.y, target_relative.z);

		// TARGET_OBS_FRAME_LOCAL_NED is absolute in local frame: convert to vehicle-relative.
		if (target_relative.frame == TARGET_OBS_FRAME_LOCAL_NED) {
			if (!_vehicle_local_position_sub.copy(&vehicle_local_position)
			    || !vehicle_local_position.xy_valid || !vehicle_local_position.z_valid) {
				return;
			}

			target_relative_frame(0) -= vehicle_local_position.x;
			target_relative_frame(1) -= vehicle_local_position.y;
			target_relative_frame(2) -= vehicle_local_position.z;
		}

		fiducial_marker_pos_report.rel_pos[0] = target_relative_frame(0);
		fiducial_marker_pos_report.rel_pos[1] = target_relative_frame(1);
		fiducial_marker_pos_report.rel_pos[2] = target_relative_frame(2);

		fiducial_marker_pos_report.cov_rel_pos[0] = target_relative.pos_std[0] * target_relative.pos_std[0];
		fiducial_marker_pos_report.cov_rel_pos[1] = target_relative.pos_std[1] * target_relative.pos_std[1];
		fiducial_marker_pos_report.cov_rel_pos[2] = target_relative.pos_std[2] * target_relative.pos_std[2];

		q_sensor.copyTo(fiducial_marker_pos_report.q);
		_fiducial_marker_pos_report_pub.publish(fiducial_marker_pos_report);

		const matrix::Quatf q_target(target_relative.q_target);
		const bool target_attitude_available = (fabsf(q_target(0)) + fabsf(q_target(1)) + fabsf(q_target(2)) +
							fabsf(q_target(3))) > FLT_EPSILON;

		if (target_attitude_available) {
			// Yaw report
			fiducial_marker_yaw_report_s fiducial_marker_yaw_report{};
			fiducial_marker_yaw_report.timestamp = now;
			fiducial_marker_yaw_report.timestamp_sample = timestamp_sample;

			// Transform quaternion from the target's frame to the TARGET_OBS_FRAME to the yaw relative to NED
			const matrix::Quatf q_in_ned = q_sensor * q_target;
			const float target_yaw_ned = matrix::Eulerf(q_in_ned).psi();

			fiducial_marker_yaw_report.yaw_ned = target_yaw_ned;
			fiducial_marker_yaw_report.yaw_var_ned = target_relative.yaw_std * target_relative.yaw_std;
			_fiducial_marker_yaw_report_pub.publish(fiducial_marker_yaw_report);
		}
	}

}
#endif // MAVLINK_MSG_ID_TARGET_RELATIVE
#endif // CONFIG_MODULES_VISION_TARGET_ESTIMATOR

void SimulatorMavlink::handle_message_odometry(const mavlink_message_t *msg)
{
	mavlink_odometry_t odom_in;
	mavlink_msg_odometry_decode(msg, &odom_in);

	// fill vehicle_odometry from Mavlink ODOMETRY
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = hrt_absolute_time(); // _mavlink_timesync.sync_stamp(odom_in.time_usec);

	// position x/y/z (m)
	matrix::Vector3f odom_in_p(odom_in.x, odom_in.y, odom_in.z);

	if (odom_in_p.isAllFinite()) {
		// frame_id: Coordinate frame of reference for the pose data.
		switch (odom_in.frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom.position[0] =  odom_in.y; // y: North
			odom.position[1] =  odom_in.x; // x: East
			odom.position[2] = -odom_in.z; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom.position[0] =  odom_in.x; // x: Forward
			odom.position[1] = -odom_in.y; // y: Left
			odom.position[2] = -odom_in.z; // z: Up
			break;

		default:
			break;
		}

		// pose_covariance
		//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw)
		//  first six entries are the first ROW, next five entries are the second ROW, etc.
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
				// position variances copied directly
				odom.position_variance[0] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[1] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.position_variance[0] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[1] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			default:
				break;
			}
		}
	}

	// q: the quaternion of the ODOMETRY msg represents a rotation from body frame to a local frame
	if (matrix::Quatf(odom_in.q).isAllFinite()) {
		odom.q[0] = odom_in.q[0];
		odom.q[1] = odom_in.q[1];
		odom.q[2] = odom_in.q[2];
		odom.q[3] = odom_in.q[3];

		// pose_covariance (roll, pitch, yaw)
		//  states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix pose_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			odom.orientation_variance[0] = odom_in.pose_covariance[15]; // R  row 3, col 3
			odom.orientation_variance[1] = odom_in.pose_covariance[18]; // P  row 4, col 4
			odom.orientation_variance[2] = odom_in.pose_covariance[20]; // Y  row 5, col 5
		}
	}

	// velocity vx/vy/vz (m/s)
	matrix::Vector3f odom_in_v(odom_in.vx, odom_in.vy, odom_in.vz);

	if (odom_in_v.isAllFinite()) {
		// child_frame_id: Coordinate frame of reference for the velocity in free space (twist) data.
		switch (odom_in.child_frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom.velocity[0] =  odom_in.vy; // y: North
			odom.velocity[1] =  odom_in.vx; // x: East
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom.velocity[0] =  odom_in.vx; // x: Forward
			odom.velocity[1] = -odom_in.vy; // y: Left
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		default:
			// unsupported child_frame_id
			break;
		}

		// velocity_covariance (vx, vy, vz)
		//  states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix velocity_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.child_frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
			case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_FRD:
				// velocity covariances copied directly
				odom.velocity_variance[0] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[1] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.velocity_variance[0] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[1] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			default:
				// unsupported child_frame_id
				break;
			}
		}
	}

	// ODOMETRY twist uses child_frame_id, whereas uORB angular velocity is always
	// body FRD, independently of the linear velocity reference frame.
	if (PX4_ISFINITE(odom_in.rollspeed)
	    && PX4_ISFINITE(odom_in.pitchspeed)
	    && PX4_ISFINITE(odom_in.yawspeed)) {

		const matrix::Vector3f angular_velocity(odom_in.rollspeed, odom_in.pitchspeed, odom_in.yawspeed);

		switch (odom_in.child_frame_id) {
		case MAV_FRAME_BODY_NED:
		case MAV_FRAME_BODY_OFFSET_NED:
		case MAV_FRAME_BODY_FRD:
			angular_velocity.copyTo(odom.angular_velocity);
			break;

		case MAV_FRAME_LOCAL_NED:
		case MAV_FRAME_LOCAL_FRD:

			// The attitude must refer to the same local frame as the angular rates.
			if (odom_in.frame_id == odom_in.child_frame_id) {
				simulator_odometry::angular_velocity_to_body(matrix::Quatf(odom_in.q), angular_velocity).copyTo(odom.angular_velocity);
			}

			break;

		default:
			break; // Unsupported frames remain unknown rather than being labeled body FRD.
		}
	}

	odom.reset_counter = odom_in.reset_counter;
	odom.quality = odom_in.quality;

	switch (odom_in.estimator_type) {
	case MAV_ESTIMATOR_TYPE_UNKNOWN: // accept MAV_ESTIMATOR_TYPE_UNKNOWN for legacy support
	case MAV_ESTIMATOR_TYPE_NAIVE:
	case MAV_ESTIMATOR_TYPE_VISION:
	case MAV_ESTIMATOR_TYPE_VIO:
		sensor_received(Vision);

		if (!_vio_blocked && sensor_enabled(Vision)) {
			odom.timestamp = hrt_absolute_time();
			_visual_odometry_pub.publish(odom);
			sensor_published(Vision);
		}

		break;

	case MAV_ESTIMATOR_TYPE_MOCAP:
		odom.timestamp = hrt_absolute_time();
		_mocap_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_GPS:
	case MAV_ESTIMATOR_TYPE_GPS_INS:
	case MAV_ESTIMATOR_TYPE_LIDAR:
	case MAV_ESTIMATOR_TYPE_AUTOPILOT:
	default:
		PX4_ERR("ODOMETRY: estimator_type %" PRIu8 " unsupported", odom_in.estimator_type);
		return;
	}
}

void SimulatorMavlink::handle_message_optical_flow(const mavlink_message_t *msg)
{
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = 0;
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}

void SimulatorMavlink::handle_message_rc_channels(const mavlink_message_t *msg)
{
	mavlink_rc_channels_t rc_channels;
	mavlink_msg_rc_channels_decode(msg, &rc_channels);

	input_rc_s rc_input{};
	rc_input.timestamp_last_signal = hrt_absolute_time();
	rc_input.channel_count = rc_channels.chancount;
	rc_input.rssi = rc_channels.rssi;
	rc_input.values[0] = rc_channels.chan1_raw;
	rc_input.values[1] = rc_channels.chan2_raw;
	rc_input.values[2] = rc_channels.chan3_raw;
	rc_input.values[3] = rc_channels.chan4_raw;
	rc_input.values[4] = rc_channels.chan5_raw;
	rc_input.values[5] = rc_channels.chan6_raw;
	rc_input.values[6] = rc_channels.chan7_raw;
	rc_input.values[7] = rc_channels.chan8_raw;
	rc_input.values[8] = rc_channels.chan9_raw;
	rc_input.values[9] = rc_channels.chan10_raw;
	rc_input.values[10] = rc_channels.chan11_raw;
	rc_input.values[11] = rc_channels.chan12_raw;
	rc_input.values[12] = rc_channels.chan13_raw;
	rc_input.values[13] = rc_channels.chan14_raw;
	rc_input.values[14] = rc_channels.chan15_raw;
	rc_input.values[15] = rc_channels.chan16_raw;
	rc_input.values[16] = rc_channels.chan17_raw;
	rc_input.values[17] = rc_channels.chan18_raw;

	rc_input.link_quality = -1;
	rc_input.rssi_dbm = NAN;

	rc_input.timestamp = hrt_absolute_time();

	// publish message
	_input_rc_pub.publish(rc_input);
}

void SimulatorMavlink::handle_message_vision_position_estimate(const mavlink_message_t *msg)
{
	sensor_received(Vision);

	if (!sensor_enabled(Vision) || _vio_blocked) {
		return;
	}

	mavlink_vision_position_estimate_t vpe;
	mavlink_msg_vision_position_estimate_decode(msg, &vpe);

	// fill vehicle_odometry from Mavlink VISION_POSITION_ESTIMATE
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = hrt_absolute_time(); // _mavlink_timesync.sync_stamp(vpe.usec);

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = vpe.x;
	odom.position[1] = vpe.y;
	odom.position[2] = vpe.z;

	const matrix::Quatf q(matrix::Eulerf(vpe.roll, vpe.pitch, vpe.yaw));
	q.copyTo(odom.q);

	// VISION_POSITION_ESTIMATE covariance
	//  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle
	//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = vpe.covariance[0];  // X  row 0, col 0
	odom.position_variance[1] = vpe.covariance[6];  // Y  row 1, col 1
	odom.position_variance[2] = vpe.covariance[11]; // Z  row 2, col 2

	odom.orientation_variance[0] = vpe.covariance[15]; // R  row 3, col 3
	odom.orientation_variance[1] = vpe.covariance[18]; // P  row 4, col 4
	odom.orientation_variance[2] = vpe.covariance[20]; // Y  row 5, col 5

	odom.reset_counter = vpe.reset_counter;

	odom.timestamp = hrt_absolute_time();

	_visual_odometry_pub.publish(odom);
	sensor_published(Vision);
}

void SimulatorMavlink::send_mavlink_message(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t bufLen = 0;

	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	ssize_t len;

	if (_ip == InternetProtocol::UDP) {
		len = ::sendto(_fd, buf, bufLen, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

	} else {
		len = ::send(_fd, buf, bufLen, 0);
	}

	if (len <= 0) {
		PX4_WARN("Failed sending mavlink message: %s", strerror(errno));
	}
}

void *SimulatorMavlink::sending_trampoline(void * /*unused*/)
{
	_instance->send();
	return nullptr;
}

void SimulatorMavlink::send()
{
#ifdef __PX4_DARWIN
	pthread_setname_np("sim_send");
#else
	pthread_setname_np(pthread_self(), "sim_send");
#endif

	// Subscribe to topics.
	// Only subscribe to the first actuator_outputs to fill a single HIL_ACTUATOR_CONTROLS.
	_actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs_sim), 0);

	// Before starting, we ought to send a heartbeat to initiate the SITL
	// simulator to start sending sensor data which will set the time and
	// get everything rolling.
	// Without this, we get stuck at px4_poll which waits for a time update.
	send_heartbeat();

	px4_pollfd_struct_t fds_actuator_outputs[1] = {};
	fds_actuator_outputs[0].fd = _actuator_outputs_sub;
	fds_actuator_outputs[0].events = POLLIN;

	while (true) {

		// Wait for up to 100ms for data.
		int pret = px4_poll(&fds_actuator_outputs[0], 1, 100);
		// A stopped actuator stream must not prevent sensor commands or parameter updates.
		parameters_update(false);

		if (pret == 0) {
			// Timed out, try again.
			continue;
		}

		if (pret < 0) {
			PX4_ERR("poll error %s", strerror(errno));
			continue;
		}

		if (fds_actuator_outputs[0].revents & POLLIN) {
			// Got new data to read, update all topics.
			{
				std::lock_guard<std::mutex> lock(_sensor_mutex);
				updateFailureConfig();
				_vehicle_status_sub.update(&_vehicle_status);
				_battery_status_sub.update(&_battery_status);
			}

			// Wait for other modules, such as logger or ekf2
			px4_lockstep_wait_for_components();

			send_controls();
		}
	}

	orb_unsubscribe(_actuator_outputs_sub);
}

void SimulatorMavlink::request_hil_state_quaternion()
{
	std::lock_guard<std::mutex> lock(_sensor_mutex);
	mavlink_command_long_t cmd_long = {};
	mavlink_message_t message = {};
	cmd_long.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	cmd_long.param1 = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	cmd_long.param2 = 5e3;
	mavlink_msg_command_long_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &cmd_long);
	send_mavlink_message(message);
}

void SimulatorMavlink::send_heartbeat()
{
	std::lock_guard<std::mutex> lock(_sensor_mutex);
	mavlink_heartbeat_t hb = {};
	mavlink_message_t message = {};
	hb.autopilot = 12;
	hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
	mavlink_msg_heartbeat_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &hb);
	send_mavlink_message(message);
}

void SimulatorMavlink::run()
{
#ifdef __PX4_DARWIN
	pthread_setname_np("sim_rcv");
#else
	pthread_setname_np(pthread_self(), "sim_rcv");
#endif

	struct sockaddr_in _myaddr {};
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_port);

	if (_tcp_remote_ipaddr != nullptr) {
		_myaddr.sin_addr.s_addr = inet_addr(_tcp_remote_ipaddr);

	} else if (!_hostname.empty()) {
		/* resolve hostname */
		struct hostent *host;
		host = gethostbyname(_hostname.c_str());
		memcpy(&_myaddr.sin_addr, host->h_addr_list[0], host->h_length);

		char ip[30];
		strcpy(ip, (char *)inet_ntoa((struct in_addr)_myaddr.sin_addr));
		PX4_INFO("Resolved host '%s' to address: %s", _hostname.c_str(), ip);
	}


	if (_ip == InternetProtocol::UDP) {

		_fd = socket(AF_INET, SOCK_DGRAM, 0);

		if (_fd < 0) {
			PX4_ERR("Creating UDP socket failed: %s", strerror(errno));
			return;
		}

		if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
			PX4_ERR("bind for UDP port %i failed (%i)", _port, errno);
			::close(_fd);
			return;
		}

		PX4_INFO("Waiting for simulator to connect on UDP port %u", _port);

		while (true) {
			// Once we receive something, we're most probably good and can carry on.
			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0,
					     (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				break;

			} else {
				system_usleep(500);
			}
		}

		PX4_INFO("Simulator connected on UDP port %u.", _port);

	} else {

		PX4_INFO("Waiting for simulator to accept connection on TCP port %u", _port);

		while (true) {
			_fd = socket(AF_INET, SOCK_STREAM, 0);

			if (_fd < 0) {
				PX4_ERR("Creating TCP socket failed: %s", strerror(errno));
				return;
			}

			int yes = 1;
			int ret = setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int));

			if (ret != 0) {
				PX4_ERR("setsockopt failed: %s", strerror(errno));
			}

			socklen_t myaddr_len = sizeof(_myaddr);
			ret = connect(_fd, (struct sockaddr *)&_myaddr, myaddr_len);

			if (ret == 0) {
				break;

			} else {
				::close(_fd);
				system_usleep(500);
			}
		}

		PX4_INFO("Simulator connected on TCP port %u.", _port);

	}

	// Create a thread for sending data to the simulator.
	pthread_t sender_thread;

	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(8000));

	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);

	// sender thread should run immediately after new outputs are available
	//  to send the lockstep update to the simulation
	param.sched_priority = SCHED_PRIORITY_ACTUATOR_OUTPUTS + 1;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);

	struct pollfd fds[2] = {};
	unsigned fd_count = 1;
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	// got data from simulator, now activate the sending thread
	pthread_create(&sender_thread, &sender_thread_attr, SimulatorMavlink::sending_trampoline, nullptr);
	pthread_attr_destroy(&sender_thread_attr);

	mavlink_status_t mavlink_status = {};
	unsigned consecutive_poll_timeouts = 0;
	static constexpr unsigned NETWORK_POLL_TIMEOUT_MS = 20;

	// Request HIL_STATE_QUATERNION for ground truth.
	request_hil_state_quaternion();

	while (true) {

		// wait for new mavlink messages to arrive
		int pret = ::poll(&fds[0], fd_count, NETWORK_POLL_TIMEOUT_MS);
		parameters_update(false);

		if (pret == 0) {
			// This is a real-time timeout. Simulated time may be frozen while the
			// controller waits for the native voter to expire a disabled primary IMU.
			parameters_update(true);
			recover_imu_transition();

			if (++consecutive_poll_timeouts >= 1000 / NETWORK_POLL_TIMEOUT_MS) {
				PX4_WARN("Simulator receive timeout");
				consecutive_poll_timeouts = 0;
			}

			continue;
		}

		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			consecutive_poll_timeouts = 0;

			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				mavlink_message_t msg;

				for (int i = 0; i < len; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &mavlink_status)) {
						handle_message(&msg);
					}
				}
			}
		}
	}
}

void SimulatorMavlink::updateFailureConfig()
{
	_failure_config.update();

	const failure_injection::Mode airspeed_mode = _failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_AIRSPEED, 1);
	_airspeed_disconnected = (airspeed_mode == failure_injection::Mode::Off);

	if (airspeed_mode == failure_injection::Mode::Wrong) {
		// Simulate pitot blockage: start the ramp on the rising edge and keep it running.
		if (_airspeed_blocked_timestamp == 0) {
			_airspeed_blocked_timestamp = hrt_absolute_time();
		}

	} else {
		_airspeed_blocked_timestamp = 0;
	}

	_vio_blocked = (_failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_VIO, 1) == failure_injection::Mode::Off);
}

static uint8_t distance_sensor_orientation(uint8_t mavlink_orientation)
{
	switch (mavlink_orientation) {
	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270:
		return distance_sensor_s::ROTATION_DOWNWARD_FACING;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_90:
		return distance_sensor_s::ROTATION_UPWARD_FACING;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_180:
		return distance_sensor_s::ROTATION_BACKWARD_FACING;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_NONE:
		return distance_sensor_s::ROTATION_FORWARD_FACING;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_270:
		return distance_sensor_s::ROTATION_LEFT_FACING;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_90:
		return distance_sensor_s::ROTATION_RIGHT_FACING;

	default:
		return distance_sensor_s::ROTATION_CUSTOM;
	}
}

int SimulatorMavlink::publish_distance_topic(const mavlink_distance_sensor_t *dist_mavlink)
{
	device::Device::DeviceId device_id {};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
	device_id.devid_s.address = dist_mavlink->id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;

	// Reuse the slot already claimed by this device id, otherwise claim and configure a free one.
	PX4Rangefinder *rangefinder = nullptr;

	for (size_t i = 0; i < DIST_SENSOR_COUNT_MAX; i++) {
		if (_dist_sensor_ids[i] == device_id.devid) {
			rangefinder = &_px4_rangefinder[i];
			break;
		}

		if (_dist_sensor_ids[i] == 0) {
			rangefinder = &_px4_rangefinder[i];
			rangefinder->set_device_id(device_id.devid);
			rangefinder->set_orientation(distance_sensor_orientation(dist_mavlink->orientation));
			rangefinder->set_min_distance(dist_mavlink->min_distance / 100.0f);
			rangefinder->set_max_distance(dist_mavlink->max_distance / 100.0f);
			rangefinder->set_rangefinder_type(dist_mavlink->type);
			rangefinder->set_hfov(dist_mavlink->horizontal_fov);
			rangefinder->set_vfov(dist_mavlink->vertical_fov);
			_dist_sensor_ids[i] = device_id.devid;
			break;
		}
	}

	if (rangefinder == nullptr) {
		PX4_ERR("No free simulated distance sensor slot. Max: %d", DIST_SENSOR_COUNT_MAX);
		return PX4_ERROR;
	}

	// MAVLink signal_quality: 0 means unknown (-1 in uORB) and 1..100 remaps to 0..100.
	const int8_t signal_quality = dist_mavlink->signal_quality == 0 ? -1 : 100 * (dist_mavlink->signal_quality - 1) / 99;
	rangefinder->set_variance(dist_mavlink->covariance * 1e-4f); // cm^2 to m^2
	// update() applies distance-sensor failure injection internally before publishing.
	rangefinder->update(hrt_absolute_time(), dist_mavlink->current_distance * 0.01f, signal_quality,
			    dist_mavlink->quaternion);

	return _failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR,
				    rangefinder->get_instance() + 1) == failure_injection::Mode::Off ? PX4_ERROR : PX4_OK;
}

int SimulatorMavlink::start(int argc, char *argv[])
{
	_instance = new SimulatorMavlink();

	if (_instance) {

		if (argc == 5 && strcmp(argv[3], "-u") == 0) {
			_instance->set_ip(InternetProtocol::UDP);
			_instance->set_port(atoi(argv[4]));
		}

		if (argc == 5 && strcmp(argv[3], "-c") == 0) {
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_port(atoi(argv[4]));
		}

		if (argc == 6 && strcmp(argv[3], "-t") == 0) {
			PX4_INFO("using TCP on remote host %s port %s", argv[4], argv[5]);
			PX4_WARN("Please ensure port %s is not blocked by a firewall.", argv[5]);
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_tcp_remote_ipaddr(argv[4]);
			_instance->set_port(atoi(argv[5]));
		}

		if (argc == 6 && strcmp(argv[3], "-h") == 0) {
			PX4_INFO("using TCP on remote host %s port %s", argv[4], argv[5]);
			PX4_WARN("Please ensure port %s is not blocked by a firewall.", argv[5]);
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_hostname(argv[4]);
			_instance->set_port(atoi(argv[5]));
		}

		_instance->run();

		return 0;

	} else {
		PX4_WARN("creation failed");
		return 1;
	}
}

static void usage()
{
	PX4_INFO("Usage: simulator_mavlink {start -[spt] [-u udp_port / -c tcp_port] |stop|status}");
	PX4_INFO("Start simulator:     simulator_mavlink start");
	PX4_INFO("Connect using UDP: simulator_mavlink start -u udp_port");
	PX4_INFO("Connect using TCP: simulator_mavlink start -c tcp_port");
	PX4_INFO("Connect to a remote server using TCP: simulator_mavlink start -t ip_addr tcp_port");
	PX4_INFO("Connect to a remote server via hostname using TCP: simulator_mavlink start -h hostname tcp_port");
	PX4_INFO("Control observations: simulator_mavlink sensor {imu1|imu2|imu3|baro|mag|gnss|gnss_yaw|range|vision} {on|off}");
	PX4_INFO("Restore observations: simulator_mavlink sensor reset");
}

__BEGIN_DECLS
extern int simulator_mavlink_main(int argc, char *argv[]);
__END_DECLS

int simulator_mavlink_main(int argc, char *argv[])
{
	if (argc > 1 && strcmp(argv[1], "start") == 0) {

		if (g_sim_task >= 0) {
			PX4_WARN("Simulator already started");
			return 0;
		}

		g_sim_task = px4_task_spawn_cmd("simulator_mavlink",
						SCHED_DEFAULT,
						SCHED_PRIORITY_MAX,
						1500,
						SimulatorMavlink::start,
						argv);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		// We want to prevent the rest of the startup script from running until time
		// is initialized by the HIL_SENSOR messages from the simulator.
		while (true) {
			if (SimulatorMavlink::getInstance() && SimulatorMavlink::getInstance()->has_initialized()) {
				break;
			}

			system_usleep(100);
		}

#endif

	} else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
			return 1;

		} else {
			px4_task_delete(g_sim_task);
			px4_task_join(g_sim_task);
			g_sim_task = -1;
		}

	} else if (argc > 2 && strcmp(argv[1], "sensor") == 0) {
		if (g_sim_task < 0 || SimulatorMavlink::getInstance() == nullptr) {
			PX4_WARN("Simulator not running");
			return 1;
		}

		return SimulatorMavlink::getInstance()->sensor_command(argc - 2, argv + 2);

	} else if (argc == 2 && strcmp(argv[1], "status") == 0) {
		if (g_sim_task < 0 || SimulatorMavlink::getInstance() == nullptr) {
			PX4_WARN("Simulator not running");
			return 1;

		} else {
			SimulatorMavlink::getInstance()->print_status();
		}

	} else {
		usage();
		return 1;
	}

	return 0;
}
