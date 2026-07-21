#include "hw_test.h"

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_accel.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>

#include <cstdio>
#include <errno.h>
#include <inttypes.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>

static constexpr unsigned kI2cSensorObserveMs = 2000;
static constexpr unsigned kI2cFreshWindowUs = 200000;
static constexpr unsigned kMaxAccelSens = 4;
static constexpr const char *kTestName = "I2C";

#if defined(CONFIG_DRIVERS_IMU_BOSCH_BMI088_I2C)

extern "C" int bmi088_i2c_main(int argc, char *argv[]);

static int bmi088_i2c_invoke(int argc, char **argv)
{
	return bmi088_i2c_main(argc, argv);
}

static void bus_num_to_str(int bus, char *out, size_t out_sz)
{
	(void)snprintf(out, out_sz, "%d", bus);
}

static int bmi088_accel_start(int bus)
{
	char a0[] = "bmi088_i2c";
	char a1[] = "-A";
	char a2[] = "-X";
	char a3[] = "-b";
	char bstr[8] {};
	bus_num_to_str(bus, bstr, sizeof(bstr));
	char a5[] = "-R";
	char a6[] = "4";
	char a7[] = "start";
	char *argv[] = {a0, a1, a2, a3, bstr, a5, a6, a7, nullptr};
	return bmi088_i2c_invoke(8, argv);
}

static int bmi088_accel_stop(int bus)
{
	char a0[] = "bmi088_i2c";
	char a1[] = "-A";
	char a2[] = "-X";
	char a3[] = "-b";
	char bstr[8] {};
	bus_num_to_str(bus, bstr, sizeof(bstr));
	char a5[] = "stop";
	char *argv[] = {a0, a1, a2, a3, bstr, a5, nullptr};
	return bmi088_i2c_invoke(6, argv);
}

struct I2cAccelSeen {
	bool found{false};
	hrt_abstime last_timestamp{0};
	bool error_count_initialized{false};
	uint32_t error_count_first{0};
	uint32_t error_count_last{0};
};

static bool is_bmi088_i2c_accel_on_bus(uint32_t device_id, int bus)
{
	device::Device::DeviceId devid{};
	devid.devid = device_id;

	return (devid.devid_s.bus_type == device::Device::DeviceBusType_I2C)
	       && (devid.devid_s.bus == bus)
	       && (devid.devid_s.devtype == DRV_ACC_DEVTYPE_BMI088);
}

static I2cAccelSeen observe_bmi088_i2c_accel(int bus)
{
	uORB::SubscriptionMultiArray<sensor_accel_s, kMaxAccelSens> accel_subs{ORB_ID::sensor_accel};
	const hrt_abstime start = hrt_absolute_time();
	I2cAccelSeen seen{};

	while ((hrt_elapsed_time(&start) / 1000) < kI2cSensorObserveMs) {
		for (unsigned i = 0; i < kMaxAccelSens; i++) {
			sensor_accel_s msg {};

			while (accel_subs[i].update(&msg)) {
				if (is_bmi088_i2c_accel_on_bus(msg.device_id, bus)) {
					seen.found = true;
					seen.last_timestamp = msg.timestamp;
					seen.error_count_last = msg.error_count;

					if (!seen.error_count_initialized) {
						seen.error_count_first = msg.error_count;
						seen.error_count_initialized = true;
					}
				}
			}
		}

		px4_usleep(10000);
	}

	return seen;
}

static int test_bmi088_i2c_accel_bus(int bus)
{
	const int start_ret = bmi088_accel_start(bus);

	if (start_ret != 0) {
		(void)bmi088_accel_stop(bus);
		HW_TEST_FAIL(kTestName, "BMI088 accel start failed on bus%d: %d", bus, start_ret);
		return start_ret;
	}

	const I2cAccelSeen seen = observe_bmi088_i2c_accel(bus);
	(void)bmi088_accel_stop(bus);

	if (!seen.found) {
		HW_TEST_FAIL(kTestName, "BMI088 accel missing on bus%d", bus);
		return -ENODEV;
	}

	const bool fresh = (seen.last_timestamp != 0)
			   && ((hrt_absolute_time() - seen.last_timestamp) <= kI2cFreshWindowUs);

	if (!fresh) {
		HW_TEST_FAIL(kTestName, "BMI088 accel stream stale on bus%d (<=%u ms required)", bus,
			     (unsigned)(kI2cFreshWindowUs / 1000));
		return -ETIMEDOUT;
	}

	if (!seen.error_count_initialized) {
		HW_TEST_FAIL(kTestName, "BMI088 accel error_count baseline missing on bus%d", bus);
		return -ENODATA;
	}

	const uint32_t error_count_delta = seen.error_count_last - seen.error_count_first;
	PX4_INFO("I2C bus%d BMI err_count:%" PRIu32 "->%" PRIu32 " d=%" PRIu32,
		 bus, seen.error_count_first, seen.error_count_last, error_count_delta);

	if (error_count_delta > 0) {
		HW_TEST_FAIL(kTestName, "BMI088 accel error_count increased on bus%d: %" PRIu32,
			     bus, error_count_delta);
		return -EIO;
	}

	return PX4_OK;
}

#endif

int test_i2c()
{
#if !defined(CONFIG_DRIVERS_IMU_BOSCH_BMI088_I2C)
	HW_TEST_FAIL(kTestName, "BMI088 I2C driver unavailable: %d", -ENODEV);
	return -ENODEV;
#else
	int ret = test_bmi088_i2c_accel_bus(1);

	if (ret != PX4_OK) {
		return ret;
	}

	px4_usleep(50000);

	ret = test_bmi088_i2c_accel_bus(4);

	if (ret != PX4_OK) {
		return ret;
	}

	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
#endif
}
