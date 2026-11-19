#include "hw_test.h"

#include <cstdio>
#include <errno.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>

/* After `bmi088_i2c -A start` returns OK, the driver has already probed the chip.
 * Allow the FIFO/work queue to run before stop/switch (uORB matching is unreliable here). */
static constexpr unsigned kPostStartSettleUs = 400000;

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

static int test_i2c_bus(int bus, bool verify_stop = false)
{
	const int ret = bmi088_accel_start(bus);

	if (ret != 0) {
		(void)bmi088_accel_stop(bus);
		PX4_ERR("I2C FAIL bus%d %d", bus, ret);
		return ret;
	}

	px4_usleep(kPostStartSettleUs);
	const int stop_ret = bmi088_accel_stop(bus);

	if (verify_stop && stop_ret != 0) {
		PX4_ERR("I2C FAIL bus%d: BMI088 stop returned %d", bus, stop_ret);
		return stop_ret;
	}

	return 0;
}
#endif

int test_i2c()
{
#if !defined(CONFIG_DRIVERS_IMU_BOSCH_BMI088_I2C)
	PX4_ERR("I2C FAIL %d", -ENODEV);
	return -ENODEV;
#else
	int ret = test_i2c_bus(1);

	if (ret != 0) {
		return ret;
	}

	px4_usleep(50000);
	ret = test_i2c_bus(4);

	if (ret == 0) {
		PX4_INFO("I2C PASS");
	}

	return ret;
#endif
}

#if defined(CONFIG_BOARD_FLYCORE_Q_HW_TEST)
int hw_test_q_i2c(unsigned item)
{
#if defined(CONFIG_DRIVERS_IMU_BOSCH_BMI088_I2C)

	if (item != HW_TEST_Q_I2C1 && item != HW_TEST_Q_I2C4) {
		return -EINVAL;
	}

	const int ret = test_i2c_bus(item == HW_TEST_Q_I2C1 ? 1 : 4, true);

	// Keep the bus-switch settle delay used by the ICF6 fixture.
	px4_usleep(50000);
	return ret;
#else
	(void)item;
	return -ENODEV;
#endif
}
#endif
