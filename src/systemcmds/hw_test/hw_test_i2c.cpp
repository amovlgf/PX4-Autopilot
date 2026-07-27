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

#endif

int test_i2c()
{
#if !defined(CONFIG_DRIVERS_IMU_BOSCH_BMI088_I2C)
	PX4_ERR("I2C FAIL %d", -ENODEV);
	return -ENODEV;
#else
	int ret = bmi088_accel_start(1);

	if (ret != 0) {
		(void)bmi088_accel_stop(1);
		PX4_ERR("I2C FAIL bus1 %d", ret);
		return ret;
	}

	px4_usleep(kPostStartSettleUs);

	(void)bmi088_accel_stop(1);
	px4_usleep(50000);

	ret = bmi088_accel_start(4);

	if (ret != 0) {
		(void)bmi088_accel_stop(4);
		PX4_ERR("I2C FAIL bus4 %d", ret);
		return ret;
	}

	px4_usleep(kPostStartSettleUs);

	(void)bmi088_accel_stop(4);
	PX4_INFO("I2C PASS");
	return 0;
#endif
}
