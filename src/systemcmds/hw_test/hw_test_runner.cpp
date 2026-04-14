#include "hw_test.h"

#include <cstring>

#include <px4_platform_common/log.h>

int hw_test_run_target(const char *target)
{
	if (strcmp(target, "uart") == 0) {
		return test_uart();
	}

	if (strcmp(target, "can") == 0) {
		return test_can();
	}

	if (strcmp(target, "can_ext") == 0) {
		return test_can_ext();
	}

	if (strcmp(target, "spi") == 0) {
		return test_spi();
	}

	if (strcmp(target, "i2c") == 0) {
		return test_i2c();
	}

	if (strcmp(target, "sd") == 0) {
		return test_sd();
	}

	if (strcmp(target, "pwm") == 0) {
		return test_pwm();
	}

	if (strcmp(target, "um982_cfg") == 0) {
		return test_um982_cfg("/dev/ttyS1", 115200);
	}

	PX4_ERR("unknown target: %s", target);
	return -EINVAL;
}

int hw_test_run_all()
{
	const char *targets[] {"uart", "can", "spi", "i2c", "sd", "pwm"};
	int ret = 0;
	constexpr size_t max_failures = sizeof(targets) / sizeof(targets[0]);
	const char *failed_targets[max_failures] {};
	size_t failed_count = 0;

	for (const char *target : targets) {
		const int sub_ret = hw_test_run_target(target);

		if (sub_ret != 0) {
			if (ret == 0) {
				ret = sub_ret;
			}

			if (failed_count < max_failures) {
				failed_targets[failed_count++] = target;
			}
		}
	}

	if (failed_count == 0) {
		PX4_INFO("");
		PX4_INFO("================================================================================");
		PX4_INFO("================================================================================");
		PX4_INFO("=======================        HW_TEST ALL: PASS        ========================");
		PX4_INFO("================================================================================");
		PX4_INFO("================================================================================");
		PX4_INFO("  uart can spi i2c sd pwm - all targets passed");
		PX4_INFO("================================================================================");
		PX4_INFO("");

	} else {
		PX4_ERR("");
		PX4_ERR("================================================================================");
		PX4_ERR("================================================================================");
		PX4_ERR("=======================        HW_TEST ALL: FAIL        ========================");
		PX4_ERR("================================================================================");
		PX4_ERR("================================================================================");
		PX4_ERR("failed targets (%u):", static_cast<unsigned>(failed_count));

		for (size_t i = 0; i < failed_count; i++) {
			PX4_ERR("  - %s", failed_targets[i]);
		}

		PX4_ERR("================================================================================");
		PX4_ERR("");
	}
	return ret;
}
