#include "hw_test.h"

#include <cerrno>
#include <cstring>

#include <px4_platform_common/log.h>

#if defined(CONFIG_BOARD_FLYCORE_Q_HW_TEST)

#include <pthread.h>

static pthread_mutex_t test_mutex = PTHREAD_MUTEX_INITIALIZER;

static constexpr uint16_t item_bit(unsigned item)
{
	return static_cast<uint16_t>(1u << item);
}

static int run_q_item(unsigned item)
{
	switch (item) {
	case HW_TEST_Q_UART2:
	case HW_TEST_Q_UART8:
	case HW_TEST_Q_UART6:
	case HW_TEST_Q_UART7:
	case HW_TEST_Q_UART4:
		return hw_test_q_uart(item);

	case HW_TEST_Q_UART5:
		return hw_test_q_ping_6490();

	case HW_TEST_Q_SPI1:
	case HW_TEST_Q_SPI2:
	case HW_TEST_Q_SPI4:
		return hw_test_q_spi(item);

	case HW_TEST_Q_PWM:
		return hw_test_q_pwm();

	case HW_TEST_Q_CAN1_TO_CAN2:
	case HW_TEST_Q_CAN2_TO_CAN1:
		return hw_test_q_can(item);

	case HW_TEST_Q_I2C1:
	case HW_TEST_Q_I2C4:
		return hw_test_q_i2c(item);

	case HW_TEST_Q_SD:
		return hw_test_q_sd();

	default:
		return -EINVAL;
	}
}

static int run_q_items(uint16_t selected, HwTestQResult &result, void (*progress)(unsigned, void *), void *context)
{
	result = {};
	const int lock_result = pthread_mutex_trylock(&test_mutex);

	if (lock_result != 0) {
		return -lock_result;
	}

	static constexpr const char *item_names[] {
		"UART2", "UART8", "UART6", "UART7", "UART4 RTK", "UART5 6490",
		"SPI1 BMI088", "SPI2 ICM42688P", "SPI4 FRAM", "PWM 1-8", "CAN1 to CAN2", "CAN2 to CAN1",
		"I2C1", "I2C4", "SD card"
	};
	static_assert(sizeof(item_names) / sizeof(item_names[0]) == HW_TEST_Q_ITEM_COUNT, "Missing item name");
	int ret = 0;

	for (unsigned item = 0; item < HW_TEST_Q_ITEM_COUNT; ++item) {
		const uint16_t bit = item_bit(item);

		if (!(selected & bit)) {
			continue;
		}

		if (progress) {
			progress(item, context);
		}

		const int item_result = run_q_item(item);
		result.executed_mask |= bit;

		if (item_result == 0) {
			result.passed_mask |= bit;
			HW_TEST_PASS(item_names[item]);

		} else {
			HW_TEST_FAIL(item_names[item], "error %d", item_result);

			if (ret == 0) {
				ret = item_result;
			}
		}
	}

	pthread_mutex_unlock(&test_mutex);
	return ret;
}

int hw_test_q_run(HwTestQResult &result, void (*progress)(unsigned, void *), void *context)
{
	return run_q_items(HW_TEST_Q_REQUIRED_MASK, result, progress, context);
}

int hw_test_run_all()
{
	HwTestQResult result{};
	const int ret = hw_test_q_run(result);
	PX4_INFO("Flycore-Q %s, executed 0x%04x, passed 0x%04x",
		 ret == 0 ? "PASS" : "FAIL", static_cast<unsigned>(result.executed_mask),
		 static_cast<unsigned>(result.passed_mask));
	return ret;
}

int hw_test_run_target(const char *target)
{
	static constexpr struct {
		const char *name;
		uint16_t items;
	} targets[] {
		{"uart", (1u << (HW_TEST_Q_UART5 + 1)) - 1},
		{"spi", item_bit(HW_TEST_Q_SPI1) | item_bit(HW_TEST_Q_SPI2) | item_bit(HW_TEST_Q_SPI4)},
		{"pwm", item_bit(HW_TEST_Q_PWM)},
		{"can", item_bit(HW_TEST_Q_CAN1_TO_CAN2) | item_bit(HW_TEST_Q_CAN2_TO_CAN1)},
		{"can_ext", item_bit(HW_TEST_Q_CAN1_TO_CAN2) | item_bit(HW_TEST_Q_CAN2_TO_CAN1)},
		{"i2c", item_bit(HW_TEST_Q_I2C1) | item_bit(HW_TEST_Q_I2C4)},
		{"sd", item_bit(HW_TEST_Q_SD)}
	};

	for (const auto &entry : targets) {
		if (strcmp(target, entry.name) == 0) {
			HwTestQResult result{};
			return run_q_items(entry.items, result, nullptr, nullptr);
		}
	}

	PX4_ERR("unknown target: %s", target);
	return -EINVAL;
}

#else

static constexpr const char *kAnsiReset = "\x1b[0m";
static constexpr const char *kAnsiBoldGreen = "\x1b[1;32m";
static constexpr const char *kAnsiBoldRed = "\x1b[1;31m";

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
		return test_um982_cfg(hw_test_um982_default_device(), 115200);
	}

	PX4_ERR("unknown target: %s", target);
	return -EINVAL;
}

int hw_test_run_all()
{
	const char *targets[] {"um982_cfg", "can", "spi", "i2c", "sd", "pwm", "uart"};
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
		PX4_INFO("%s================================================================================%s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("%s================================================================================%s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("%s=======================        HW_TEST ALL: PASS        ========================%s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("%s  ______     __         __            ______     __  __    %s", kAnsiBoldGreen, kAnsiReset);
		PX4_INFO("%s /\\  __ \\   /\\ \\       /\\ \\          /\\  __ \\   /\\ \\/ /    %s", kAnsiBoldGreen, kAnsiReset);
		PX4_INFO("%s \\ \\  __ \\  \\ \\ \\____  \\ \\ \\____     \\ \\ \\/\\ \\  \\ \\  _\"-.  %s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("%s  \\ \\_\\ \\_\\  \\ \\_____\\  \\ \\_____\\     \\ \\_____\\  \\ \\_\\ \\_\\ %s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("%s   \\/_/\\/_/   \\/_____/   \\/_____/      \\/_____/   \\/_/\\/_/ %s", kAnsiBoldGreen, kAnsiReset);
		PX4_INFO("%s================================================================================%s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("%s================================================================================%s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("  um982_cfg can spi i2c sd pwm uart - all targets passed");
		PX4_INFO("%s================================================================================%s", kAnsiBoldGreen,
			 kAnsiReset);
		PX4_INFO("");

	} else {
		PX4_ERR("");
		PX4_ERR("%s================================================================================%s", kAnsiBoldRed,
			kAnsiReset);
		PX4_ERR("%s================================================================================%s", kAnsiBoldRed,
			kAnsiReset);
		PX4_ERR("%s=======================        HW_TEST ALL: FAIL        ========================%s", kAnsiBoldRed,
			kAnsiReset);
		PX4_ERR("%s  ______   ______     __     __ %s", kAnsiBoldRed, kAnsiReset);
		PX4_ERR("%s /\\  ___\\ /\\  __ \\   /\\ \\   /\\ \\%s", kAnsiBoldRed, kAnsiReset);
		PX4_ERR("%s \\ \\  __\\ \\ \\  __ \\  \\ \\ \\  \\ \\ \\__%s", kAnsiBoldRed, kAnsiReset);
		PX4_ERR("%s  \\ \\_\\    \\ \\_\\ \\_\\  \\ \\_\\  \\ \\_____%s", kAnsiBoldRed, kAnsiReset);
		PX4_ERR("%s   \\/_/     \\/_/\\/_/   \\/_/   \\/____/%s", kAnsiBoldRed, kAnsiReset);
		PX4_ERR("%s================================================================================%s", kAnsiBoldRed,
			kAnsiReset);
		PX4_ERR("%s================================================================================%s", kAnsiBoldRed,
			kAnsiReset);
		PX4_ERR("failed targets (%u):", static_cast<unsigned>(failed_count));

		for (size_t i = 0; i < failed_count; i++) {
			PX4_ERR("  - %s", failed_targets[i]);
		}

		PX4_ERR("%s================================================================================%s", kAnsiBoldRed,
			kAnsiReset);
		PX4_ERR("");
	}

	return ret;
}

#endif // CONFIG_BOARD_FLYCORE_Q_HW_TEST
