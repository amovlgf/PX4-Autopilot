#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <stdint.h>

#define HW_TEST_PASS(_name) PX4_INFO("[PASS] %s", _name)
#define HW_TEST_FAIL(_name, _fmt, ...) PX4_ERR("[FAIL] %s (" _fmt ")", _name, ##__VA_ARGS__)

static constexpr int HW_TEST_TIMEOUT_MS = 500;
static constexpr int HW_TEST_OK = 0;

/** Run a single hw_test sub-target by name (e.g. "uart", "can"). */
int hw_test_run_target(const char *target);
/** Run full factory suite (same sequence as former `hw_test all`). */
int hw_test_run_all();

#if defined(CONFIG_BOARD_FLYCORE_Q_HW_TEST)
// The item order is part of the MAVLink result format.
enum HwTestQItem : unsigned {
	HW_TEST_Q_UART2,
	HW_TEST_Q_UART8,
	HW_TEST_Q_UART6,
	HW_TEST_Q_UART7,
	HW_TEST_Q_UART4,
	HW_TEST_Q_UART5,
	HW_TEST_Q_SPI1,
	HW_TEST_Q_SPI2,
	HW_TEST_Q_SPI4,
	HW_TEST_Q_PWM,
	HW_TEST_Q_CAN1_TO_CAN2,
	HW_TEST_Q_CAN2_TO_CAN1,
	HW_TEST_Q_I2C1,
	HW_TEST_Q_I2C4,
	HW_TEST_Q_SD,
	HW_TEST_Q_ITEM_COUNT
};

static_assert(HW_TEST_Q_ITEM_COUNT == 15, "The result protocol defines 15 hardware checks");
static constexpr uint16_t HW_TEST_Q_REQUIRED_MASK = (1u << HW_TEST_Q_ITEM_COUNT) - 1;

struct HwTestQResult {
	uint16_t executed_mask{};
	uint16_t passed_mask{};
};

int hw_test_q_run(HwTestQResult &result, void (*progress)(unsigned, void *) = nullptr, void *context = nullptr);
int hw_test_q_uart(unsigned item);
int hw_test_q_can(unsigned item);
int hw_test_q_spi(unsigned item);
int hw_test_q_i2c(unsigned item);
int hw_test_q_pwm();
int hw_test_q_sd();
int hw_test_q_service_start();
int hw_test_q_ping_6490();
#endif

int test_uart();
int test_can();
int test_can_ext();
/** Single-bus bench: \a bus 0 = can0 (FDCAN1 / CAN1 connector), 1 = can1 (FDCAN2).
 *  If \a socket_loopback, enables CAN_RAW_LOOPBACK (no bus ACK needed; stack self-test only). */
int test_can_tx(int bus, unsigned duration_s, unsigned period_ms, bool socket_loopback = false, bool ext_id = false);
int test_can_rx(int bus, unsigned duration_s);
int test_spi();
int test_i2c();
int test_sd();
int test_pwm();
int test_um982_cfg(const char *dev, unsigned baud);

/** UM982 产测默认串口固定为 TELEM2(/dev/ttyS1)。 */
static inline const char *hw_test_um982_default_device()
{
	return "/dev/ttyS1";
}
