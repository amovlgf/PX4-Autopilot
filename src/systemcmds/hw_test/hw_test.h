#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#define HW_TEST_PASS(_name) PX4_INFO("[PASS] %s", _name)
#define HW_TEST_FAIL(_name, _fmt, ...) PX4_ERR("[FAIL] %s (" _fmt ")", _name, ##__VA_ARGS__)

static constexpr int HW_TEST_TIMEOUT_MS = 500;
static constexpr int HW_TEST_OK = 0;

/** Run a single hw_test sub-target by name (e.g. "uart", "can"). */
int hw_test_run_target(const char *target);
/** Run full factory suite (same sequence as former `hw_test all`). */
int hw_test_run_all();

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
