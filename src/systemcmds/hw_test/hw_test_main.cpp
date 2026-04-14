#include "hw_test.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <cstring>
#include <cstdlib>
#include <errno.h>

extern "C" __EXPORT int hw_test_main(int argc, char *argv[]);

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
PX4 hardware FAT command.

Usage:
  hw_test_all                                     — run full suite (uart can spi i2c sd pwm)
  hw_test <uart|can|spi|i2c|sd|pwm>
  hw_test um982_cfg [dev] [baud]                  — UM982 config via TELEM2 default /dev/ttyS1 @115200
  hw_test can_ext                                 — alias of 'can' (Extended ID only)
  hw_test can_tx <0|1> [duration_s] [period_ms] [loopback] — CAN bench (Extended ID only; loopback=1: no bus ACK)
  hw_test can_rx <0|1> [duration_s]              — print frames received on one port
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("hw_test", "command");
	PRINT_MODULE_USAGE_ARG("uart|can|can_ext|spi|i2c|sd|pwm|um982_cfg|can_tx|can_rx", "test target", false);
}

extern "C" __EXPORT int hw_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return -EINVAL;
	}

	if (strcmp(argv[1], "can_tx") == 0) {
		if (argc < 3) {
			PX4_ERR("usage: hw_test can_tx <0|1> [duration_s] [period_ms] [loopback]");
			PX4_ERR("  always uses Extended ID (29-bit); loopback 1 = CAN_RAW_LOOPBACK (no ACK on wire)");
			return -EINVAL;
		}

		const int bus = atoi(argv[2]);

		if (bus != 0 && bus != 1) {
			PX4_ERR("can_tx: bus must be 0 or 1");
			return -EINVAL;
		}

		unsigned duration_s = 30;
		unsigned period_ms = 100;

		if (argc > 3) {
			duration_s = static_cast<unsigned>(atoi(argv[3]));
		}

		if (argc > 4) {
			period_ms = static_cast<unsigned>(atoi(argv[4]));
		}

		bool socket_loopback = false;

		if (argc > 5) {
			socket_loopback = (atoi(argv[5]) != 0);
		}

		return test_can_tx(bus, duration_s, period_ms, socket_loopback);
	}

	if (strcmp(argv[1], "can_rx") == 0) {
		if (argc < 3) {
			PX4_ERR("usage: hw_test can_rx <0|1> [duration_s]");
			PX4_ERR("  default listen 30s; analyzer sends to the same port");
			return -EINVAL;
		}

		const int bus = atoi(argv[2]);

		if (bus != 0 && bus != 1) {
			PX4_ERR("can_rx: bus must be 0 or 1");
			return -EINVAL;
		}

		unsigned duration_s = 30;

		if (argc > 3) {
			duration_s = static_cast<unsigned>(atoi(argv[3]));
		}

		return test_can_rx(bus, duration_s);
	}

	if (strcmp(argv[1], "um982_cfg") == 0) {
		const char *dev = "/dev/ttyS1";
		unsigned baud = 115200;

		if (argc > 2) {
			dev = argv[2];
		}

		if (argc > 3) {
			baud = static_cast<unsigned>(atoi(argv[3]));
		}

		return test_um982_cfg(dev, baud);
	}

	return hw_test_run_target(argv[1]);
}
