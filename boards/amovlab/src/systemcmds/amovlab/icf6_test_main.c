#include <px4_platform_common/module.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>


#include "icf6_test.h"


__EXPORT int amovlab_main(int argc, char *argv[]);

int amovlab_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("not enough arguments");
		icf6_print_usage();
		return 1;
	}

	const char *command = argv[1];

	if (strcmp(command, "led") == 0) {
		return icf6_led_test();
	}

	icf6_print_usage();
	PX4_ERR("test error: Invalid argument");
	return -EINVAL;
}
