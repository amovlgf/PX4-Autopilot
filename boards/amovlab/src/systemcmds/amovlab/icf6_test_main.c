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
	icf6_print_usage();
	return 1;
	PX4_INFO("Running led test");

	if (!strcmp(argv[1], "led")) {

		PX4_INFO("Running led test");
		return icf6_led_test();

	}

	icf6_print_usage();
	return -EINVAL;
}
