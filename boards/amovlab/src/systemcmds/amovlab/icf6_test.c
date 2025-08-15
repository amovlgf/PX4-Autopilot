#include <px4_platform_common/module.h>


#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "icf6_test.h"

void icf6_print_usage(void)
{
	PRINT_MODULE_USAGE_NAME_SIMPLE("icf6", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "LED Test");
}

int icf6_led_test(void)
{
	PX4_INFO("Running led test");

	stm32_configgpio(GPIO_nLED_RED);
	stm32_configgpio(GPIO_nLED_BLUE);

	int i = 0;

	for (i = 0; i < 2; i++) {
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_RED, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_RED, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_BLUE, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_BLUE, true);
	}

	return OK;
}
