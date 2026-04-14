#include "hw_test.h"

#include <px4_platform_common/module.h>

extern "C" __EXPORT int hw_test_all_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	return hw_test_run_all();
}
