#include "hw_test.h"

#include <px4_platform_common/px4_config.h>

static constexpr const char *kTestName = "SD";

extern "C" int sd_bench_main(int argc, char *argv[]);

int test_sd()
{
	char arg0[] = "sd_bench";
	char arg1[] = "-b";
	char arg2[] = "4096";
	char arg3[] = "-r";
	char arg4[] = "2";
	char arg5[] = "-d";
	char arg6[] = "1000";
	char *argv[] = {arg0, arg1, arg2, arg3, arg4, arg5, arg6, nullptr};

	const int ret = sd_bench_main(7, argv);

	if (ret == 0) {
		HW_TEST_PASS(kTestName);
		return HW_TEST_OK;
	}

	HW_TEST_FAIL(kTestName, "sd_bench returned %d", ret);
	return ret;
}
