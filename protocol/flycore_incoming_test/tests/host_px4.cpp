/* SPDX-License-Identifier: BSD-3-Clause */
#include "host_px4.h"
#include "hw_test.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace
{
bool fail_sd{};
thread_local bool local_check{};
} // namespace

int param_find(const char *name)
{
	if (std::strstr(name, "BAUD")) {
		return 115200;
	}

	if (std::strstr(name, "COMP")) {
		return 191;
	}

	return std::strstr(name, "8550") ? 2 : 3;
}

int param_get(int parameter, void *value)
{
	*static_cast<int32_t *>(value) = parameter;
	return 0;
}

int px4_task_spawn_cmd(const char *, int, int, int, px4_main_t entry, char *const [])
{
	std::thread([entry] { entry(0, nullptr); }).detach();
	return 0;
}

int px4_task_delete(px4_task_t)
{
	return -ENOSYS;
}

extern "C" int __real_open(const char *path, int flags, ...);

extern "C" int __wrap_open(const char *path, int flags, ...)
{
	if (std::strcmp(path, "/dev/ttyS1") == 0) {
		return __real_open(std::getenv("FLYCORE_TEST_STATION"), flags);
	}

	if (std::strcmp(path, "/dev/ttyS3") == 0) {
		return __real_open(std::getenv("FLYCORE_TEST_RESPONDER"), flags);
	}

	errno = ENOENT;
	return -1;
}

int hw_test_q_uart(unsigned) { return 0; }
int hw_test_q_can(unsigned) { return 0; }
int hw_test_q_spi(unsigned) { return 0; }
int hw_test_q_i2c(unsigned) { return 0; }
int hw_test_q_sd() { return fail_sd ? -EIO : 0; }

int hw_test_q_pwm()
{
	if (local_check) {
		std::puts("LOCAL STARTED");
		return std::getchar() == 'R' ? 0 : -EIO;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	fail_sd = argc == 2 && std::strcmp(argv[1], "--fail-sd") == 0;
	std::setvbuf(stdout, nullptr, _IONBF, 0);

	if (hw_test_q_service_start() != 0) {
		return 1;
	}

	std::puts("READY");

	// A local command uses the real runner and holds its lock inside the PWM check.
	for (int command; (command = std::getchar()) != EOF;) {
		if (command == 'L') {
			local_check = true;
			std::printf("LOCAL RESULT %d\n", hw_test_run_target("pwm"));
			local_check = false;
		}
	}

	return 0;
}
