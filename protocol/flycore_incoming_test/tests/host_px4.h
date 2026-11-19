/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <semaphore.h>
#include <unistd.h>

#define CONFIG_BOARD_FLYCORE_Q_HW_TEST 1
#define PX4_INFO(...) do { std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); } while (0)
#define PX4_WARN(...) PX4_INFO(__VA_ARGS__)
#define PX4_ERR(...) PX4_INFO(__VA_ARGS__)
#define px4_usleep usleep

using hrt_abstime = uint64_t;

inline hrt_abstime hrt_absolute_time()
{
	using namespace std::chrono;
	return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then)
{
	return hrt_absolute_time() - *then;
}

int param_find(const char *name);
int param_get(int parameter, void *value);

using px4_sem_t = sem_t;
#define px4_sem_init sem_init
#define px4_sem_destroy sem_destroy
#define px4_sem_wait sem_wait
#define px4_sem_post sem_post

using px4_task_t = int;
using px4_main_t = int (*)(int, char *[]);
constexpr int SCHED_DEFAULT = 0;
constexpr int SCHED_PRIORITY_DEFAULT = 0;

int px4_task_spawn_cmd(const char *, int, int, int, px4_main_t entry, char *const []);
int px4_task_delete(px4_task_t);
