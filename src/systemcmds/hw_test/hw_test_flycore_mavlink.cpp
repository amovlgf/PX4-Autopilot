/****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#include "hw_test.h"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/tasks.h>

#include <flycore_test/mavlink.h>

#include <cerrno>
#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr uint8_t kFlightSystem = 1;
constexpr uint8_t kFlightComponent = 1;
constexpr uint8_t kNoItem = UINT8_MAX;
constexpr unsigned kStation = 0;
constexpr unsigned kResponder = 1;
constexpr unsigned kNumLinks = 2;
constexpr unsigned kPingAttempts = 3;
constexpr hrt_abstime kResponseInterval = 1000000;
constexpr hrt_abstime kWriteTimeout = 50000;
constexpr unsigned kLoopDelayUs = 10000;

struct Lock {
	explicit Lock(pthread_mutex_t &mutex) : _mutex(mutex) { pthread_mutex_lock(&_mutex); }
	~Lock() { pthread_mutex_unlock(&_mutex); }
	Lock(const Lock &) = delete;
	Lock &operator=(const Lock &) = delete;

private:
	pthread_mutex_t &_mutex;
};

struct Ping {
	bool active{};
	int result{};
	unsigned attempts{};
	uint32_t sequence{};
	hrt_abstime sent_at{};
	hrt_abstime next_send{};
};

struct Link {
	const char *device;
	const char *baud_parameter;
	const char *system_parameter;
	const char *component_parameter;
	speed_t speed{};
	uint8_t system{};
	uint8_t component{};
	int fd{-1};
	hrt_abstime reopen_at{};
	mavlink_message_t rx_buffer{};
	mavlink_status_t rx_status{};
	Ping ping{};
	uint32_t next_ping_sequence{};
};

Link links[kNumLinks] {
	{"/dev/ttyS1", "FQ_8550_BAUD", "FQ_8550_SYS", "FQ_8550_COMP"},
	{"/dev/ttyS3", "FQ_6490_BAUD", "FQ_6490_SYS", "FQ_6490_COMP"}
};

struct Run {
	bool active{};
	bool busy_pending{};
	bool accepted_pending{};
	bool progress_pending{};
	uint8_t item{kNoItem};
	bool report_pending{};
	HwTestQResult result{};
};

Run run;
pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
px4_sem_t worker_request;
bool started{};

speed_t serial_speed(int32_t rate)
{
	switch (rate) {
	case 9600: return B9600;

	case 19200: return B19200;

	case 38400: return B38400;

	case 57600: return B57600;

	case 115200: return B115200;

	case 230400: return B230400;

	case 460800: return B460800;

	case 921600: return B921600;

	default: return 0;
	}
}

bool configure(Link &link)
{
	int32_t baud = 0;
	int32_t system = 0;
	int32_t component = 0;

	if (param_get(param_find(link.baud_parameter), &baud) != 0
	    || param_get(param_find(link.system_parameter), &system) != 0
	    || param_get(param_find(link.component_parameter), &component) != 0) {
		PX4_ERR("Missing inspection UART parameters for %s", link.device);
		return false;
	}

	link.speed = serial_speed(baud);

	if (!link.speed || system < 1 || system > UINT8_MAX || component < 1 || component > UINT8_MAX
	    || (system == kFlightSystem && component == kFlightComponent)) {
		PX4_ERR("Invalid inspection UART baud or MAVLink ID for %s", link.device);
		return false;
	}

	link.system = static_cast<uint8_t>(system);
	link.component = static_cast<uint8_t>(component);
	return true;
}

int open_link(const Link &link)
{
	const int fd = open(link.device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		return -1;
	}

	termios options{};

	if (tcgetattr(fd, &options) == 0) {
		cfmakeraw(&options);
		options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
		options.c_cflag |= CLOCAL | CREAD | CS8;
		options.c_cc[VMIN] = 0;
		options.c_cc[VTIME] = 0;

		if (cfsetispeed(&options, link.speed) == 0 && cfsetospeed(&options, link.speed) == 0
		    && tcsetattr(fd, TCSANOW, &options) == 0) {
			return fd;
		}
	}

	close(fd);
	return -1;
}

bool transmit(const Link &link, const mavlink_message_t &message)
{
	if (link.fd < 0) {
		return false;
	}

	uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
	const unsigned size = mavlink_msg_to_send_buffer(bytes, &message);
	unsigned offset = 0;
	const hrt_abstime start = hrt_absolute_time();

	while (offset < size && hrt_elapsed_time(&start) < kWriteTimeout) {
		const ssize_t written = write(link.fd, bytes + offset, size - offset);

		if (written > 0) {
			offset += static_cast<unsigned>(written);

		} else if (written == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
			px4_usleep(1000);

		} else {
			return false;
		}
	}

	return offset == size;
}

void send_status(FLYCORE_TEST_PHASE phase, uint8_t item = kNoItem)
{
	const Link &station = links[kStation];
	mavlink_message_t message{};
	mavlink_msg_flycore_test_status_pack_chan(kFlightSystem, kFlightComponent, MAVLINK_COMM_0, &message,
			station.system, station.component, phase, item);
	transmit(station, message);
}

void send_result()
{
	const Link &station = links[kStation];
	mavlink_message_t message{};
	const bool pass = run.result.executed_mask == HW_TEST_Q_REQUIRED_MASK
			  && run.result.passed_mask == HW_TEST_Q_REQUIRED_MASK;
	mavlink_msg_flycore_test_result_pack_chan(kFlightSystem, kFlightComponent, MAVLINK_COMM_0, &message,
			run.result.executed_mask, run.result.passed_mask, station.system, station.component, pass);

	if (!transmit(station, message)) {
		PX4_WARN("Could not send inspection result to 8550");
	}
}

bool for_flight_controller(uint8_t system, uint8_t component)
{
	return system == kFlightSystem && component == kFlightComponent;
}

void handle_start(const mavlink_message_t &message)
{
	mavlink_flycore_test_start_t request{};
	mavlink_msg_flycore_test_start_decode(&message, &request);

	if (!for_flight_controller(request.target_system, request.target_component)) {
		return;
	}

	if (run.active || run.report_pending || run.busy_pending) {
		send_status(FLYCORE_TEST_PHASE_BUSY);
		return;
	}

	run.active = true;

	if (px4_sem_post(&worker_request) != 0) {
		run.active = false;
		send_status(FLYCORE_TEST_PHASE_BUSY);
	}
}

void handle_message(unsigned index, const mavlink_message_t &message)
{
	Link &link = links[index];

	if (message.magic != MAVLINK_STX || message.sysid != link.system || message.compid != link.component) {
		return;
	}

	const Lock lock(state_mutex);

	if (index == kResponder && message.msgid == MAVLINK_MSG_ID_PING) {
		mavlink_ping_t payload{};
		mavlink_msg_ping_decode(&message, &payload);

		if (payload.target_system == 0 && payload.target_component == 0) {
			mavlink_message_t response{};
			mavlink_msg_ping_pack_chan(kFlightSystem, kFlightComponent, index, &response,
						   payload.time_usec, payload.seq, link.system, link.component);
			transmit(link, response);

		} else if (link.ping.active && link.ping.attempts != 0
			   && hrt_absolute_time() < link.ping.next_send
			   && for_flight_controller(payload.target_system, payload.target_component)
			   && payload.seq == link.ping.sequence && payload.time_usec == link.ping.sent_at) {
			link.ping.result = 0;
			link.ping.active = false;
		}

	} else if (index == kStation && message.msgid == MAVLINK_MSG_ID_FLYCORE_TEST_START) {
		handle_start(message);
	}
}

void receive(unsigned index)
{
	Link &link = links[index];

	if (link.fd < 0) {
		if (hrt_absolute_time() < link.reopen_at) {
			return;
		}

		link.reopen_at = hrt_absolute_time() + kResponseInterval;
		link.fd = open_link(link);

		if (link.fd < 0) {
			return;
		}

		link.rx_buffer = {};
		link.rx_status = {};
		PX4_INFO("MAVLink inspection link ready on %s", link.device);
	}

	uint8_t bytes[128];
	const ssize_t count = read(link.fd, bytes, sizeof(bytes));

	if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
		PX4_ERR("Inspection UART read failed on %s (%d)", link.device, errno);
		close(link.fd);
		link.fd = -1;
		return;
	}

	mavlink_message_t message{};

	for (ssize_t i = 0; i < count; ++i) {
		if (mavlink_frame_char_buffer(&link.rx_buffer, &link.rx_status, bytes[i], &message, nullptr)
		    == MAVLINK_FRAMING_OK) {
			handle_message(index, message);
		}
	}
}

void tick_ping(unsigned index)
{
	Link &link = links[index];
	Ping &ping = link.ping;
	const hrt_abstime now = hrt_absolute_time();

	if (!ping.active || now < ping.next_send) {
		return;
	}

	if (ping.attempts >= kPingAttempts) {
		ping.result = -ETIMEDOUT;
		ping.active = false;
		return;
	}

	ping.sequence = link.next_ping_sequence++;
	ping.sent_at = now;
	ping.next_send = now + kResponseInterval;
	++ping.attempts;

	mavlink_message_t request{};
	mavlink_msg_ping_pack_chan(kFlightSystem, kFlightComponent, index, &request, ping.sent_at, ping.sequence, 0, 0);
	transmit(link, request);
}

void tick_run()
{
	if (run.busy_pending) {
		send_status(FLYCORE_TEST_PHASE_BUSY);
		run.busy_pending = false;
	}

	if (run.accepted_pending) {
		send_status(FLYCORE_TEST_PHASE_ACCEPTED);
		run.accepted_pending = false;
	}

	if (run.active && run.progress_pending) {
		send_status(FLYCORE_TEST_PHASE_RUNNING, run.item);
		run.progress_pending = false;
	}

	if (run.report_pending) {
		send_result();
		run.report_pending = false;
	}
}

int serial_main(int, char *[])
{
	for (;;) {
		for (unsigned index = 0; index < kNumLinks; ++index) {
			receive(index);
		}

		{
			const Lock lock(state_mutex);

			tick_ping(kResponder);

			tick_run();
		}

		px4_usleep(kLoopDelayUs);
	}

	return 0;
}

void report_progress(unsigned item, void *)
{
	const Lock lock(state_mutex);

	if (run.item == kNoItem) {
		run.accepted_pending = true;
	}

	run.item = static_cast<uint8_t>(item);
	run.progress_pending = true;
}

int worker_main(int, char *[])
{
	for (;;) {
		int wait_result;

		do {
			wait_result = px4_sem_wait(&worker_request);
		} while (wait_result != 0 && errno == EINTR);

		if (wait_result != 0) {
			PX4_ERR("Inspection worker wait failed (%d)", errno);
			return -errno;
		}

		HwTestQResult result{};
		const int ret = hw_test_q_run(result, report_progress);
		const Lock lock(state_mutex);

		if (ret == -EBUSY && result.executed_mask == 0) {
			run.busy_pending = true;

		} else {
			if (run.item == kNoItem) {
				run.accepted_pending = true;
			}

			run.result = result;
			run.report_pending = true;
		}

		run.item = kNoItem;
		run.progress_pending = false;
		run.active = false;
	}

	return 0;
}
} // namespace

int hw_test_q_service_start()
{
	const Lock lock(state_mutex);

	if (started) {
		return 0;
	}

	for (Link &link : links) {
		if (!configure(link)) {
			return -EINVAL;
		}
	}

	if (px4_sem_init(&worker_request, 0, 0) != 0) {
		return -errno;
	}

	const px4_task_t worker = px4_task_spawn_cmd("flycore_test", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
				  6144, worker_main, nullptr);

	if (worker < 0) {
		px4_sem_destroy(&worker_request);
		return worker;
	}

	const px4_task_t serial = px4_task_spawn_cmd("flycore_test_io", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
				  4096, serial_main, nullptr);

	if (serial < 0) {
		px4_task_delete(worker);
		px4_sem_destroy(&worker_request);
		return serial;
	}

	started = true;
	return 0;
}

int hw_test_q_ping_6490()
{
	Ping &ping = links[kResponder].ping;

	{
		const Lock lock(state_mutex);

		if (!started) {
			return -ENODEV;
		}

		if (ping.active) {
			return -EBUSY;
		}

		ping = {};
		ping.active = true;
		ping.result = -ETIMEDOUT;
	}

	const hrt_abstime deadline = hrt_absolute_time() + (kPingAttempts + 1) * kResponseInterval;

	for (;;) {
		{
			const Lock lock(state_mutex);

			if (!ping.active) {
				return ping.result;
			}

			if (hrt_absolute_time() >= deadline) {
				ping.active = false;
				return -ETIMEDOUT;
			}
		}

		px4_usleep(kLoopDelayUs);
	}
}
