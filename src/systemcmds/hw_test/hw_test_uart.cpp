#include "hw_test.h"

#include <drivers/drv_hrt.h>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <cstdio>
#include <termios.h>
#include <sys/ioctl.h>

static constexpr const char *kTestName = "UART";
/* Short payload keeps FAT fast; separate write/read timers avoid starving RX.
 * ICF6 FAT image: build `amovlab_icf6_test` (no rc_input, /dev/ttyS4 free) — see boards/amovlab/icf6/test.px4board. */
static constexpr size_t kLoopSize = 32;
static constexpr int kPollSliceMs = 20;
static constexpr int kUartWriteTimeoutMs = 300;
static constexpr int kUartReadTimeoutMs = 500;
static constexpr int kRetryCount = 3;
static constexpr int kRetryIntervalUs = 25000;
static constexpr unsigned kPostWriteSettleUs = 200;

static void uart_item_pass(const char *item)
{
	PX4_INFO("[PASS] %s %s", kTestName, item);
	printf("[PASS] %s %s\n", kTestName, item);
}

static void uart_item_fail(const char *item, const char *reason)
{
	PX4_ERR("[FAIL] %s %s (%s)", kTestName, item, reason);
	printf("[FAIL] %s %s (%s)\n", kTestName, item, reason);
}

static void uart_item_fail_errno(const char *item, const char *op, int err)
{
	PX4_ERR("[FAIL] %s %s (%s failed: %d)", kTestName, item, op, err);
	printf("[FAIL] %s %s (%s failed: %d)\n", kTestName, item, op, err);
}

static int open_uart(const char *dev)
{
	const int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	return fd;
}

static int configure_uart_raw_8n1_115200(int fd)
{
	struct termios t {};

	if (tcgetattr(fd, &t) != 0) {
		return -errno;
	}

	// raw mode, 8N1, no flow control
	cfmakeraw(&t);
	t.c_cflag |= (CLOCAL | CREAD);
	t.c_cflag &= ~CRTSCTS;
	t.c_cflag &= ~PARENB;
	t.c_cflag &= ~CSTOPB;
	t.c_cflag &= ~CSIZE;
	t.c_cflag |= CS8;

	// make reads return as soon as bytes arrive
	t.c_cc[VMIN] = 0;
	t.c_cc[VTIME] = 0;

	(void)cfsetispeed(&t, B115200);
	(void)cfsetospeed(&t, B115200);

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		return -errno;
	}

	(void)tcflush(fd, TCIOFLUSH);
	return 0;
}

static int uart_loopback_once(const char *dev, int &out_errno, const char *&out_reason)
{
	out_errno = 0;
	out_reason = nullptr;

	const int fd = open_uart(dev);

	if (fd < 0) {
		out_errno = errno;
		out_reason = "open failed";
		return -out_errno;
	}

	// If this UART is already used by another module (e.g. MAVLink on TELEM1/TELEM2),
	// loopback bytes may be consumed by the other reader. Try to request exclusive access.
	if (ioctl(fd, TIOCEXCL, 0) != 0) {
		const int e = errno;

		// Some NuttX serial drivers don't implement TIOCEXCL (commonly ENOTTY=25).
		// In that case, continue without exclusive access.
		if (e != ENOTTY && e != EINVAL) {
			out_errno = e;
			close(fd);
			out_reason = "port busy";
			return -EBUSY;
		}
	}

	const int cfg_ret = configure_uart_raw_8n1_115200(fd);

	if (cfg_ret != 0) {
		out_errno = -cfg_ret;
		close(fd);
		out_reason = "configure failed";
		return cfg_ret;
	}

	char tx[kLoopSize];
	char rx[kLoopSize];

	for (size_t i = 0; i < kLoopSize; ++i) {
		// printable payload (avoid control chars)
		tx[i] = static_cast<char>('A' + (i % 26));
	}

	// Write with timeout protection (non-blocking fd).
	size_t total_written = 0;
	const hrt_abstime write_start = hrt_absolute_time();

	while (total_written < sizeof(tx) && (hrt_elapsed_time(&write_start) / 1000) < kUartWriteTimeoutMs) {
		struct pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLOUT;

		const int poll_ret = poll(&pfd, 1, kPollSliceMs);

		if (poll_ret > 0 && (pfd.revents & POLLOUT)) {
			const ssize_t n = write(fd, &tx[total_written], sizeof(tx) - total_written);

			if (n > 0) {
				total_written += static_cast<size_t>(n);

			} else if (n < 0 && errno != EAGAIN) {
				out_errno = errno;
				close(fd);
				out_reason = "write failed";
				return -EIO;
			}
		}
	}

	if (total_written != sizeof(tx)) {
		out_errno = errno;
		close(fd);
		out_reason = "write timeout";
		return -ETIMEDOUT;
	}

	/* Let loopback + driver DMA/FIFO finish; read gets its own full timeout window. */
	px4_usleep(kPostWriteSettleUs);

	size_t total_read = 0;
	const hrt_abstime read_start = hrt_absolute_time();

	while (total_read < sizeof(rx) && (hrt_elapsed_time(&read_start) / 1000) < kUartReadTimeoutMs) {
		struct pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLIN;

		const int poll_ret = poll(&pfd, 1, kPollSliceMs);

		if (poll_ret > 0 && (pfd.revents & POLLIN)) {
			const ssize_t n = read(fd, &rx[total_read], sizeof(rx) - total_read);

			if (n > 0) {
				total_read += n;
			}
		}
	}

	close(fd);

	if (total_read != sizeof(rx)) {
		out_reason = "read timeout";
		return -ETIMEDOUT;
	}

	if (memcmp(tx, rx, sizeof(tx)) != 0) {
		out_reason = "loopback data mismatch";
		return -EIO;
	}

	return 0;
}

int test_uart()
{
	// Only test the UARTs that are physically looped back (RX/TX shorted) on ICF6.
	// Mapping reference (boards/amovlab/icf6/nuttx-config/include/board.h):
	// USART2 -> /dev/ttyS0
	// USART3 -> /dev/ttyS1
	// UART5  -> /dev/ttyS3
	// USART6 -> /dev/ttyS4  (same loopback test; FAT image should not run rc_input on this port)
	// UART7  -> /dev/ttyS5
	static constexpr const char *loop_ports[] {"/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS5"};
	int ret = 0;

	for (const char *dev : loop_ports) {
		int saved_errno = 0;
		const char *reason = nullptr;
		int r = -ETIMEDOUT;

		for (int attempt = 0; attempt < kRetryCount; ++attempt) {
			r = uart_loopback_once(dev, saved_errno, reason);

			if (r == 0) {
				break;
			}

			px4_usleep(kRetryIntervalUs);
		}

		if (r == 0) {
			uart_item_pass(dev);

		} else {
			if (saved_errno != 0 && reason != nullptr) {
				// Provide both a FAIL flag and the underlying errno.
				if (strcmp(reason, "open failed") == 0) {
					uart_item_fail_errno(dev, "open", saved_errno);

				} else if (strcmp(reason, "write failed") == 0) {
					uart_item_fail_errno(dev, "write", saved_errno);

				} else {
					uart_item_fail_errno(dev, reason, saved_errno);
				}

			} else {
				uart_item_fail(dev, reason != nullptr ? reason : "unknown error");
			}
			PX4_ERR("[FAIL] %s %s (failed after %d attempts)", kTestName, dev, kRetryCount);
			printf("[FAIL] %s %s (failed after %d attempts)\n", kTestName, dev, kRetryCount);

			if (ret == 0) {
				ret = r;
			}
		}
	}

	if (ret == 0) {
		HW_TEST_PASS(kTestName);
		return HW_TEST_OK;
	}

	HW_TEST_FAIL(kTestName, "one or more uart checks failed");
	return ret;
}
