#include "hw_test.h"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>
#include <cstring>
#include <cctype>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>

static constexpr const char *kTestName = "UM982_CFG";
/** UM982 配置阶段主机串口必须与模块一致，固定 115200；`config com1 230400` 后再切 230400。 */
static constexpr unsigned kUm982CfgBaud = 115200;
static constexpr int kWritePollMs = 200;
static constexpr useconds_t kInterCmdGapUs = 50 * 1000;

/** After last RX byte, exit after this many ms with no data. */
static constexpr int kRxIdleMs = 200;

/** Hard cap on total wait for one command reply. */
static constexpr int kRxMaxTotalMs = 2500;

static speed_t baud_to_speed(unsigned baud)
{
	switch (baud) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
#ifdef B230400
	case 230400: return B230400;
#endif
	default: return 0;
	}
}

static int setup_uart(int fd, unsigned baud)
{
	termios t {};

	if (tcgetattr(fd, &t) != 0) {
		return -errno;
	}

	cfmakeraw(&t);
	t.c_cflag |= (CLOCAL | CREAD);
	t.c_cflag &= ~CRTSCTS;
	t.c_cflag &= ~PARENB;
	t.c_cflag &= ~CSTOPB;
	t.c_cflag &= ~CSIZE;
	t.c_cflag |= CS8;
	t.c_cc[VMIN] = 0;
	t.c_cc[VTIME] = 0;

	const speed_t speed = baud_to_speed(baud);

	if (speed == 0) {
		return -EINVAL;
	}

	(void)cfsetispeed(&t, speed);
	(void)cfsetospeed(&t, speed);

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		return -errno;
	}

	(void)tcflush(fd, TCIOFLUSH);
	return 0;
}

static int write_line(int fd, const char *line)
{
	const size_t len = strlen(line);
	size_t sent = 0;

	while (sent < len) {
		pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLOUT;

		const int pr = poll(&pfd, 1, kWritePollMs);

		if (pr <= 0) {
			return -ETIMEDOUT;
		}

		const ssize_t n = write(fd, line + sent, len - sent);

		if (n > 0) {
			sent += static_cast<size_t>(n);

		} else if (n < 0 && errno != EAGAIN) {
			return -errno;
		}
	}

	return 0;
}

/** Sanitize for single-line log (keep \\r \\n \\t). */
static void log_rx_chunk(const char *prefix, const char *data, size_t n)
{
	char safe[400];
	size_t j = 0;

	for (size_t i = 0; i < n && j < sizeof(safe) - 2; i++) {
		const unsigned char c = static_cast<unsigned char>(data[i]);

		if (c == '\r' || c == '\n' || c == '\t') {
			safe[j++] = static_cast<char>(c);

		} else if (std::isprint(c)) {
			safe[j++] = static_cast<char>(c);

		} else {
			safe[j++] = '.';
		}
	}

	safe[j] = '\0';
	PX4_INFO("%s%s", prefix, safe);
}

/** Read reply in bursts; after any data, stop when poll times out idle (kRxIdleMs). */
static void read_and_print_response(int fd)
{
	const hrt_abstime t0 = hrt_absolute_time();
	bool got_any = false;

	while (hrt_elapsed_time(&t0) < (uint64_t)kRxMaxTotalMs * 1000ULL) {
		pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLIN;
		const int pr = poll(&pfd, 1, got_any ? kRxIdleMs : 50);

		if (pr > 0 && (pfd.revents & POLLIN)) {
			char buf[384];
			const ssize_t n = read(fd, buf, sizeof(buf) - 1);

			if (n > 0) {
				got_any = true;
				log_rx_chunk("UM982 rx: ", buf, static_cast<size_t>(n));
				continue;
			}
		}

		if (got_any && pr == 0) {
			break;
		}
	}

	if (!got_any) {
		PX4_INFO("UM982 rx: (no data)");
	}
}

static void log_tx_line(const char *line)
{
	const size_t n = strlen(line);

	if (n >= 2 && line[n - 2] == '\r' && line[n - 1] == '\n') {
		PX4_INFO("UM982 tx: %.*s", (int)(n - 2), line);

	} else {
		PX4_INFO("UM982 tx: %s", line);
	}
}

int test_um982_cfg(const char *dev, unsigned baud)
{
	(void)baud; /* 配置阶段固定 kUm982CfgBaud，与 CLI 传入值无关 */

	static constexpr const char *cmds[] {
		"FRESET\r\n",
		"GPGGA COM1 0.2\r\n",
		"GPRMC COM1 0.2\r\n",
		"AGRICA COM1 0.2\r\n",
		"GPGSA COM1 0.2\r\n",
		"GPGST COM1 0.2\r\n",
		"UNIHEADINGA COM1 0.4\r\n",
		"config com1 230400\r\n",
		"saveconfig\r\n"
	};

	const int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		HW_TEST_FAIL(kTestName, "open %s failed: %d", dev, errno);
		return -errno;
	}

	const int cfg_ret = setup_uart(fd, kUm982CfgBaud);

	if (cfg_ret != 0) {
		close(fd);
		HW_TEST_FAIL(kTestName, "set %s to %u baud failed: %d", dev, kUm982CfgBaud, -cfg_ret);
		return cfg_ret;
	}

	PX4_INFO("UM982: %s set to %u baud, then sending config", dev, kUm982CfgBaud);

	for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
		const int wr = write_line(fd, cmds[i]);

		if (wr != 0) {
			close(fd);
			HW_TEST_FAIL(kTestName, "send cmd %u failed: %d", static_cast<unsigned>(i + 1), -wr);
			return wr;
		}

		log_tx_line(cmds[i]);
		read_and_print_response(fd);

		/* After "config com1 230400", module may switch COM1 baud; match host UART. */
		if (i == 7) {
			px4_usleep(50 * 1000);
			const int re_cfg = setup_uart(fd, 230400);

			if (re_cfg != 0) {
				close(fd);
				HW_TEST_FAIL(kTestName, "switch local baud to 230400 failed: %d", -re_cfg);
				return re_cfg;
			}

			read_and_print_response(fd);
		}

		px4_usleep(kInterCmdGapUs);
	}

	close(fd);
	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
}
