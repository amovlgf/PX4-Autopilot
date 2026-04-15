#include "hw_test.h"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>
#include <cstring>
#include <cctype>
#include <cstdio>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>

static constexpr const char *kTestName = "UM982_CFG";
/** 飞控与 UM982 通信串口固定 115200。`config com1 230400` 只改模块内部 COM1 波特率，不改飞控串口。 */
static constexpr unsigned kUm982CfgBaud = 115200;
static constexpr int kWritePollMs = 200;
static constexpr useconds_t kInterCmdGapUs = 50 * 1000;

static constexpr int kRxIdleMs = 200;
/** 普通命令等待应答上限 */
static constexpr int kRxMaxTotalMs = 4000;
/** FRESET 后模块可能较慢才回 `response: OK` */
static constexpr int kRxMaxTotalMsFreset = 20000;
static constexpr int kRxIdleMsFreset = 500;
/** `config com1 230400` / `saveconfig` 后常插播 `$devicename,COM2`，真正 OK 略晚 */
static constexpr int kRxMaxTotalMsCom1Cfg = 12000;
static constexpr int kRxMaxTotalMsSaveconfig = 12000;
/** FRESET 应答 OK 后给模块重启/稳定时间再发下一条 */
static constexpr useconds_t kPostFresetDelayUs = 3 * 1000 * 1000;
/** `config com1 230400` 收到 OK 后再等一会再 saveconfig，避免与模块写 flash 竞态 */
static constexpr useconds_t kPostCom1CfgDelayUs = 1200 * 1000;
/** 若首轮仅收到 devicename 等插播、未见 OK，重试前短延时 */
static constexpr useconds_t kUm982CmdRetryGapUs = 300 * 1000;
/** `config com1 230400` 后模块还会异步输出 devicename，给更长稳定窗口 */
static constexpr int kPostCom1CfgDrainMs = 4500;
static constexpr int kPostCom1CfgDrainIdleMs = 1500;
/** 每条命令发送前先短排空，避免上条/他模块残留污染应答匹配。 */
static constexpr int kPreCmdDrainMs = 150;
static constexpr size_t kRxAccumMax = 1024;
static constexpr size_t kRxChunkMax = 192;

/** 模块应答格式: `$command,<echo>,response: OK*xx` */
static constexpr const char kResponseOk[] = "response: OK";
/* 避免在 hw_test 小栈线程上分配大数组 */
static char g_rx_accum[kRxAccumMax];
static char g_rx_chunk[kRxChunkMax];

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

	/* 等发送移位寄存器排空，避免未发完就 poll 读导致误判无应答 */
	(void)tcdrain(fd);

	return 0;
}

static void log_rx_lines(const char *data)
{
	char line[120] {};
	size_t j = 0;

	for (size_t i = 0; data[i] != '\0'; i++) {
		const unsigned char c = static_cast<unsigned char>(data[i]);

		if (c == '\r' || c == '\n') {
			if (j > 0) {
				line[j] = '\0';
				PX4_INFO("UM982 rx: %s", line);
				j = 0;
			}

			continue;
		}

		const char out = std::isprint(c) ? static_cast<char>(c) : '.';

		if (j < sizeof(line) - 1) {
			line[j++] = out;

		} else {
			line[j] = '\0';
			PX4_INFO("UM982 rx: %s", line);
			j = 0;
			line[j++] = out;
		}
	}

	if (j > 0) {
		line[j] = '\0';
		PX4_INFO("UM982 rx: %s", line);
	}
}

/** 排空串口残留（如 *01、$devicename,COM2），不做 OK 判定。 */
static void read_drain_lines(int fd, int max_total_ms)
{
	char drain[256] {};
	size_t a = 0;
	const hrt_abstime t0 = hrt_absolute_time();
	hrt_abstime last_rx = 0;

	while (hrt_elapsed_time(&t0) < (uint64_t)max_total_ms * 1000ULL) {
		pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLIN;
		const int pr = poll(&pfd, 1, 50);

		if (pr > 0 && (pfd.revents & POLLIN)) {
			const ssize_t n = read(fd, g_rx_chunk, sizeof(g_rx_chunk) - 1);

			if (n > 0) {
				last_rx = hrt_absolute_time();

				if ((size_t)n + a >= sizeof(drain)) {
					break;
				}

				memcpy(drain + a, g_rx_chunk, (size_t)n);
				a += (size_t)n;
				drain[a] = '\0';
				continue;
			}
		}

		/* 有过数据后，连续空闲一段时间才认为尾包结束（COM2 公告可能晚到） */
		if (last_rx != 0 && hrt_elapsed_time(&last_rx) > (uint64_t)kPostCom1CfgDrainIdleMs * 1000ULL) {
			break;
		}
	}

	if (a > 0) {
		log_rx_lines(drain);
	}
}

static bool response_ok_for_command(const char *accum, const char *cmd_tag);

/**
 * 读应答到 accum，打印分片；要求 accum 中出现 `response: OK` 才判成功（与串口助手行为一致）。
 * @param max_total_ms 总等待毫秒，<0 则用 kRxMaxTotalMs
 * @param rx_idle_ms 收到字节后 poll 空闲毫秒，<0 则用 kRxIdleMs
 */
static int read_response_expect_ok(int fd, const char *cmd_tag, int max_total_ms = -1, int rx_idle_ms = -1)
{
	if (max_total_ms < 0) {
		max_total_ms = kRxMaxTotalMs;
	}

	if (rx_idle_ms < 0) {
		rx_idle_ms = kRxIdleMs;
	}

	memset(g_rx_accum, 0, sizeof(g_rx_accum));
	size_t a = 0;
	const hrt_abstime t0 = hrt_absolute_time();
	bool got_any = false;
	bool got_ok = false;

	while (hrt_elapsed_time(&t0) < (uint64_t)max_total_ms * 1000ULL) {
		pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLIN;
		const int pr = poll(&pfd, 1, got_any ? rx_idle_ms : 50);

		if (pr > 0 && (pfd.revents & POLLIN)) {
			const ssize_t n = read(fd, g_rx_chunk, sizeof(g_rx_chunk) - 1);

			if (n > 0) {
				got_any = true;

				if ((size_t)n + a >= sizeof(g_rx_accum)) {
					PX4_ERR("UM982 FAIL: rx overflow for [%s]", cmd_tag);
					return -ENOBUFS;
				}

				memcpy(g_rx_accum + a, g_rx_chunk, (size_t)n);
				a += (size_t)n;
				g_rx_accum[a] = '\0';

				/* 必须命中“本命令回显 + OK”，避免上一条残包误判成功 */
				if (response_ok_for_command(g_rx_accum, cmd_tag)) {
					got_ok = true;
					break;
				}

				continue;
			}
		}
	}

	if (got_any) {
		log_rx_lines(g_rx_accum);
	}

	if (!got_any) {
		/* LogMessage 仅 127B，长句会被截断，拆成两行 */
		PX4_ERR("UM982 FAIL: [%s] zero RX bytes, timeout %d ms", cmd_tag, max_total_ms);
		PX4_ERR("UM982: not missing-OK; check uart wire/baud/conflict");
		return -ETIMEDOUT;
	}

	if (got_ok) {
		return 0;
	}

	PX4_ERR("UM982 FAIL: [%s] got %zu B, no substring %s", cmd_tag, a, kResponseOk);

	PX4_ERR("UM982: see UM982 rx lines above (garbled/wrong fmt?)");
	return -EIO;
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

/** 从 "CMD\r\n" 得到日志用短标签 */
static void cmd_to_tag(const char *line, char *tag, size_t tag_sz)
{
	const size_t n = strlen(line);
	size_t len = n;

	if (len >= 2 && line[len - 2] == '\r' && line[len - 1] == '\n') {
		len -= 2;
	}

	if (len >= tag_sz) {
		len = tag_sz - 1;
	}

	memcpy(tag, line, len);
	tag[len] = '\0';
}

static bool response_ok_for_command(const char *accum, const char *cmd_tag)
{
	if (strstr(accum, kResponseOk) == nullptr) {
		return false;
	}

	char expect[96] {};
	(void)snprintf(expect, sizeof(expect), "$command,%s", cmd_tag);
	return (strstr(accum, expect) != nullptr);
}

int test_um982_cfg(const char *dev, unsigned baud)
{
	(void)baud;

	static constexpr const char *cmds[] {
		"FRESET\r\n",
		"config com1 230400\r\n",
		"GPGGA COM1 0.2\r\n",
		"GPRMC COM1 0.2\r\n",
		"AGRICA COM1 0.2\r\n",
		"GPGSA COM1 0.2\r\n",
		"GPGST COM1 0.2\r\n",
		"UNIHEADINGA COM1 0.4\r\n",
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

	for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
		char tag[80] {};
		cmd_to_tag(cmds[i], tag, sizeof(tag));
		read_drain_lines(fd, kPreCmdDrainMs);
		(void)tcflush(fd, TCIFLUSH);

		const int wr = write_line(fd, cmds[i]);

		if (wr != 0) {
			close(fd);
			HW_TEST_FAIL(kTestName, "send [%s] failed: %d", tag, -wr);
			return wr;
		}

		log_tx_line(cmds[i]);

		const bool is_freset = (strcmp(tag, "FRESET") == 0);
		const bool is_cfg_com1_230400 = (strcmp(tag, "config com1 230400") == 0);
		const bool is_saveconfig = (strcmp(tag, "saveconfig") == 0);
		int rr = is_freset ? read_response_expect_ok(fd, tag, kRxMaxTotalMsFreset, kRxIdleMsFreset)
			 : is_cfg_com1_230400 ? read_response_expect_ok(fd, tag, kRxMaxTotalMsCom1Cfg, kRxIdleMsFreset)
			 : is_saveconfig ? read_response_expect_ok(fd, tag, kRxMaxTotalMsSaveconfig, kRxIdleMsFreset)
			 : read_response_expect_ok(fd, tag);

		if (rr != 0 && is_cfg_com1_230400) {
			px4_usleep(kUm982CmdRetryGapUs);

			const int wr_retry = write_line(fd, cmds[i]);

			if (wr_retry == 0) {
				log_tx_line(cmds[i]);
				rr = read_response_expect_ok(fd, tag, kRxMaxTotalMsCom1Cfg, kRxIdleMsFreset);
			}
		}

		if (rr != 0 && is_freset) {
			px4_usleep(kUm982CmdRetryGapUs);
			read_drain_lines(fd, kPreCmdDrainMs);

			const int wr_retry = write_line(fd, cmds[i]);

			if (wr_retry == 0) {
				log_tx_line(cmds[i]);
				rr = read_response_expect_ok(fd, tag, kRxMaxTotalMsFreset, kRxIdleMsFreset);
			}
		}

		if (rr != 0 && is_saveconfig) {
			px4_usleep(kUm982CmdRetryGapUs);

			const int wr_retry = write_line(fd, cmds[i]);

			if (wr_retry == 0) {
				log_tx_line(cmds[i]);
				rr = read_response_expect_ok(fd, tag, kRxMaxTotalMsSaveconfig, kRxIdleMsFreset);
			}
		}

		if (rr != 0) {
			close(fd);
			HW_TEST_FAIL(kTestName, "[%s] verify failed: %d", tag, rr);
			return rr;
		}

		if (is_freset) {
			px4_usleep(kPostFresetDelayUs);
		}

		if (is_cfg_com1_230400) {
			px4_usleep(kPostCom1CfgDelayUs);
			read_drain_lines(fd, kPostCom1CfgDrainMs);
			/* 再留一小段静默时间，避免 devicename 与下一条命令应答混叠 */
			px4_usleep(300 * 1000);
		}

		px4_usleep(kInterCmdGapUs);
	}

	close(fd);
	PX4_INFO("UM982 PASS (all commands returned OK)");
	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
}
