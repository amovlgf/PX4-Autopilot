/****************************************************************************
 * hw_test CAN — NSH 命令与帧格式备忘（随本文件维护，方便产测查阅）
 *
 * 连接器 ↔ NuttX（STM32H7 FDCAN 典型映射）
 *   CAN1 口 → can0 → FDCAN1    hw_test 里 bus / 参数写 0
 *   CAN2 口 → can1 → FDCAN2    hw_test 里 bus / 参数写 1
 *
 * ── hw_test can / hw_test can_ext
 *   用途：CAN1 与 CAN2 外总线短接后双向互测（单条命令内完成两向）。
 *   接线：CAN1_H↔CAN2_H，CAN1_L↔CAN2_L，GND 共地；总线终端约 60Ω（常见两端各 120Ω）。
 *   帧类型：经典 CAN，扩展帧 29 bit，ID = 0x001ABCDE（见 kCanIdExt），DLC = 8。
 *   载荷：can0→can1：55 AA 12 34 7E 80 DE AD
 *         can1→can0：A5 5A 21 43 E7 08 ED DA
 *   说明：当前产测逻辑仅使用扩展帧，减少现场标准帧差异导致的误判。
 *
 * ── hw_test can_tx <0|1> [duration_s] [period_ms] [loopback]
 *   用途：单路对外发周期帧，给 CAN 分析仪抓包（固定扩展帧）。
 *   默认：duration_s=30，period_ms=100；第 4 参数 loopback：0=真实总线，1=CAN_RAW_LOOPBACK（无总线 ACK 也可跑栈侧自测）。
 *   帧类型：经典 CAN，扩展帧 29 bit，ID = 0x18DAF110（见 kBenchTxCanIdExt），DLC = 8。
 *   载荷：ASCII「PX4T」+ uint32 小端递增序号（每发一帧 +1）。
 *   注意：loopback=0 时总线需有节点 ACK（分析仪勿用纯 Listen-only）；共地 + 终端；仲裁段 1 Mbps。
 *
 * ── hw_test can_rx <0|1> [duration_s]
 *   用途：单路监听并打印收到的帧；默认 duration_s=30。
 *   分析仪：经典 CAN 建议 1 Mbps；ID 可自定。若打开 CAN_RAW_FD_FRAMES，也会打印 CAN FD（CAN RXFD）。
 *
 * errno=11 (EAGAIN) on can_tx：多为总线无 ACK（接线/终端/分析仪模式）；可试 can_tx … 1 开 loopback 或修正分析仪为 Normal。
 *
 * ── 测试命令案例（NSH / QGC MAVLink Console 同）
 *
 *   # 两路已短接 + 共地 + 终端后，一键互测
 *   hw_test can
 *
 *   # CAN1 对外发 30s（默认每 100ms），分析仪接 CAN1、Normal、1M
 *   hw_test can_tx 0
 *
 *   # CAN2 发 60s，每 200ms 一帧
 *   hw_test can_tx 1 60 200
 *
 *   # 无分析仪或只听模式导致无 ACK 时，栈内回环自测（线上可能看不到波形）
 *   hw_test can_tx 1 30 100 1
 *
 *   # 监听 CAN1 口 60s，期间用分析仪往 CAN1 发经典帧（如 ID 0x100）
 *   hw_test can_rx 0 60
 *
 *   # 监听 CAN2 口，默认 30s
 *   hw_test can_rx 1
 *
 *   # 产测顺序示例：先分别验证两口，再短接跑互测
 *   hw_test can_tx 0 5 100
 *   hw_test can_rx 0 10
 *   hw_test can_tx 1 5 100
 *   hw_test can_rx 1 10
 *   hw_test can
 ****************************************************************************/

#include "hw_test.h"

#include <drivers/drv_hrt.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <cstdint>

#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <net/if.h>
#include <nuttx/can.h>
#include <nuttx/net/ioctl.h>
#include <netpacket/can.h>
#endif

static constexpr const char *kTestName = "CAN";
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
static constexpr const char *kCanIf0 = "can0"; // FDCAN1
static constexpr const char *kCanIf1 = "can1"; // FDCAN2
static constexpr canid_t kCanIdExt = (CAN_EFF_FLAG | 0x001ABCDE); /* 29-bit */
/* Bus timing comes from NuttX FDCAN Kconfig; do not SIOCSCANBITRATE on every open (re-inits controller). */
static constexpr int kCanRecvTimeoutMs = 800;
static constexpr unsigned kCanRecvPollSliceUs = 500;
static constexpr unsigned kCanDirectionRetries = 3;
static constexpr unsigned kCanDirectionRetryGapUs = 20000;
static constexpr canid_t kBenchTxCanIdExt = (CAN_EFF_FLAG | 0x18DAF110); /* 29-bit */
#endif

#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
static int can_iface_up(const char *ifname)
{
	const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (fd < 0) {
		PX4_ERR("socket(PF_CAN) %s ifup failed: %d", ifname, errno);
		return -errno;
	}

	struct ifreq ifr_fl {};
	(void)strncpy(ifr_fl.ifr_name, ifname, IFNAMSIZ - 1);

	if (ioctl(fd, SIOCGIFFLAGS, &ifr_fl) == 0) {
		ifr_fl.ifr_flags |= IFF_UP;

		if (ioctl(fd, SIOCSIFFLAGS, &ifr_fl) < 0) {
			PX4_WARN("SIOCSIFFLAGS %s IFF_UP failed: %d", ifname, errno);
			close(fd);
			return -errno;
		}

	} else {
		PX4_WARN("SIOCGIFFLAGS %s failed: %d", ifname, errno);
		close(fd);
		return -errno;
	}

	close(fd);
	return 0;
}

/* ifindex + bitrate: helps spot wrong iface mapping or mismatched FDCAN1/FDCAN2 Kconfig rates. */
static void can_log_diagnostics()
{
	const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (fd < 0) {
		return;
	}

	static const char *const names[] = {kCanIf0, kCanIf1};

	for (unsigned n = 0; n < sizeof(names) / sizeof(names[0]); n++) {
		const char *const name = names[n];
		struct ifreq ifr {};

		(void)strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);

		if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
			PX4_WARN("CAN diag: %s SIOCGIFINDEX failed: %d", name, errno);
			continue;
		}

		PX4_INFO("CAN diag: %s ifindex=%d (expect FDCAN1=can0, FDCAN2=can1)", name, ifr.ifr_ifindex);

		(void)strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
		memset(&ifr.ifr_ifru.ifru_can_data, 0, sizeof(ifr.ifr_ifru.ifru_can_data));

		if (ioctl(fd, SIOCGCANBITRATE, &ifr) == 0) {
			const struct can_ioctl_data_s *const br = &ifr.ifr_ifru.ifru_can_data;
			PX4_INFO("CAN diag: %s bitrate arbi=%u kbit/s data=%u kbit/s", name,
				 (unsigned)br->arbi_bitrate, (unsigned)br->data_bitrate);
		}
	}

	close(fd);
}

/* Re-init controller with existing bitrate (clears sticky state / filters in some cases). */
static void can_reinit_iface_if_possible(const char *ifname)
{
	const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (fd < 0) {
		return;
	}

	struct ifreq ifr {};
	(void)strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

	if (ioctl(fd, SIOCGCANBITRATE, &ifr) == 0) {
		/* Write back same settings (kbit/s) to force fdcan_initialize()+ifup */
		(void)ioctl(fd, SIOCSCANBITRATE, &ifr);
	}

	close(fd);
}

/* Accept-all filter; loopback per test (off for real bus / cross-iface). */
static void can_apply_raw_sockopts(int fd, bool loopback_enable)
{
	struct can_filter acc_all {};
	acc_all.can_id = 0;
	acc_all.can_mask = 0;

	if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &acc_all, sizeof(acc_all)) < 0) {
		PX4_WARN("CAN_RAW_FILTER (accept all) failed: %d", errno);
	}

	const int32_t loopback = loopback_enable ? 1 : 0;

	if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
		PX4_WARN("CAN_RAW_LOOPBACK=%d failed: %d", (int)loopback, errno);
	}

#ifdef CONFIG_NET_CAN_CANFD
	/* Accept CAN FD frames as well (NuttX defaults this off). */
	const int32_t fd_frames = 1;

	if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd_frames, sizeof(fd_frames)) < 0) {
		PX4_WARN("CAN_RAW_FD_FRAMES=1 failed: %d", errno);
	}
#endif
}

static int open_can_socket(const char *ifname, bool loopback_enable = false)
{
	const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (fd < 0) {
		PX4_ERR("socket(PF_CAN) failed: %d", errno);
		return -errno;
	}

	struct ifreq ifr_idx {};
	(void)strncpy(ifr_idx.ifr_name, ifname, IFNAMSIZ - 1);

	if (ioctl(fd, SIOCGIFINDEX, &ifr_idx) < 0) {
		const int err = -errno;
		PX4_ERR("SIOCGIFINDEX %s failed: %d", ifname, errno);
		close(fd);
		return err;
	}

	const int ifindex = ifr_idx.ifr_ifindex;

	struct sockaddr_can addr {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifindex;

	if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
		const int err = -errno;
		PX4_ERR("bind %s failed: %d", ifname, errno);
		close(fd);
		return err;
	}

	can_apply_raw_sockopts(fd, loopback_enable);

	const int flags = fcntl(fd, F_GETFL, 0);

	if (flags >= 0) {
		(void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
	}

	return fd;
}

static int can_send_frame(int fd, canid_t id, const uint8_t payload[8])
{
	struct can_frame tx {};
	tx.can_id = id;
	tx.can_dlc = 8;
	(void)memcpy(tx.data, payload, 8);

	const ssize_t nbytes = write(fd, &tx, sizeof(tx));

	if (nbytes < 0 || static_cast<size_t>(nbytes) != sizeof(tx)) {
		PX4_ERR("CAN write failed: nbytes=%ld errno=%d", (long)nbytes, errno);
		return (nbytes < 0) ? -errno : -EIO;
	}

	return 0;
}

static canid_t can_rx_id_payload_key(const struct can_frame &fr)
{
	if (fr.can_id & CAN_ERR_FLAG) {
		return CAN_EFF_MASK + 1; /* never matches test pattern */
	}

	if (fr.can_id & CAN_EFF_FLAG) {
		return fr.can_id & CAN_EFF_MASK;
	}

	return fr.can_id & CAN_SFF_MASK;
}

/* Wait by periodic read(): NuttX poll/notifier path can miss edges; scope sees bus, app may still see no frame. */
static int can_receive_frame(int fd, canid_t expected_id, const uint8_t expected_payload[8], int timeout_ms,
			     bool quiet_timeout = false)
{
	const bool want_ext = (expected_id & CAN_EFF_FLAG) != 0;
	const canid_t want_id = want_ext ? (expected_id & CAN_EFF_MASK) : (expected_id & CAN_SFF_MASK);
	const uint64_t deadline_us = static_cast<uint64_t>(timeout_ms) * 1000ULL;

	for (uint64_t spent_us = 0; spent_us < deadline_us; spent_us += kCanRecvPollSliceUs) {
		struct can_frame rx {};
		const ssize_t nbytes = read(fd, &rx, sizeof(rx));

		if (nbytes == static_cast<ssize_t>(sizeof(rx))) {
			const bool rx_ext = (rx.can_id & CAN_EFF_FLAG) != 0;
			const canid_t rid = can_rx_id_payload_key(rx);

			if (rx_ext == want_ext && rid == want_id && rx.can_dlc == 8 && memcmp(rx.data, expected_payload, 8) == 0) {
				return 0;
			}

			PX4_WARN("CAN skip frame id=0x%lx dlc=%u (want 0x%lx dlc=8)",
				 (unsigned long)rx.can_id, (unsigned)rx.can_dlc, (unsigned long)want_id);
			continue;
		}

		if (nbytes < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_ERR("CAN read failed: errno=%d", errno);
				return -errno;
			}
		} else {
			PX4_ERR("CAN read short: nbytes=%ld", (long)nbytes);
			return -EIO;
		}

		usleep(kCanRecvPollSliceUs);
	}

	if (!quiet_timeout) {
		PX4_ERR("CAN recv timeout %d ms (no matching frame from stack)", timeout_ms);
	}

	return -ETIMEDOUT;
}

static int can_direction_test(const char *tx_if, const char *rx_if, canid_t test_id, const uint8_t payload[8])
{
	const int tx_fd = open_can_socket(tx_if);

	if (tx_fd < 0) {
		return tx_fd;
	}

	const int rx_fd = open_can_socket(rx_if);

	if (rx_fd < 0) {
		close(tx_fd);
		return rx_fd;
	}

	struct can_frame flush {};

	while (read(rx_fd, &flush, sizeof(flush)) > 0) {}

	int rx_ret = -ETIMEDOUT;
	unsigned tries = 0;

	for (tries = 0; tries < kCanDirectionRetries; tries++) {
		const int tx_ret = can_send_frame(tx_fd, test_id, payload);

		if (tx_ret != 0) {
			rx_ret = tx_ret;
			break;
		}

		const bool quiet = (tries + 1U < kCanDirectionRetries);
		rx_ret = can_receive_frame(rx_fd, test_id, payload, kCanRecvTimeoutMs, quiet);

		if (rx_ret == 0) {
			break;
		}

		if (rx_ret != -ETIMEDOUT) {
			break;
		}

		px4_usleep(kCanDirectionRetryGapUs);
	}

	if (rx_ret == -ETIMEDOUT) {
		PX4_ERR("CAN %s->%s: TX retries=%u, no frame in SocketCAN RX", tx_if, rx_if, tries);
	}

	close(tx_fd);
	close(rx_fd);

	return rx_ret;
}

static const char *can_if_by_bus(int bus)
{
	return (bus == 0) ? kCanIf0 : kCanIf1;
}

static void can_log_rx_frame(const struct can_frame &rx)
{
	const canid_t raw = rx.can_id;
	const bool eff = (raw & CAN_EFF_FLAG) != 0;
	const canid_t id_only = eff ? (raw & CAN_EFF_MASK) : (raw & CAN_SFF_MASK);

	PX4_INFO("CAN RX %s id=0x%lx dlc=%u data=%02x %02x %02x %02x %02x %02x %02x %02x",
		 eff ? "ext" : "std",
		 (unsigned long)id_only, (unsigned)rx.can_dlc,
		 rx.data[0], rx.data[1], rx.data[2], rx.data[3],
		 rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
}

#ifdef CONFIG_NET_CAN_CANFD
static void can_log_rx_frame_fd(const struct canfd_frame &rx)
{
	const canid_t raw = rx.can_id;
	const bool eff = (raw & CAN_EFF_FLAG) != 0;
	const canid_t id_only = eff ? (raw & CAN_EFF_MASK) : (raw & CAN_SFF_MASK);

	PX4_INFO("CAN RXFD %s id=0x%lx len=%u data=%02x %02x %02x %02x %02x %02x %02x %02x ...",
		 eff ? "ext" : "std",
		 (unsigned long)id_only, (unsigned)rx.len,
		 rx.data[0], rx.data[1], rx.data[2], rx.data[3],
		 rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
}
#endif

static int can_send_frame_with_deadline(int fd, const struct can_frame &tx, unsigned deadline_ms)
{
	/* Enable TX deadline support (non-standard NuttX sockopt) */
	const int32_t enable_deadline = 1;
	(void)setsockopt(fd, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &enable_deadline, sizeof(enable_deadline));

	struct timeval tv {};
	tv.tv_sec = deadline_ms / 1000U;
	tv.tv_usec = (deadline_ms % 1000U) * 1000U;

	struct iovec iov {};
	iov.iov_base = const_cast<struct can_frame *>(&tx);
	iov.iov_len = sizeof(tx);

	/* Provide deadline in CMSG as required by NuttX (see can_sendmsg.c) */
	alignas(struct cmsghdr) uint8_t cmsgbuf[CMSG_SPACE(sizeof(struct timeval))] {};

	struct msghdr msg {};
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = cmsgbuf;
	msg.msg_controllen = sizeof(cmsgbuf);

	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	cmsg->cmsg_level = SOL_CAN_RAW;
	cmsg->cmsg_type = CAN_RAW_TX_DEADLINE;
	cmsg->cmsg_len = CMSG_LEN(sizeof(struct timeval));
	(void)memcpy(CMSG_DATA(cmsg), &tv, sizeof(tv));

	const ssize_t ns = sendmsg(fd, &msg, 0);

	if (ns < 0 || static_cast<size_t>(ns) != sizeof(tx)) {
		return (ns < 0) ? -errno : -EIO;
	}

	return 0;
}
#endif

int test_can()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
	const uint8_t p1[8] {0x55, 0xAA, 0x12, 0x34, 0x7E, 0x80, 0xDE, 0xAD};
	const uint8_t p2[8] {0xA5, 0x5A, 0x21, 0x43, 0xE7, 0x08, 0xED, 0xDA};

	(void)can_iface_up(kCanIf0);
	(void)can_iface_up(kCanIf1);
	usleep(200 * 1000);
	can_reinit_iface_if_possible(kCanIf0);
	can_reinit_iface_if_possible(kCanIf1);
	usleep(50 * 1000);
	can_log_diagnostics();

	const int ret_1 = can_direction_test(kCanIf0, kCanIf1, kCanIdExt, p1);

	if (ret_1 != 0) {
		HW_TEST_FAIL(kTestName, "CAN1->CAN2 ext failed (%d)", ret_1);
		return ret_1;
	}

	usleep(120 * 1000);

	const int ret_2 = can_direction_test(kCanIf1, kCanIf0, kCanIdExt, p2);

	if (ret_2 != 0) {
		HW_TEST_FAIL(kTestName, "CAN2->CAN1 ext failed (%d)", ret_2);
		return ret_2;
	}

	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
#else
	HW_TEST_FAIL(kTestName, "CAN test unavailable (NuttX CONFIG_NET_CAN + FDCAN1/2 required)");
	return -ENOSYS;
#endif
}

int test_can_ext()
{
	return test_can();
}

int test_can_tx(int bus, unsigned duration_s, unsigned period_ms, bool socket_loopback, bool /*ext_id*/)
{
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
	if (bus != 0 && bus != 1) {
		PX4_ERR("can_tx: bus must be 0 (can0 = FDCAN1 / CAN1) or 1 (can1 = FDCAN2)");
		return -EINVAL;
	}

	if (duration_s < 1 || duration_s > 600) {
		PX4_ERR("can_tx: duration_s use 1..600");
		return -EINVAL;
	}

	if (period_ms < 10 || period_ms > 2000) {
		PX4_ERR("can_tx: period_ms use 10..2000");
		return -EINVAL;
	}

	const char *const ifname = can_if_by_bus(bus);
	(void)can_iface_up(ifname);
	usleep(50 * 1000);
	can_reinit_iface_if_possible(ifname);
	usleep(10 * 1000);

	const int fd = open_can_socket(ifname, socket_loopback);

	if (fd < 0) {
		return fd;
	}

	can_log_diagnostics();

	const hrt_abstime t_end = hrt_absolute_time() + static_cast<uint64_t>(duration_s) * 1000000ULL;
	uint32_t seq = 0;
	unsigned sent = 0;

	const canid_t bench_id = kBenchTxCanIdExt;
	PX4_INFO("CAN TX bench: %s for %us every %ums, ext ID 0x%lx payload PX4T+seq%s",
		 ifname, duration_s, period_ms, (unsigned long)(bench_id & CAN_EFF_MASK),
		 socket_loopback ? " (SOCKET LOOPBACK=1, no bus ACK)" : "");
	PX4_INFO("Set analyzer to classic CAN, arbitration ~1000 kbit/s (see CAN diag above).");
	PX4_INFO("If errno=11 (EAGAIN): bus has no ACK — use normal-mode analyzer, 120R+GND, or: hw_test can_tx %d %u %u 1",
		 bus, duration_s, period_ms);

	while (hrt_absolute_time() < t_end) {
		struct can_frame tx {};
		tx.can_id = bench_id;
		tx.can_dlc = 8;
		tx.data[0] = 'P';
		tx.data[1] = 'X';
		tx.data[2] = '4';
		tx.data[3] = 'T';
		memcpy(&tx.data[4], &seq, sizeof(seq));

		int ret = 0;

		if (socket_loopback) {
			const ssize_t nw = write(fd, &tx, sizeof(tx));
			ret = (nw < 0) ? -errno : ((static_cast<size_t>(nw) != sizeof(tx)) ? -EIO : 0);
		} else {
			/* sendmsg() + TX deadline avoids filling HW TX FIFO when no ACK on the bus */
			ret = can_send_frame_with_deadline(fd, tx, 200);

			if (ret == -EAGAIN) {
				const hrt_abstime retry_end = hrt_absolute_time() + 500000ULL;

				do {
					px4_usleep(10 * 1000);
					ret = can_send_frame_with_deadline(fd, tx, 200);
				} while (ret == -EAGAIN && hrt_absolute_time() < retry_end);
			}
		}

		if (ret != 0) {
			PX4_ERR("can_tx send failed: %d (errno=%d)", ret, errno);

			if (!socket_loopback) {
				PX4_ERR("Hints: analyzer normal mode (not listen-only), GND+120R, 1M; or try hw_test can_tx %d %u %u 1",
					bus, duration_s, period_ms);
			}

			close(fd);
			return ret;
		}

		++seq;
		++sent;
		px4_usleep(static_cast<unsigned>(period_ms) * 1000U);
	}

	close(fd);
	PX4_INFO("CAN TX bench done: %u frames on %s", sent, ifname);
	return 0;
#else
	PX4_ERR("CAN TX unavailable (NuttX CONFIG_NET_CAN required)");
	return -ENOSYS;
#endif
}

int test_can_rx(int bus, unsigned duration_s)
{
#if defined(__PX4_NUTTX) && defined(CONFIG_NET_CAN)
	if (bus != 0 && bus != 1) {
		PX4_ERR("can_rx: bus must be 0 or 1");
		return -EINVAL;
	}

	if (duration_s < 1 || duration_s > 600) {
		PX4_ERR("can_rx: duration_s use 1..600");
		return -EINVAL;
	}

	const char *const ifname = can_if_by_bus(bus);
	(void)can_iface_up(ifname);
	usleep(50 * 1000);
	can_reinit_iface_if_possible(ifname);
	usleep(10 * 1000);

	const int fd = open_can_socket(ifname);

	if (fd < 0) {
		return fd;
	}

	struct can_frame flush {};

	while (read(fd, &flush, sizeof(flush)) > 0) {}

	can_log_diagnostics();

	const hrt_abstime t_end = hrt_absolute_time() + static_cast<uint64_t>(duration_s) * 1000000ULL;
	unsigned n_rx = 0;

	PX4_INFO("CAN RX bench: %s for %us (printing all frames)", ifname, duration_s);

	while (hrt_absolute_time() < t_end) {
		struct canfd_frame rx_any {};
		const ssize_t nr = read(fd, &rx_any, sizeof(rx_any));

		if (nr == static_cast<ssize_t>(sizeof(struct can_frame))) {
			const struct can_frame *rx = reinterpret_cast<const struct can_frame *>(&rx_any);
			can_log_rx_frame(*rx);
			++n_rx;

#ifdef CONFIG_NET_CAN_CANFD
		} else if (nr == static_cast<ssize_t>(sizeof(struct canfd_frame))) {
			can_log_rx_frame_fd(rx_any);
			++n_rx;
#endif
		} else if (nr < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_ERR("can_rx read errno=%d", errno);
				close(fd);
				return -errno;
			}
		} else if (nr > 0) {
			PX4_WARN("can_rx unexpected frame size: %ld", (long)nr);
		}

		px4_usleep(2000);
	}

	close(fd);
	PX4_INFO("CAN RX bench done: %u frames on %s", n_rx, ifname);
	return 0;
#else
	PX4_ERR("CAN RX unavailable (NuttX CONFIG_NET_CAN required)");
	return -ENOSYS;
#endif
}
