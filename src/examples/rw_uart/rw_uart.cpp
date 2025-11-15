#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <poll.h>

static bool thread_should_exit = false;
static bool thread_running = false;
static int rw_uart_task;

// 函数声明
extern "C" __EXPORT int rw_uart_main(int argc, char *argv[]);
static int uart_init(const char *uart_name);
static int set_uart_baudrate(int fd, unsigned int baud);
static void usage(const char *reason);
int rw_uart_thread_main(int argc, char *argv[]);

static void usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO("usage: rw_uart {start|stop|status}");
}

int set_uart_baudrate(int fd, unsigned int baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		PX4_ERR("Unsupported baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	// 获取当前配置
	if (tcgetattr(fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr failed: %s", strerror(errno));
		return -1;
	}

	// 设置输入输出波特率
	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);

	// 配置为原始模式（RAW mode）
	uart_config.c_cflag |= (CLOCAL | CREAD);  // 本地连接，启用接收
	uart_config.c_cflag &= ~CSIZE;            // 清除数据位掩码
	uart_config.c_cflag |= CS8;               // 8位数据位
	uart_config.c_cflag &= ~PARENB;           // 无奇偶校验
	uart_config.c_cflag &= ~CSTOPB;           // 1位停止位
	uart_config.c_cflag &= ~CRTSCTS;          // 无硬件流控

	// 输入模式配置
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
	uart_config.c_iflag &= ~(INLCR | ICRNL);        // 禁用输入转换

	// 输出模式配置
	uart_config.c_oflag &= ~OPOST;  // 原始输出
	uart_config.c_oflag &= ~ONLCR;  // 防止换行转换

	// 本地模式配置
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// 超时设置：立即返回
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	// 应用配置
	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed: %s", strerror(errno));
		return -1;
	}

	// 清空输入输出缓冲区
	tcflush(fd, TCIOFLUSH);

	return 0;
}

int uart_init(const char *uart_name)
{
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (serial_fd < 0) {
		PX4_ERR("Failed to open serial port %s: %s", uart_name, strerror(errno));
		return -1;
	}

	PX4_INFO("Opened serial port %s with fd: %d", uart_name, serial_fd);
	return serial_fd;
}

int rw_uart_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			PX4_WARN("rw_uart already running");
			return 0;
		}

		thread_should_exit = false;
		rw_uart_task = px4_task_spawn_cmd("rw_uart",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_DEFAULT,
						  2000,
						  rw_uart_thread_main,
						  (argv && argc > 2) ? (char *const *)&argv[2] : (char *const *)NULL);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		PX4_INFO("Stopping rw_uart thread");
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int rw_uart_thread_main(int argc, char *argv[])
{
	int uart_fd = uart_init("/dev/ttyS6");

	if (uart_fd < 0) {
		return -1;
	}

	if (set_uart_baudrate(uart_fd, 57600) != 0) {
		PX4_ERR("Failed to set baudrate");
		close(uart_fd);
		return -1;
	}

	PX4_INFO("UART initialization successful");
	thread_running = true;

	const char *message = "amovlab\n";  // 包含换行符的完整消息

	// 接收缓冲区
	char recv_buffer[256];
	ssize_t recv_len = 0;

	// 设置poll用于非阻塞读取
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	// 发送计数器
	int send_counter = 0;
	const int send_interval_ms = 1000; // 1秒发送间隔

	while (!thread_should_exit) {
		// 检查是否有数据可读（非阻塞）
		int ret = poll(fds, 1, 100); // 100ms超时

		if (ret > 0) {
			if (fds[0].revents & POLLIN) {
				// 有数据可读
				recv_len = read(uart_fd, recv_buffer, sizeof(recv_buffer) - 1);

				if (recv_len > 0) {
					// 成功读取数据
					recv_buffer[recv_len] = '\0'; // 添加字符串结束符

					PX4_INFO("Received %zd bytes: %s", recv_len, recv_buffer);

					// 将接收到的数据发送回去（回显功能）
					ssize_t echo_bytes = write(uart_fd, recv_buffer, recv_len);

					if (echo_bytes < 0) {
						PX4_ERR("Echo write error: %s", strerror(errno));

					} else if (echo_bytes != recv_len) {
						PX4_WARN("Partial echo: %zd/%zd bytes", echo_bytes, recv_len);

					} else {
						PX4_DEBUG("Echoed: %s", recv_buffer);
					}

				} else if (recv_len < 0) {
					// 读取错误
					if (errno != EAGAIN && errno != EWOULDBLOCK) {
						PX4_ERR("Serial read error: %s", strerror(errno));
					}
				}
			}

		} else if (ret < 0) {
			// poll错误
			PX4_ERR("Poll error: %s", strerror(errno));
		}

		// 定期发送消息（每秒一次）
		send_counter += 100; // 每次循环增加100ms

		if (send_counter >= send_interval_ms) {
			ssize_t bytes_written = write(uart_fd, message, strlen(message));

			if (bytes_written < 0) {
				PX4_ERR("Serial write error: %s", strerror(errno));
				break;

			} else if (bytes_written != (ssize_t)strlen(message)) {
				PX4_WARN("Partial write: %zd/%zu bytes", bytes_written, strlen(message));

			} else {
				PX4_DEBUG("Sent: %s", message);
			}

			send_counter = 0; // 重置计数器
		}

		// 短暂休眠，避免过度消耗CPU
		px4_usleep(100000);  // 100ms延迟
	}

	PX4_INFO("rw_uart thread exiting");
	thread_running = false;

	if (close(uart_fd) < 0) {
		PX4_ERR("Failed to close UART: %s", strerror(errno));

	} else {
		PX4_INFO("UART closed successfully");
	}

	return 0;
}
