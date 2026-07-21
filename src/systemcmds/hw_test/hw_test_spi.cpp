/**
 * hw_test SPI — 测试逻辑与命令备注（现场产测快速参考）
 *
 * 目标
 *   通过 IMU 运行状态 + FRAM 可访问性，验证 SPI1/SPI2 总线连通与稳定性。
 *
 * 当前判定逻辑（test_spi）
 *   1) 在 2s 观察窗口内采集多实例 uORB：
 *      - vehicle_imu（主判定来源，识别 accel/gyro 对应的设备类型）
 *      - sensor_accel / sensor_gyro（用于 error_count 增量判定）
 *   2) 必须识别到四项传感器状态：
 *      - BMI088 accel + gyro
 *      - ICM42688P accel + gyro
 *   3) 四路流都必须“新鲜”：
 *      - 最近更新时间 <= 200 ms（kSpiFreshWindowUs）
 *   4) FRAM 路径可访问：
 *      - open("/fs/mtd_params", O_RDONLY) 成功
 *   5) 四路 error_count 基线必须完整：
 *      - 检查窗口内 error_count 增量必须为 0（任何增长判 FAIL）
 *      - 基线缺失直接判 FAIL
 *
 * 输出含义
 *   - SPI IMU detect:      四项识别状态
 *   - SPI IMU fresh<=...:  四项新鲜度状态
 *   - SPI err_count ...:   first -> last 与 delta
 *   - [PASS] SPI / [FAIL] SPI(...)
 *
 * 常用命令（NSH / QGC MAVLink Console）
 *   # 一键 SPI 产测（推荐）
 *   hw_test spi
 *
 *   # 辅助查看驱动状态（人工复核）
 *   bmi088 -A status
 *   bmi088 -G status
 *   icm42688p status
 *
 * 典型失败解释
 *   - IMU status missing: 未识别到某路 BMI/ICM（驱动未起、device_id 不匹配、总线/连接问题）
 *   - IMU stream stale:   识别到但更新不连续（采样中断、调度堵塞、总线不稳）
 *   - IMU error_count increased: 窗口内错误计数上涨（链路稳定性差）
 *   - FRAM open failed:   参数存储 SPI 路径不可访问
 */

#include "hw_test.h"

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_imu.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>

static constexpr const char *kTestName = "SPI";
static constexpr unsigned kSpiSensorWaitMs = 2000;
static constexpr unsigned kSpiFreshWindowUs = 200000; /* each sensor stream must update within 200 ms */
static constexpr unsigned kMaxAccelSens = 4;
static constexpr unsigned kMaxGyroSens = 4;
static constexpr unsigned kMaxVehicleImu = 4;

static uint8_t device_type_from_id(uint32_t device_id)
{
	/* PX4 device_id layout: [devtype:8][address:8][bus:5 + bus_type:3] */
	const uint8_t devtype = static_cast<uint8_t>((device_id >> 16) & 0xFF);

	/* Fallback for unexpected legacy encodings */
	if (devtype == 0) {
		return static_cast<uint8_t>(device_id & 0xFF);
	}

	return devtype;
}

static bool device_is_on_bus_type(uint32_t device_id, device::Device::DeviceBusType bus_type)
{
	device::Device::DeviceId devid{};
	devid.devid = device_id;
	return devid.devid_s.bus_type == bus_type;
}

struct SpiSensorSeen {
	bool accel_bmi{false};
	bool accel_icm{false};
	bool gyro_bmi{false};
	bool gyro_icm{false};
	hrt_abstime accel_bmi_last{0};
	hrt_abstime accel_icm_last{0};
	hrt_abstime gyro_bmi_last{0};
	hrt_abstime gyro_icm_last{0};
	bool accel_bmi_err_init{false};
	bool accel_icm_err_init{false};
	bool gyro_bmi_err_init{false};
	bool gyro_icm_err_init{false};
	uint32_t accel_bmi_err_first{0};
	uint32_t accel_icm_err_first{0};
	uint32_t gyro_bmi_err_first{0};
	uint32_t gyro_icm_err_first{0};
	uint32_t accel_bmi_err_last{0};
	uint32_t accel_icm_err_last{0};
	uint32_t gyro_bmi_err_last{0};
	uint32_t gyro_icm_err_last{0};
};

static SpiSensorSeen wait_for_spi_sensor_status()
{
	uORB::SubscriptionMultiArray<sensor_accel_s, kMaxAccelSens> accel_subs{ORB_ID::sensor_accel};
	uORB::SubscriptionMultiArray<sensor_gyro_s, kMaxGyroSens> gyro_subs{ORB_ID::sensor_gyro};
	uORB::SubscriptionMultiArray<vehicle_imu_s, kMaxVehicleImu> vimu_subs{ORB_ID::vehicle_imu};
	const hrt_abstime start = hrt_absolute_time();
	SpiSensorSeen seen{};

	while ((hrt_elapsed_time(&start) / 1000) < kSpiSensorWaitMs) {
		for (unsigned i = 0; i < kMaxVehicleImu; i++) {
			vehicle_imu_s msg {};

			while (vimu_subs[i].update(&msg)) {
				const uint8_t acc_dt = device_type_from_id(msg.accel_device_id);
				const uint8_t gyr_dt = device_type_from_id(msg.gyro_device_id);

				if (device_is_on_bus_type(msg.accel_device_id, device::Device::DeviceBusType_SPI)
				    && acc_dt == DRV_ACC_DEVTYPE_BMI088) {
					seen.accel_bmi = true;
					seen.accel_bmi_last = msg.timestamp;

				} else if (device_is_on_bus_type(msg.accel_device_id, device::Device::DeviceBusType_SPI)
					   && acc_dt == DRV_IMU_DEVTYPE_ICM42688P) {
					seen.accel_icm = true;
					seen.accel_icm_last = msg.timestamp;
				}

				if (device_is_on_bus_type(msg.gyro_device_id, device::Device::DeviceBusType_SPI)
				    && gyr_dt == DRV_GYR_DEVTYPE_BMI088) {
					seen.gyro_bmi = true;
					seen.gyro_bmi_last = msg.timestamp;

				} else if (device_is_on_bus_type(msg.gyro_device_id, device::Device::DeviceBusType_SPI)
					   && gyr_dt == DRV_IMU_DEVTYPE_ICM42688P) {
					seen.gyro_icm = true;
					seen.gyro_icm_last = msg.timestamp;
				}
			}
		}

		for (unsigned i = 0; i < kMaxAccelSens; i++) {
			sensor_accel_s msg {};

			while (accel_subs[i].update(&msg)) {
				const uint8_t dt = device_type_from_id(msg.device_id);

				if (device_is_on_bus_type(msg.device_id, device::Device::DeviceBusType_SPI)
				    && dt == DRV_ACC_DEVTYPE_BMI088) {
					seen.accel_bmi = true;
					seen.accel_bmi_last = msg.timestamp;
					seen.accel_bmi_err_last = msg.error_count;

					if (!seen.accel_bmi_err_init) {
						seen.accel_bmi_err_first = msg.error_count;
						seen.accel_bmi_err_init = true;
					}

				} else if (device_is_on_bus_type(msg.device_id, device::Device::DeviceBusType_SPI)
					   && dt == DRV_IMU_DEVTYPE_ICM42688P) {
					seen.accel_icm = true;
					seen.accel_icm_last = msg.timestamp;
					seen.accel_icm_err_last = msg.error_count;

					if (!seen.accel_icm_err_init) {
						seen.accel_icm_err_first = msg.error_count;
						seen.accel_icm_err_init = true;
					}
				}
			}
		}

		for (unsigned i = 0; i < kMaxGyroSens; i++) {
			sensor_gyro_s msg {};

			while (gyro_subs[i].update(&msg)) {
				const uint8_t dt = device_type_from_id(msg.device_id);

				if (device_is_on_bus_type(msg.device_id, device::Device::DeviceBusType_SPI)
				    && dt == DRV_GYR_DEVTYPE_BMI088) {
					seen.gyro_bmi = true;
					seen.gyro_bmi_last = msg.timestamp;
					seen.gyro_bmi_err_last = msg.error_count;

					if (!seen.gyro_bmi_err_init) {
						seen.gyro_bmi_err_first = msg.error_count;
						seen.gyro_bmi_err_init = true;
					}

				} else if (device_is_on_bus_type(msg.device_id, device::Device::DeviceBusType_SPI)
					   && dt == DRV_IMU_DEVTYPE_ICM42688P) {
					seen.gyro_icm = true;
					seen.gyro_icm_last = msg.timestamp;
					seen.gyro_icm_err_last = msg.error_count;

					if (!seen.gyro_icm_err_init) {
						seen.gyro_icm_err_first = msg.error_count;
						seen.gyro_icm_err_init = true;
					}
				}
			}
		}

		px4_usleep(10000);
	}

	return seen;
}

int test_spi()
{
	const SpiSensorSeen seen = wait_for_spi_sensor_status();
	PX4_INFO("SPI IMU detect: BMI(a:%d g:%d) ICM(a:%d g:%d)",
		 seen.accel_bmi, seen.gyro_bmi, seen.accel_icm, seen.gyro_icm);

	const int fram_fd = open("/fs/mtd_params", O_RDONLY);

	if (fram_fd < 0) {
		HW_TEST_FAIL(kTestName, "FRAM open failed: %d", errno);
		return -errno;
	}

	close(fram_fd);

	if (!seen.accel_bmi || !seen.gyro_bmi || !seen.accel_icm || !seen.gyro_icm) {
		HW_TEST_FAIL(kTestName,
			     "IMU status missing (BMI a:%d g:%d, ICM a:%d g:%d)",
			     seen.accel_bmi, seen.gyro_bmi, seen.accel_icm, seen.gyro_icm);
		return -ENODEV;
	}

	/* Streams must still be alive recently to avoid pass on stale one-shot messages. */
	const hrt_abstime now = hrt_absolute_time();
	const bool bmi_accel_fresh = (seen.accel_bmi_last != 0) && ((now - seen.accel_bmi_last) <= kSpiFreshWindowUs);
	const bool bmi_gyro_fresh  = (seen.gyro_bmi_last != 0) && ((now - seen.gyro_bmi_last) <= kSpiFreshWindowUs);
	const bool icm_accel_fresh = (seen.accel_icm_last != 0) && ((now - seen.accel_icm_last) <= kSpiFreshWindowUs);
	const bool icm_gyro_fresh  = (seen.gyro_icm_last != 0) && ((now - seen.gyro_icm_last) <= kSpiFreshWindowUs);
	PX4_INFO("SPI IMU fresh<=%u ms: BMI(a:%d g:%d) ICM(a:%d g:%d)",
		 (unsigned)(kSpiFreshWindowUs / 1000),
		 bmi_accel_fresh, bmi_gyro_fresh, icm_accel_fresh, icm_gyro_fresh);

	if (!bmi_accel_fresh || !bmi_gyro_fresh || !icm_accel_fresh || !icm_gyro_fresh) {
		HW_TEST_FAIL(kTestName,
			     "IMU stream stale (BMI a:%d g:%d, ICM a:%d g:%d; <=%u ms required)",
			     bmi_accel_fresh, bmi_gyro_fresh, icm_accel_fresh, icm_gyro_fresh,
			     (unsigned)(kSpiFreshWindowUs / 1000));
		return -ETIMEDOUT;
	}

	const bool err_count_ready = seen.accel_bmi_err_init && seen.gyro_bmi_err_init && seen.accel_icm_err_init
				     && seen.gyro_icm_err_init;

	if (!err_count_ready) {
		HW_TEST_FAIL(kTestName, "IMU error_count baseline missing");
		return -ENODATA;
	}

	const uint32_t d_bmi_accel_err = seen.accel_bmi_err_last - seen.accel_bmi_err_first;
	const uint32_t d_bmi_gyro_err  = seen.gyro_bmi_err_last - seen.gyro_bmi_err_first;
	const uint32_t d_icm_accel_err = seen.accel_icm_err_last - seen.accel_icm_err_first;
	const uint32_t d_icm_gyro_err  = seen.gyro_icm_err_last - seen.gyro_icm_err_first;

	PX4_INFO("SPI err_count BMI(a:%" PRIu32 "->%" PRIu32 " d=%" PRIu32 ", g:%" PRIu32 "->%" PRIu32 " d=%" PRIu32 ")",
		 seen.accel_bmi_err_first, seen.accel_bmi_err_last, d_bmi_accel_err,
		 seen.gyro_bmi_err_first, seen.gyro_bmi_err_last, d_bmi_gyro_err);
	PX4_INFO("SPI err_count ICM(a:%" PRIu32 "->%" PRIu32 " d=%" PRIu32 ", g:%" PRIu32 "->%" PRIu32 " d=%" PRIu32 ")",
		 seen.accel_icm_err_first, seen.accel_icm_err_last, d_icm_accel_err,
		 seen.gyro_icm_err_first, seen.gyro_icm_err_last, d_icm_gyro_err);

	if ((d_bmi_accel_err > 0) || (d_bmi_gyro_err > 0) || (d_icm_accel_err > 0) || (d_icm_gyro_err > 0)) {
		HW_TEST_FAIL(kTestName,
			     "IMU error_count increased (BMI a:%" PRIu32 " g:%" PRIu32 ", ICM a:%" PRIu32 " g:%" PRIu32 ")",
			     d_bmi_accel_err, d_bmi_gyro_err, d_icm_accel_err, d_icm_gyro_err);
		return -EIO;
	}

	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
}
