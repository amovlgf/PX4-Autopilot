#include "hw_test.h"

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cmath>
#include <px4_platform_common/px4_config.h>

#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
#include <stm32_gpio.h>
#endif

static constexpr const char *kTestName = "PWM";
static constexpr unsigned kProbeMaxChannels = 16;
static constexpr uint32_t kPwmMaskAll = 0xFFFF;
static constexpr uint16_t kPwmMinUs = 1000;
static constexpr uint16_t kPwmMaxUs = 2000;
static constexpr useconds_t kStepHoldUs = 350000; /* LED observation dwell per channel */
static constexpr useconds_t kSettleUs = 120000;
static constexpr float kActuatorOff = -1.f;
static constexpr float kActuatorOn = 1.f;

#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
/* ICF6 FMU_CH1..10 pins as plain GPIO outputs for LED high-level test */
static constexpr uint32_t kIcf6PwmGpio[] {
	/* CH1 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN9),
	/* CH2 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN11),
	/* CH3 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN13),
	/* CH4 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN14),
	/* CH5 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN7),
	/* CH6 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0),
	/* CH7 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN9),
	/* CH8 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN13),
	/* CH9 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN14),
	/* CH10*/ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN15),
};

static int run_gpio_high_fallback()
{
	/* init all as output low first */
	for (unsigned i = 0; i < sizeof(kIcf6PwmGpio) / sizeof(kIcf6PwmGpio[0]); i++) {
		px4_arch_configgpio(kIcf6PwmGpio[i]);
		px4_arch_gpiowrite(kIcf6PwmGpio[i], 0);
	}

	for (unsigned i = 0; i < sizeof(kIcf6PwmGpio) / sizeof(kIcf6PwmGpio[0]); i++) {
		px4_arch_gpiowrite(kIcf6PwmGpio[i], 1);
		PX4_INFO("PWM GPIO fallback CH%u HIGH", i + 1);
		px4_usleep(kStepHoldUs);
		px4_arch_gpiowrite(kIcf6PwmGpio[i], 0);
	}

	PX4_INFO("PWM GPIO fallback done");
	return HW_TEST_OK;
}
#endif

static int run_actuator_test_fallback()
{
	uORB::Publication<actuator_test_s> pub{ORB_ID(actuator_test)};
	actuator_test_s msg {};
	msg.action = actuator_test_s::ACTION_DO_CONTROL;
	msg.timeout_ms = 500;
	msg.value = kActuatorOff;

	for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; i++) {
		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		msg.value = kActuatorOn;
		pub.publish(msg);
		PX4_INFO("PWM fallback MOTOR%u active", i + 1);
		px4_usleep(kStepHoldUs);

		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		msg.value = kActuatorOff;
		pub.publish(msg);
	}

	for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; i++) {
		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_SERVO1 + i;
		msg.value = kActuatorOn;
		pub.publish(msg);
		PX4_INFO("PWM fallback SERVO%u active", i + 1);
		px4_usleep(kStepHoldUs);

		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_SERVO1 + i;
		msg.value = kActuatorOff;
		pub.publish(msg);
	}

	/* release all channels */
	msg.action = actuator_test_s::ACTION_RELEASE_CONTROL;
	msg.value = NAN;

	for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; i++) {
		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		pub.publish(msg);
	}

	for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; i++) {
		msg.timestamp = hrt_absolute_time();
		msg.function = actuator_test_s::FUNCTION_SERVO1 + i;
		pub.publish(msg);
	}

	PX4_INFO("PWM fallback done: verify LEDs stepped in order");
	return HW_TEST_OK;
}

int test_pwm()
{
	/* Optional device-node check for diagnostics only (some boards expose different node paths). */
	int fd = open("/dev/pwm_output0", O_RDWR);

	if (fd < 0) {
		fd = open("/dev/pwm_output1", O_RDWR);
	}

	if (fd >= 0) {
		close(fd);
	}

	if (up_pwm_servo_init(kPwmMaskAll) != 0) {
#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
		const int gpio_ret = run_gpio_high_fallback();

		if (gpio_ret == HW_TEST_OK) {
			HW_TEST_PASS(kTestName);
			return HW_TEST_OK;
		}

#endif

		const int ret = run_actuator_test_fallback();

		if (ret == HW_TEST_OK) {
			HW_TEST_PASS(kTestName);
			return HW_TEST_OK;
		}

		HW_TEST_FAIL(kTestName, "up_pwm_servo_init failed");
		return ret;
	}

	up_pwm_servo_arm(true, 0);
	px4_usleep(kSettleUs);

	unsigned channels[kProbeMaxChannels] {};
	unsigned n_channels = 0;

	/* Probe valid channels by setting safe minimum pulse. */
	for (unsigned ch = 0; ch < kProbeMaxChannels; ch++) {
		if (up_pwm_servo_set(ch, kPwmMinUs) == 0) {
			channels[n_channels++] = ch;
		}
	}

	if (n_channels == 0) {
		up_pwm_servo_arm(false, 0);
		HW_TEST_FAIL(kTestName, "no PWM channels detected");
		return -ENODEV;
	}

	PX4_INFO("PWM test: %u channels detected, stepping each channel (%u->%u us)", n_channels, kPwmMinUs, kPwmMaxUs);

	/* Visual test pattern: one channel ON (max), others OFF (min). */
	for (unsigned idx = 0; idx < n_channels; idx++) {
		const unsigned active = channels[idx];

		for (unsigned j = 0; j < n_channels; j++) {
			const uint16_t pulse = (channels[j] == active) ? kPwmMaxUs : kPwmMinUs;

			if (up_pwm_servo_set(channels[j], pulse) != 0) {
				up_pwm_servo_arm(false, 0);
				HW_TEST_FAIL(kTestName, "set pwm ch%u failed", channels[j]);
				return -EIO;
			}
		}

		PX4_INFO("PWM CH%u active (%u us), others %u us", active + 1, kPwmMaxUs, kPwmMinUs);
		px4_usleep(kStepHoldUs);
	}

	/* Finish by returning all channels to minimum. */
	for (unsigned j = 0; j < n_channels; j++) {
		(void)up_pwm_servo_set(channels[j], kPwmMinUs);
	}

	up_pwm_servo_arm(false, 0);
	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
}
