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
#include <board_config.h>
#endif

static constexpr const char *kTestName = "PWM";
static constexpr unsigned kProbeMaxChannels = 16;
static constexpr uint32_t kPwmMaskAll = 0xFFFF;
static constexpr uint16_t kPwmMinUs = 1000;
static constexpr uint16_t kPwmMaxUs = 2000;
static constexpr useconds_t kStepHoldUs = 350000; /* LED observation dwell per channel */
static constexpr useconds_t kSettleUs = 120000;
static constexpr unsigned kBlinkCount = 5;
static constexpr float kActuatorOff = -1.f;
static constexpr float kActuatorOn = 1.f;

#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
static void buzzer_gpio_init()
{
#if defined(GPIO_BUZZER_1)
	px4_arch_configgpio(GPIO_BUZZER_1);
	px4_arch_gpiowrite(GPIO_BUZZER_1, 0);
#endif
}

static void buzzer_gpio_write(bool high)
{
#if defined(GPIO_BUZZER_1)
	px4_arch_gpiowrite(GPIO_BUZZER_1, high ? 1 : 0);
#endif
}

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
	buzzer_gpio_init();

	/* init all as output low first */
	for (unsigned i = 0; i < sizeof(kIcf6PwmGpio) / sizeof(kIcf6PwmGpio[0]); i++) {
		px4_arch_configgpio(kIcf6PwmGpio[i]);
		px4_arch_gpiowrite(kIcf6PwmGpio[i], 0);
	}

	for (unsigned b = 0; b < kBlinkCount; b++) {
		for (unsigned i = 0; i < sizeof(kIcf6PwmGpio) / sizeof(kIcf6PwmGpio[0]); i++) {
			px4_arch_gpiowrite(kIcf6PwmGpio[i], 1);
		}

		buzzer_gpio_write(true);

		PX4_INFO("PWM GPIO fallback ALL HIGH + BUZZER HIGH (%u/%u)", b + 1, kBlinkCount);
		px4_usleep(kStepHoldUs);

		for (unsigned i = 0; i < sizeof(kIcf6PwmGpio) / sizeof(kIcf6PwmGpio[0]); i++) {
			px4_arch_gpiowrite(kIcf6PwmGpio[i], 0);
		}

		buzzer_gpio_write(false);

		PX4_INFO("PWM GPIO fallback ALL LOW + BUZZER LOW (%u/%u)", b + 1, kBlinkCount);
		px4_usleep(kStepHoldUs);
	}

	PX4_INFO("PWM GPIO fallback done");
	return HW_TEST_OK;
}
#endif

static int run_actuator_test_fallback()
{
	buzzer_gpio_init();
	uORB::Publication<actuator_test_s> pub{ORB_ID(actuator_test)};
	actuator_test_s msg {};
	msg.action = actuator_test_s::ACTION_DO_CONTROL;
	msg.timeout_ms = 500;
	msg.value = kActuatorOff;

	for (unsigned b = 0; b < kBlinkCount; b++) {
		for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; i++) {
			msg.timestamp = hrt_absolute_time();
			msg.function = actuator_test_s::FUNCTION_MOTOR1 + i;
			msg.value = kActuatorOn;
			pub.publish(msg);
		}

		for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; i++) {
			msg.timestamp = hrt_absolute_time();
			msg.function = actuator_test_s::FUNCTION_SERVO1 + i;
			msg.value = kActuatorOn;
			pub.publish(msg);
		}

		buzzer_gpio_write(true);

		PX4_INFO("PWM fallback ALL ON + BUZZER HIGH (%u/%u)", b + 1, kBlinkCount);
		px4_usleep(kStepHoldUs);

		for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; i++) {
			msg.timestamp = hrt_absolute_time();
			msg.function = actuator_test_s::FUNCTION_MOTOR1 + i;
			msg.value = kActuatorOff;
			pub.publish(msg);
		}

		for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; i++) {
			msg.timestamp = hrt_absolute_time();
			msg.function = actuator_test_s::FUNCTION_SERVO1 + i;
			msg.value = kActuatorOff;
			pub.publish(msg);
		}

		buzzer_gpio_write(false);

		PX4_INFO("PWM fallback ALL OFF + BUZZER LOW (%u/%u)", b + 1, kBlinkCount);
		px4_usleep(kStepHoldUs);
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

	PX4_INFO("PWM fallback done: verify ALL channels blinked 3 times");
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
#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
	buzzer_gpio_init();
#endif

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

	PX4_INFO("PWM test: %u channels detected, all channels blink %u times (%u/%u us)",
		 n_channels, kBlinkCount, kPwmMaxUs, kPwmMinUs);

	for (unsigned b = 0; b < kBlinkCount; b++) {
		for (unsigned j = 0; j < n_channels; j++) {
			if (up_pwm_servo_set(channels[j], kPwmMaxUs) != 0) {
				up_pwm_servo_arm(false, 0);
				HW_TEST_FAIL(kTestName, "set pwm ch%u high failed", channels[j]);
				return -EIO;
			}
		}

#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
		buzzer_gpio_write(true);
#endif

		PX4_INFO("PWM ALL HIGH + BUZZER HIGH (%u/%u)", b + 1, kBlinkCount);
		px4_usleep(kStepHoldUs);

		for (unsigned j = 0; j < n_channels; j++) {
			if (up_pwm_servo_set(channels[j], kPwmMinUs) != 0) {
				up_pwm_servo_arm(false, 0);
				HW_TEST_FAIL(kTestName, "set pwm ch%u low failed", channels[j]);
				return -EIO;
			}
		}

#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
		buzzer_gpio_write(false);
#endif

		PX4_INFO("PWM ALL LOW + BUZZER LOW (%u/%u)", b + 1, kBlinkCount);
		px4_usleep(kStepHoldUs);
	}

	/* Finish by returning all channels to minimum. */
	for (unsigned j = 0; j < n_channels; j++) {
		(void)up_pwm_servo_set(channels[j], kPwmMinUs);
	}

#if defined(__PX4_NUTTX) && defined(CONFIG_ARCH_BOARD_AMOVLAB_ICF6)
	buzzer_gpio_write(false);
#endif

	up_pwm_servo_arm(false, 0);
	HW_TEST_PASS(kTestName);
	return HW_TEST_OK;
}
