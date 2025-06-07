/*******************************************************
* @file    pwm_driver.c
* @author  Nyameaama Gambrah
* @brief   DRV8302 driver file
********************************************************/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include "pwm_driver.h"
#include <stdint.h>

/**
 * @brief DRV8302 logic table
 * 
 * INL_X INH_X GL_X GH_X
 * 0      0     L    L
 * 0      1     L    H
 * 1      0     H    L
 * 1      1     L    L
 */

/**
 * @brief DRV8302 PWM MOSFET drive pins:
 * - {INHA_PIN, INLA_PIN}
 * - {INHB_PIN, INLB_PIN}
 * - {INHC_PIN, INLC_PIN}
 * 
 * @brief 50% duty cycle
 **/

LOG_MODULE_REGISTER(PWM_DRIVER, LOG_LEVEL_INF);

#define MIN_PERIOD (PWM_SEC(1U) / 128U)
#define MAX_PERIOD (PWM_SEC(1U) / 4U)
#define DEFAULT_SLEEP_TIME_MS 1000U  // Default delay

static uint32_t max_period = 0U;

/* Shared variables */
static volatile uint32_t max_duty_cycle = 0U;
static volatile uint32_t sleep_time_ms = DEFAULT_SLEEP_TIME_MS;

/* Mutex for thread safety */
static struct k_mutex motor_speed_mutex;

/* PWM pins specification */
static const struct pwm_dt_spec INHA_PIN = PWM_DT_SPEC_GET(DT_ALIAS(inh_a));
static const struct pwm_dt_spec INLA_PIN = PWM_DT_SPEC_GET(DT_ALIAS(inl_a));
static const struct pwm_dt_spec INHB_PIN = PWM_DT_SPEC_GET(DT_ALIAS(inh_b));
static const struct pwm_dt_spec INLB_PIN = PWM_DT_SPEC_GET(DT_ALIAS(inl_b));
static const struct pwm_dt_spec INHC_PIN = PWM_DT_SPEC_GET(DT_ALIAS(inh_c));
static const struct pwm_dt_spec INLC_PIN = PWM_DT_SPEC_GET(DT_ALIAS(inl_c));

static uint8_t states_without_feedback = 0;

void set_motor_speed(uint32_t speed_percentage) {
    k_mutex_lock(&motor_speed_mutex, K_FOREVER);

    if (speed_percentage > 100U) {
        speed_percentage = 100U;
    }
    // Calculate the duty cycle based on speed_percentage
    max_duty_cycle = max_period * speed_percentage / 100U;
    // Adjust sleep_time_ms inversely to speed_percentage
    sleep_time_ms = DEFAULT_SLEEP_TIME_MS - (speed_percentage * (DEFAULT_SLEEP_TIME_MS - 100U) / 100U);

    k_mutex_unlock(&motor_speed_mutex);
}

void ramp_motor_speed(uint32_t target_speed_percentage, uint32_t ramp_time_ms) {
    if (target_speed_percentage > 100U) {
        target_speed_percentage = 100U;
    }

    uint32_t current_speed_percentage;

    k_mutex_lock(&motor_speed_mutex, K_FOREVER);
    if (max_period != 0U) {
        current_speed_percentage = (max_duty_cycle * 100U) / max_period;
    } else {
        current_speed_percentage = 0U;
    }
    k_mutex_unlock(&motor_speed_mutex);

    int32_t speed_difference = (int32_t)target_speed_percentage - (int32_t)current_speed_percentage;
    int32_t steps = abs(speed_difference);
    if (steps == 0) {
        return; // No need to ramp
    }

    uint32_t step_delay_ms = ramp_time_ms / steps;
    int32_t step_increment = (speed_difference > 0) ? 1 : -1;

    for (int32_t i = 0; i < steps; i++) {
        current_speed_percentage += step_increment;
        set_motor_speed((uint32_t)current_speed_percentage);
        k_msleep(step_delay_ms);
    }
}

void pwm_driver_config(void) {
    /* Check if PWM devices are ready */
    if (!device_is_ready(INHA_PIN.dev)) {
        LOG_ERR("Error: PWM device %s is not ready", INHA_PIN.dev->name);
        return;
    }
    if (!device_is_ready(INHB_PIN.dev)) {
        LOG_ERR("Error: PWM device %s is not ready", INHB_PIN.dev->name);
        return;
    }
    if (!device_is_ready(INHC_PIN.dev)) {
        LOG_ERR("Error: PWM device %s is not ready", INHC_PIN.dev->name);
        return;
    }

    /* Calibrate max_period */
    LOG_INF("Calibrating for channel %d...", INHA_PIN.channel);
    max_period = MAX_PERIOD;
    while (pwm_set_dt(&INHA_PIN, max_period, max_period / 2U)) {
        max_period /= 2U;
        if (max_period < (4U * MIN_PERIOD)) {
            LOG_ERR("Error: PWM device does not support a period at least %u", (unsigned int)(4U * MIN_PERIOD));
            return;
        }
    }

    /* Initialize PWM outputs to zero duty cycle */
    pwm_set_dt(&INHA_PIN, max_period, 0U);
    pwm_set_dt(&INLA_PIN, max_period, 0U);
    pwm_set_dt(&INHB_PIN, max_period, 0U);
    pwm_set_dt(&INLB_PIN, max_period, 0U);
    pwm_set_dt(&INHC_PIN, max_period, 0U);
    pwm_set_dt(&INLC_PIN, max_period, 0U);

    LOG_INF("Done calibrating; maximum/minimum periods %u/%u nsec", (unsigned int)max_period, (unsigned int)MIN_PERIOD);

    /* Set default duty cycle to 50% */
    max_duty_cycle = max_period / 2U;
}

void pwm_driver_start(void) {
    while (1) {
        int ret;
        uint8_t attempts;
        const uint8_t max_attempts = 10;

        k_mutex_lock(&motor_speed_mutex, K_FOREVER);
        uint32_t current_max_duty_cycle = max_duty_cycle;
        uint32_t current_sleep_time_ms = sleep_time_ms;
        k_mutex_unlock(&motor_speed_mutex);

        switch (states_without_feedback) {
            case 0: {
                // State 0: GH_A High, GL_B High, GH_C Low
                attempts = 0;
                do {
                    ret = pwm_set_dt(&INHA_PIN, max_period, current_max_duty_cycle);  // GH_A High
                    ret |= pwm_set_dt(&INLA_PIN, max_period, 0U); // GL_A Low
                    ret |= pwm_set_dt(&INHB_PIN, max_period, 0U); // GH_B Low
                    ret |= pwm_set_dt(&INLB_PIN, max_period, current_max_duty_cycle); // GL_B High
                    ret |= pwm_set_dt(&INHC_PIN, max_period, 0U); // GH_C Low
                    ret |= pwm_set_dt(&INLC_PIN, max_period, 0U); // GL_C Low
                    attempts++;
                } while (ret != 0 && attempts < max_attempts);
                if (ret != 0) {
                    LOG_ERR("PWM Settings Fail - State 0");
                }
                states_without_feedback = 1;
                k_msleep(current_sleep_time_ms);
                break;
            }
            case 1: {
                // State 1: GH_A High, GL_C High, GH_B Low
                attempts = 0;
                do {
                    ret = pwm_set_dt(&INHA_PIN, max_period, current_max_duty_cycle);  // GH_A High
                    ret |= pwm_set_dt(&INLA_PIN, max_period, 0U); // GL_A Low
                    ret |= pwm_set_dt(&INHB_PIN, max_period, 0U); // GH_B Low
                    ret |= pwm_set_dt(&INLB_PIN, max_period, 0U); // GL_B Low
                    ret |= pwm_set_dt(&INHC_PIN, max_period, 0U); // GH_C Low
                    ret |= pwm_set_dt(&INLC_PIN, max_period, current_max_duty_cycle); // GL_C High
                    attempts++;
                } while (ret != 0 && attempts < max_attempts);
                if (ret != 0) {
                    LOG_ERR("PWM Settings Fail - State 1");
                }
                states_without_feedback = 2;
                k_msleep(current_sleep_time_ms);
                break;
            }
            case 2: {
                // State 2: GH_B High, GL_C High, GH_A Low
                attempts = 0;
                do {
                    ret = pwm_set_dt(&INHA_PIN, max_period, 0U);  // GH_A Low
                    ret |= pwm_set_dt(&INLA_PIN, max_period, 0U); // GL_A Low
                    ret |= pwm_set_dt(&INHB_PIN, max_period, current_max_duty_cycle); // GH_B High
                    ret |= pwm_set_dt(&INLB_PIN, max_period, 0U); // GL_B Low
                    ret |= pwm_set_dt(&INHC_PIN, max_period, 0U); // GH_C Low
                    ret |= pwm_set_dt(&INLC_PIN, max_period, current_max_duty_cycle); // GL_C High
                    attempts++;
                } while (ret != 0 && attempts < max_attempts);
                if (ret != 0) {
                    LOG_ERR("PWM Settings Fail - State 2");
                }
                states_without_feedback = 3;
                k_msleep(current_sleep_time_ms);
                break;
            }
            case 3: {
                // State 3: GH_B High, GL_A High, GH_C Low
                attempts = 0;
                do {
                    ret = pwm_set_dt(&INHA_PIN, max_period, 0U);  // GH_A Low
                    ret |= pwm_set_dt(&INLA_PIN, max_period, current_max_duty_cycle); // GL_A High
                    ret |= pwm_set_dt(&INHB_PIN, max_period, current_max_duty_cycle); // GH_B High
                    ret |= pwm_set_dt(&INLB_PIN, max_period, 0U); // GL_B Low
                    ret |= pwm_set_dt(&INHC_PIN, max_period, 0U); // GH_C Low
                    ret |= pwm_set_dt(&INLC_PIN, max_period, 0U); // GL_C Low
                    attempts++;
                } while (ret != 0 && attempts < max_attempts);
                if (ret != 0) {
                    LOG_ERR("PWM Settings Fail - State 3");
                }
                states_without_feedback = 4;
                k_msleep(current_sleep_time_ms);
                break;
            }
            case 4: {
                // State 4: GH_C High, GL_A High, GH_B Low
                attempts = 0;
                do {
                    ret = pwm_set_dt(&INHA_PIN, max_period, 0U);  // GH_A Low
                    ret |= pwm_set_dt(&INLA_PIN, max_period, current_max_duty_cycle); // GL_A High
                    ret |= pwm_set_dt(&INHB_PIN, max_period, 0U); // GH_B Low
                    ret |= pwm_set_dt(&INLB_PIN, max_period, 0U); // GL_B Low
                    ret |= pwm_set_dt(&INHC_PIN, max_period, current_max_duty_cycle); // GH_C High
                    ret |= pwm_set_dt(&INLC_PIN, max_period, 0U); // GL_C Low
                    attempts++;
                } while (ret != 0 && attempts < max_attempts);
                if (ret != 0) {
                    LOG_ERR("PWM Settings Fail - State 4");
                }
                states_without_feedback = 5;
                k_msleep(current_sleep_time_ms);
                break;
            }
            case 5: {
                // State 5: GH_C High, GL_B High, GH_A Low
                attempts = 0;
                do {
                    ret = pwm_set_dt(&INHA_PIN, max_period, 0U);  // GH_A Low
                    ret |= pwm_set_dt(&INLA_PIN, max_period, 0U); // GL_A Low
                    ret |= pwm_set_dt(&INHB_PIN, max_period, 0U); // GH_B Low
                    ret |= pwm_set_dt(&INLB_PIN, max_period, current_max_duty_cycle); // GL_B High
                    ret |= pwm_set_dt(&INHC_PIN, max_period, current_max_duty_cycle); // GH_C High
                    ret |= pwm_set_dt(&INLC_PIN, max_period, 0U); // GL_C Low
                    attempts++;
                } while (ret != 0 && attempts < max_attempts);
                if (ret != 0) {
                    LOG_ERR("PWM Settings Fail - State 5");
                }
                states_without_feedback = 0;
                k_msleep(current_sleep_time_ms);
                break;
            }
            default:
                LOG_ERR("Invalid state: %d", states_without_feedback);
                states_without_feedback = 0;
                break;
        }
    }
}

void pwm_task(void *arg1, void *arg2, void *arg3) {
    k_mutex_init(&motor_speed_mutex);
    pwm_driver_config();

    // Ramp up to 20% speed over 2 seconds (2000 ms)
    ramp_motor_speed(20U, 2000U);

    // Start the PWM driver
    pwm_driver_start();
}
