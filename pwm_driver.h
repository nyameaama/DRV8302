/*******************************************************
* @file    pwm_driver.h
* @author  Nyameaama Gambrah
* @brief   DRV8302 driver file
********************************************************/

#ifndef PWM_DRIVER_H_
#define PWM_DRIVER_H_

#include<stdint.h>
//#include "../../Logging/logger.h"

void set_motor_speed(uint32_t speed_percentage);

void ramp_motor_speed(uint32_t target_speed_percentage, uint32_t ramp_time_ms);

/**
 * 
 * @brief pwm driver configuration
 * 
*/
void pwm_driver_config(void);

/**
 * 
 * @brief pwm driver start
 * 
*/
void pwm_driver_start(void);

void pwm_task(void *arg1, void *arg2, void *arg3);

#endif