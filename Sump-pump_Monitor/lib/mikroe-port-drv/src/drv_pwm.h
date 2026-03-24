/*
 * Copyright (c) 2023 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _DRV_PWM_H_
#define _DRV_PWM_H_

#include "mikro_port.h"

//enum for status codes, added for library compatibility
typedef enum
{
    PWM_SUCCESS = 0,        //Success
    PWM_ERROR = (-1)        //Error
} pwm_err_t;

//struct for various PWM function parameters, added for library compatibility
typedef struct
{
    pin_name_t pin;         //PWM pin
    uint32_t freq_hz;       //PWM Frequency in Hz
} pwm_config_t;

//struc for PWM configuration, added for library compatibility
typedef struct
{
    pwm_config_t config;        //PWM driver configuration structure
} pwm_t;

//PWM functions
void pwm_configure_default(pwm_config_t *config);
int8_t pwm_open(pwm_t *obj, pwm_config_t *config);
int8_t pwm_set_freq(pwm_t *obj, uint32_t freq_hz);
int8_t pwm_start(pwm_t *obj);
int8_t pwm_set_duty(pwm_t *obj, float duty_ratio);
int8_t pwm_stop(pwm_t *obj);
void pwm_close(pwm_t *obj);


#endif // _DRV_PWM_H_