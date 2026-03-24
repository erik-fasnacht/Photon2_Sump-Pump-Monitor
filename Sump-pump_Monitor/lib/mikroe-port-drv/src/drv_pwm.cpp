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

#include "drv_pwm.h"

//global variables
static uint32_t frequency;      //global variable for the set frequency   
static uint32_t duty_cycle;     //global variable for the duty cycle     
static uint16_t pin;            //global variable for the selected pin  

//configures structure and default values of PWM
void pwm_configure_default(pwm_config_t *config)
{   
    frequency = 0;              //set frequency to 0
    duty_cycle = 0;             //set duty cycle to OFF
}

//opens PWM driver on selected pin
int8_t pwm_open(pwm_t *obj, pwm_config_t *config)
{  
    pin = config->pin;                  //set global variable to parameter
    pinMode(config->pin, OUTPUT);       //added as a check in case default function isn't called before
    analogWrite(pin, LOW);              //set pin to analog write, LOW = 0 (always OFF)
    return PWM_SUCCESS;                 //return status
}

//Set PWM frequency in Hertz
int8_t pwm_set_freq(pwm_t *obj, uint32_t freq_hz)
{
    uint32_t maxFreq;                           //local variable for the max frequency
    maxFreq = analogWriteMaxFrequency(pin);     //returns max frequency for the pin
    if(freq_hz <= maxFreq)                      //check to make sure frequency is less than the max value for a pin
    {
        frequency = freq_hz;                    //set new global variable value to function parameter
        return PWM_SUCCESS;                     //return status
    }
    return PWM_ERROR;                           //return status
}


//Set PWM duty cycle as a percentage
int8_t pwm_set_duty(pwm_t *obj, float duty_ratio)
{
    uint32_t temp = (int)(pow(2,analogWriteResolution(pin)) + 1e-9) - 1;        //set max value for conversion (default is 8-bit, 255)
    if((duty_ratio >= 0) || (duty_ratio <= 1))                                  //check for valid percentage passed
    {
        duty_cycle = duty_ratio * temp;                                         //convert from float % to uint32_t value (for analogWrite() function)   
        return PWM_SUCCESS;                                                     //return status
    }
    return PWM_ERROR;                                                           //return status
}

//start PWM module
int8_t pwm_start(pwm_t *obj)
{
    analogWrite(pin, duty_cycle, frequency);        //set duty cycle and frequency and start PWM
    return PWM_SUCCESS;                             //return status
}

//stop PWM module
int8_t pwm_stop(pwm_t *obj)
{
    analogWrite(pin, LOW);      //set pin to analog write, LOW = 0 (always OFF)
    return PWM_SUCCESS;         //return status
}

//closes PWM driver
void pwm_close(pwm_t *obj)
{ 
    pinMode(pin, INPUT);        //set pin to be input, clear/close PWM
}