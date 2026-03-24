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

#ifndef _DRV_ANALOG_IN_H_
#define _DRV_ANALOG_IN_H_

#include "mikro_port.h"

//enum for status codes, added for library compatibility
typedef enum
{
    ADC_SUCCESS = 0,        //Success
    ADC_ERROR = (-1)        //Error
} analog_in_err_t;

//enum for reference voltage, added for library compatibility
typedef enum
{
    ANALOG_IN_VREF_EXTERNAL = 0,        //External reference
    ANALOG_IN_VREF_INTERNAL = 1         //Internal reference
} analog_in_vref_t;

//enum for the various resolutions, added for library compatibility
typedef enum
{
    ANALOG_IN_RESOLUTION_NOT_SET = 0,                               //Resolution not set
    ANALOG_IN_RESOLUTION_6_BIT = 1,                                 //6-bit resolution
    ANALOG_IN_RESOLUTION_8_BIT = 2,                                 //8-bit resolution
    ANALOG_IN_RESOLUTION_10_BIT = 3,                                //10-bit resolution
    ANALOG_IN_RESOLUTION_12_BIT = 4,                                //12-bit resolution
    ANALOG_IN_RESOLUTION_14_BIT = 5,                                //14-bit resolution
    ANALOG_IN_RESOLUTION_16_BIT = 6,                                //16-bit resolution
    ANALOG_IN_RESOLUTION_DEFAULT = ANALOG_IN_RESOLUTION_12_BIT      //Default resolution
} analog_in_resolution_t;

//struct for various analog in function parameters, added for library compatibility
typedef struct
{
    pin_name_t input_pin;                   //Analog input pin
    analog_in_resolution_t resolution;      //Resolution
    analog_in_vref_t vref_input;            //Voltage reference input
    float vref_value;                       //Voltage reference value
} analog_in_config_t;

//struc for analog in configuration, added for library compatibility
typedef struct
{
    analog_in_config_t config;      //Analog in configuration structure
} analog_in_t;

//Analog In functions
void analog_in_configure_default( analog_in_config_t *config );
int8_t analog_in_open( analog_in_t *obj, analog_in_config_t *config );
int8_t analog_in_set_resolution(analog_in_t *obj, analog_in_resolution_t resolution);
int8_t analog_in_set_vref_value(analog_in_t *obj, float vref_value);
int8_t analog_in_read(analog_in_t *obj, uint16_t *readDatabuf);
int8_t analog_in_read_voltage(analog_in_t *obj, float *readDatabuf);
void analog_in_close(analog_in_t *obj);

#endif // _DRV_ANALOG_IN_H_