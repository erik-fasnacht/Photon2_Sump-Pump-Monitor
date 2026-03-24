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

#include "drv_analog_in.h"

//global variables
static uint8_t global_resolution;      //variable for the resolution
static uint16_t global_pin;             //variable for the pin

//configure analog in default
void analog_in_configure_default(analog_in_config_t *config)
{
   global_resolution = ANALOG_IN_RESOLUTION_12_BIT;        //set global variable to 12-bit
}

//open analog input on pin
int8_t analog_in_open(analog_in_t *obj, analog_in_config_t *config)
{
   global_pin = config->input_pin;      //set global pin variable to function parameter
   pinMode(global_pin, AN_INPUT);       //set pin as analog input
   return ADC_SUCCESS;                  //return status
}

//set analog resolution
int8_t analog_in_set_resolution(analog_in_t *obj, analog_in_resolution_t resolution)
{
    switch (resolution)
    {
        case ANALOG_IN_RESOLUTION_6_BIT:
        case ANALOG_IN_RESOLUTION_8_BIT:
        case ANALOG_IN_RESOLUTION_10_BIT:
        case ANALOG_IN_RESOLUTION_12_BIT:
                global_resolution = resolution;     //set global variable to function parameter
            return ADC_SUCCESS;                     //return status

        case ANALOG_IN_RESOLUTION_14_BIT:
        case ANALOG_IN_RESOLUTION_16_BIT:
                #warning this resolution is not supported as the max resolution is 12-bit
                global_resolution = resolution;     //set global variable to function parameter
            return ADC_SUCCESS;                     //return status

         default:
            return ADC_ERROR;      //return status
    }
}

//set vref input, not implemented
int8_t analog_in_set_vref_input(analog_in_t *obj, analog_in_vref_t vref)
{
   #warning analog_in_set_vref_input() function is not implemented
   return ADC_SUCCESS;      //return status
}

//set vref value, not implemented
int8_t analog_in_set_vref_value(analog_in_t *obj, float vref_value)
{
    #warning analog_in_set_vref_value() function is not implemented
    return ADC_SUCCESS;      //return status
}

//read analog input
int8_t analog_in_read(analog_in_t *obj, uint16_t *readDatabuf)
{
    uint16_t val = analogRead(global_pin);        //analog read from set pin
    switch (global_resolution)
    {
        case ANALOG_IN_RESOLUTION_6_BIT:
                val = map(val, 0, 4095, 0, 63);                         //change from 12-bit to 6-bit value
                readDatabuf = reinterpret_cast<uint16_t *>(val);        //set value to function parameter
            return ADC_SUCCESS;                                         //return status

        case ANALOG_IN_RESOLUTION_8_BIT:
                val = map(val, 0, 4095, 0, 255);                        //change from 12-bit to 8-bit value
                readDatabuf = reinterpret_cast<uint16_t *>(val);        //set value to function parameter
            return ADC_SUCCESS;                                         //return status

        case ANALOG_IN_RESOLUTION_10_BIT:
                val = map(val, 0, 4095, 0, 1023);                       //change from 12-bit to 10-bit value
                readDatabuf = reinterpret_cast<uint16_t *>(val);        //set value to function parameter
            return ADC_SUCCESS;                                         //return status

        case ANALOG_IN_RESOLUTION_12_BIT:
                readDatabuf = reinterpret_cast<uint16_t *>(val);        //set value to function parameter
            return ADC_SUCCESS;                                         //return status

        case ANALOG_IN_RESOLUTION_14_BIT:
                #warning this resolution is not supported as the max resolution is 12-bit, output may not be valid 
                val = map(val, 0, 4095, 0, 16383);                      //change from 12-bit to 14-bit value
                readDatabuf = reinterpret_cast<uint16_t *>(val);        //set value to function parameter
            return ADC_SUCCESS;                                         //return status

        case ANALOG_IN_RESOLUTION_16_BIT:
                #warning this resolution is not supported as the max resolution is 12-bit, output may not be valid
                val = map(val, 0, 4095, 0, 65535);                      //change from 12-bit to 16-bit value
                readDatabuf = reinterpret_cast<uint16_t *>(val);        //set value to function parameter
            return ADC_SUCCESS;                                         //return status

         default:      
            return ADC_ERROR;                                           //return status
    }
}

//read analog voltage
int8_t analog_in_read_voltage(analog_in_t *obj, float *readDatabuf)
//int8_t analog_in_read_voltage(analog_in_t *obj, float readDatabuf)
{
    int32_t val = analogRead(global_pin);       //analog read from set pin               
    *readDatabuf = (float) (val * 0.0008);      //convert value to voltage and set to function parameter           
    return ADC_SUCCESS;                         //return status
}

//close/clear analog input
void analog_in_close(analog_in_t *obj)
{
   pinMode(global_pin, INPUT);        //set pin to be input, clear/close analog input
}