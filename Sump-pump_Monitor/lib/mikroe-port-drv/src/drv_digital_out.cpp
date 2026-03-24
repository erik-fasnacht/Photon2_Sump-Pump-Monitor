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

#include "drv_digital_out.h"

//function for setting pin as output
int8_t digital_out_init(digital_out_t *out, uint8_t mode)
{
    if (mode == OUTPUT)
    {
        pinMode(out->pin, static_cast<PinMode>(mode));      //static cast mode and set pin as output
        return DIGITAL_OUT_SUCCESS;                         //return status
    }
    else
    {
        return DIGITAL_OUT_UNSUPPORTED_PIN;                 //return status
    }
}

//function for setting pin HIGH
void digital_out_high(digital_out_t *out)
{   
    digitalWrite(out->pin, HIGH);       //write pin to logic 1
}

//function for setting pin LOW
void digital_out_low(digital_out_t *out)
{   
    digitalWrite(out->pin, LOW);        //write pin to logic 0
}

//function for toggling pin
void digital_out_toggle(digital_out_t *out)
{   
    int32_t value = digitalRead(out->pin);      //read current value of pin
    digitalWrite(out->pin, !value);             //write opposite value 
}

//function for writing pin state
void digital_out_write(digital_out_t *out, uint8_t value)
{    
    digitalWrite(out->pin, value);      //write pin state to function parameter value
}