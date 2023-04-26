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

#include "drv_digital_in.h"

//function for setting pin as input
int8_t digital_in_init(digital_in_t *in, uint8_t mode)
{
    if (mode == INPUT)
    {
        pinMode(in->pin, static_cast<PinMode>(mode));
        return DIGITAL_IN_SUCCESS;      //return status
    }
    else        //mode other than input selected
    {
        return DIGITAL_IN_UNSUPPORTED_PIN;      //return status
    }
}

//function for reading pin state
int32_t digital_in_read(digital_in_t *in)
{
    return digitalRead(in->pin);
}