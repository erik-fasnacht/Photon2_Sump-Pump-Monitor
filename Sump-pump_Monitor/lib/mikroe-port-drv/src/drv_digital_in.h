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

#ifndef _DRV_DIGITAL_IN_H_
#define _DRV_DIGITAL_IN_H_

#include "mikro_port.h"

//enum for status codes, added for library compatibility
typedef enum
{
    DIGITAL_IN_SUCCESS = 0,                 //Success
    DIGITAL_IN_UNSUPPORTED_PIN = (-1)       //Error
} digital_in_err_t;

//struc for the various mikroBUS pins, added for library port compatibility
typedef struct
{
    uint8_t pin;     //structure defining pin base and mask
} digital_in_t;

//digital out functions
int8_t digital_in_init(digital_in_t *in, uint8_t mode);
int32_t digital_in_read(digital_in_t *in);

#endif // _DRV_DIGITAL_IN_H_