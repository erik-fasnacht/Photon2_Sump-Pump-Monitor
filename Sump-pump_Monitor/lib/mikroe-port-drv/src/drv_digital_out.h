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

#ifndef _DRV_DIGITAL_OUT_H_
#define _DRV_DIGITAL_OUT_H_

#include "mikro_port.h"

//enum for status codes, added for library compatibility
typedef enum
{
    DIGITAL_OUT_SUCCESS = 0,                //Success
    DIGITAL_OUT_UNSUPPORTED_PIN = (-1)      //Error
} digital_out_err_t;

//struc for the various mikroBUS pins, added for library port compatibility
typedef struct
{
    uint16_t pin;    //Structure defining pin base and mask
} digital_out_t;

//digital out functions
int8_t digital_out_init(digital_out_t *out, uint8_t mode);
void digital_out_high(digital_out_t *out);
void digital_out_low(digital_out_t *out);
void digital_out_toggle(digital_out_t *out);
void digital_out_write(digital_out_t *out, uint8_t value);

#endif // _DRV_DIGITAL_OUT_H_