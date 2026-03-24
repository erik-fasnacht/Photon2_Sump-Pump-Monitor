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

#ifndef _DRV_ONE_WIRE_H_
#define _DRV_ONE_WIRE_H_

#include "mikro_port.h"
#include "OneWire.h"

//#defines for various 1-wire commands
#define ROM_READ 0x33
#define ROM_SKIP 0xCC
#define ROM_MATCH 0x55
#define ROM_SEARCH 0xF0
#define READ_LEGACY 0x0F

//enum for status codes, added for library compatibility
 typedef enum {
    ONE_WIRE_SUCCESS = 0,       //Success
    ONE_WIRE_ERROR = (-1)       //Error
 } one_wire_error_t;

//struc for the 1-wire address, added for library compatibility
typedef struct {
    uint8_t address[8];     //address buffer
} one_wire_rom_address_t;

//struc for 1-wire configuration, added for library compatibility
typedef struct {
    pin_name_t data_pin;        //defines 1-wire pin
    bool state;                 //State of the 1-wire pin. NOTE must not be altered
} one_wire_t;

//1-wire functions
void one_wire_configure_default(one_wire_t *obj);
int8_t one_wire_open(one_wire_t *obj);
int8_t one_wire_reset(one_wire_t *obj);
int8_t one_wire_write_byte(one_wire_t *obj, uint8_t *write_data_buffer, size_t write_data_length);
int8_t one_wire_read_byte(one_wire_t *obj, uint8_t *read_data_buffer, size_t read_data_length);

//1-wire rom functions
int8_t one_wire_skip_rom(one_wire_t *obj);
int8_t one_wire_match_rom(one_wire_t *obj, one_wire_rom_address_t *device_rom_address);
int8_t one_wire_read_rom(one_wire_t *obj, one_wire_rom_address_t *device_rom_address);

//1-wire rom search functions
int8_t one_wire_search_first_device(one_wire_t *obj, one_wire_rom_address_t *one_wire_device_list);
int8_t one_wire_search_next_device(one_wire_t *obj, one_wire_rom_address_t *one_wire_device_list);
uint8_t oneWire_search(uint8_t *newAddr);

#endif // _DRV_ONE_WIRE_H_