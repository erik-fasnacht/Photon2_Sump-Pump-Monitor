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

#ifndef _DRV_I2C_MASTER_H_
#define _DRV_I2C_MASTER_H_

#include "mikro_port.h"

//enum for status codes, added for library compatibility
typedef enum
{
    I2C_MASTER_SUCCESS = 0,     //Success
    I2C_MASTER_ERROR = (-1)     //Error
} i2c_master_err_t;

//enum for various clock frequencies, added for library compatibility
typedef enum
{
    I2C_MASTER_SPEED_STANDARD = 0,      //standard clock frequency, 100kHz
    I2C_MASTER_SPEED_FULL = 1,          //fast mode clock frequency, 400kHz
    I2C_MASTER_SPEED_FAST = 2           //fast mode Plus clock frequency, 1MHz
} i2c_master_speed_t;

//struct for various I2C function parameters, added for library compatibility
typedef struct
{
    uint8_t addr;                       //client address
    pin_name_t sda;                     //SDA pin
    pin_name_t scl;                     //SCL pin
    uint32_t speed;                     //SCL clock rate
    uint16_t timeout_pass_count;        //timeout value , # of retries
} i2c_master_config_t;

//struc for i2c master configuration, added for library compatibility
typedef struct
{
    i2c_master_config_t config;     //I2C configuration structure
} i2c_master_t;

//i2c functions
void i2c_master_configure_default(i2c_master_config_t *config);
int8_t i2c_master_open(i2c_master_t *obj, i2c_master_config_t *config);
int8_t i2c_master_set_speed(i2c_master_t *obj, uint32_t speed);
int8_t i2c_master_set_timeout(i2c_master_t *obj, uint16_t timeout_pass_count);
int8_t i2c_master_set_slave_address(i2c_master_t *obj, uint8_t address);
int8_t i2c_master_write(i2c_master_t *obj, uint8_t *write_data_buf, size_t len_write_data);
int8_t i2c_master_read(i2c_master_t *obj, uint8_t *read_data_buf, size_t len_read_data);
int8_t i2c_master_write_then_read(i2c_master_t *obj, uint8_t *write_data_buf, size_t len_write_data, uint8_t *read_data_buf, size_t len_read_data);
void i2c_master_close(i2c_master_t *obj);

#endif // _DRV_I2C_MASTER_H_