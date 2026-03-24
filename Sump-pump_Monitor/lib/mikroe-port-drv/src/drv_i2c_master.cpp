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

#include "drv_i2c_master.h"

static uint32_t set_speed;      //scl speed variable, static as previous value should be retained
static uint8_t addr;            //7-bit cleint address variable, status as previous value should be retained

//Configures i2c_master_config_t struct to default initialization values
void i2c_master_configure_default(i2c_master_config_t *config)
{   
    addr = 0x00;                                    //set client address to 0x00
    set_speed = I2C_MASTER_SPEED_STANDARD;          //set static global variable to new speed value
    Wire.setSpeed(I2C_MASTER_SPEED_STANDARD);       //set speed, must be done before Wire.begin()
}

//Opens the I2C Master driver object on selected pins
int8_t i2c_master_open(i2c_master_t *obj, i2c_master_config_t *config)
{   
    Wire.setSpeed(set_speed);       //set speed, must be done before Wire.begin() and set to whatever the static global variable is set to
    Wire.begin();                   //join i2c bus as host                   
    return I2C_MASTER_SUCCESS;      //return status
}

//set scl speed
int8_t i2c_master_set_speed(i2c_master_t *obj, uint32_t speed)
{    
    set_speed = speed;              //set static global variable to new speed value
    Wire.setSpeed(speed);           //set speed, must be done before Wire.begin()
    return I2C_MASTER_SUCCESS;      //return status
}

//set timeout value, not implemented
int8_t i2c_master_set_timeout(i2c_master_t *obj, uint16_t timeout_pass_count)
{    
    #warning i2c_master_set_timeout() function is not implemented
    return I2C_MASTER_SUCCESS;      //return status
}

//set 7-bit client address
int8_t i2c_master_set_slave_address(i2c_master_t *obj, uint8_t address)
{    
    addr = address;                 //set function parameter to global variable
    return I2C_MASTER_SUCCESS;      //return status
}

//i2c write operation, R/W = 0
int8_t i2c_master_write(i2c_master_t *obj, uint8_t *write_data_buf, size_t len_write_data)
{   
    Wire.beginTransmission(addr);                   //send device address byte using 7-bit client address
    Wire.write(write_data_buf, len_write_data);     //write function parameter data and legnth to i2c bus
    if (Wire.endTransmission(TRUE) == TRUE)         //send stop condition (true)
    {
        return I2C_MASTER_SUCCESS;                  //return status
    }
    return I2C_MASTER_ERROR;                        //return status    
}

//i2c read operation, R/W = 1
int8_t i2c_master_read(i2c_master_t *obj, uint8_t *read_data_buf, size_t len_read_data)
{   
    //local variable 
    uint8_t temp;       //variable for determining error

    //read data from I2C bus
    temp = Wire.requestFrom(addr, len_read_data, true);     //true signals send stop after read
    for(uint8_t ii = 0; ii < len_read_data; ii++)           //unpack pointer function parameter
    {
        read_data_buf[ii] = Wire.read();                    //write pointer function parameter
    }
    
    //logic for successful transfer
    if(temp != 0)                       //requestFrom() returns 0 if timeout occurs
    {
        return I2C_MASTER_SUCCESS;      //return status
    }
    return I2C_MASTER_ERROR;            //return status
}

//i2c write (dummy write) then read operation
int8_t i2c_master_write_then_read(i2c_master_t *obj, uint8_t *write_data_buf, size_t len_write_data, uint8_t *read_data_buf, size_t len_read_data)
{   
    //local variable
    uint8_t temp;     //variable for determining error

    //write
    Wire.beginTransmission(addr);                   //send device address byte using 7-bit client address
    Wire.write(write_data_buf, len_write_data);     //write function parameter data and legnth to i2c bus
    Wire.endTransmission(FALSE);                    //send restart condition (false)

    //read
    temp = Wire.requestFrom(addr, len_read_data, true);     //true signals send stop after read
    for(uint8_t ii = 0; ii < len_read_data; ii++)           //unpack pointer function parameter
    {
        read_data_buf[ii] = Wire.read();                    //write pointer function parameter
    }

    //logic for successful transfer
    if(temp != 0)                       //requestFrom() returns 0 if timeout occurs
    {
        return I2C_MASTER_SUCCESS;      //return status
    }
    return I2C_MASTER_ERROR;            //return status
}

//Closes I2C Master Driver context object
void i2c_master_close(i2c_master_t *obj)
{    
    Wire.end();      //close i2c peripheral
}