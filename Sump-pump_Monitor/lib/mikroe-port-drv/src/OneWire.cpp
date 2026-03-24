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

#include "OneWire.h"

//global variables 
uint16_t _pin;

//sets pin for 1-wire interface
void oneWire_set_pin(uint16_t pin)
{
    _pin = pin;                  //set global variable equal to pin
    pinMode(_pin, INPUT);        //set pin as input, hal calls pin as output during low-level functions
}

//1-wire reset
uint8_t oneWire_reset(void)
{
    uint8_t temp;                       //local variable for the temp wire value
    uint8_t count = 250;                //local variable for the timeout counter before reset

    noInterrupts();                     //turn off interrupts
    HAL_Pin_Mode(_pin, INPUT);          //set pin as input, let signal float
    interrupts();                       //turn on interrupts
    do 
    {
        if (count <= 0)                 //check for counter timeout   
        {
            return FALSE;               //return error
        }
        count--;                        //decrement the count
        delayMicroseconds(1);           //delay       
    }   while (!pinReadFast(_pin));     //wait until wire goes high before reset sequence

    noInterrupts();                     //turn off interrupts
    pinResetFast(_pin);                 //set pin low
    HAL_Pin_Mode(_pin, OUTPUT);         //set pin as output
    interrupts();                       //turn on interrupts
    delayMicroseconds(512);             //delay for tRSTL = 512us

    noInterrupts();                     //turn off interrupts
    HAL_Pin_Mode(_pin, INPUT);          //set pin as input, let signal float
    delayMicroseconds(70);              //delay
    temp =! pinReadFast(_pin);          //read value, test for shorts/interrupts
    interrupts();                       //turn on interrupts

    delayMicroseconds(442);             //delay for remainder of period, tFILL = 512us
    return temp;                        //return wire value
}

//write a bit
void oneWire_write_bit(uint8_t value)
{
    if ((value & 1) == 1)               //write logic 1
    {
        noInterrupts();                 //turn off interrupts
        pinResetFast(_pin);             //set pin low
        HAL_Pin_Mode(_pin, OUTPUT);     //set pin as output
        delayMicroseconds(8);          //dealy for tLOW1

        HAL_Pin_Mode(_pin, INPUT);      //set pin as input, let signal float
        interrupts();                   //turn on interrupts

        delayMicroseconds(52);          //delay for remainder of tSLOT
    } 
    else                                //write logic 0
    {
        noInterrupts();                 //turn off interrupts
        pinResetFast(_pin);             //set pin low
        HAL_Pin_Mode(_pin, OUTPUT);     //set pin as output
        delayMicroseconds(57);          //delay for tLOW0

        HAL_Pin_Mode(_pin, INPUT);      //set pin as input, let signal float
        interrupts();                   //turn on interrupts

        delayMicroseconds(3);           //delay for remainder of tSLOT
    }
}

//read a bit
uint8_t oneWire_read_bit()
{
    uint8_t temp;                   //local variable for the temp bit value

    noInterrupts();                 //turn off interrupts
    pinResetFast(_pin);             //set pin low
    HAL_Pin_Mode(_pin, OUTPUT);     //set pin as output
    delayMicroseconds(8);           //delay for tLOW1

    HAL_Pin_Mode(_pin, INPUT);      //set pin as input, let signal float
    delayMicroseconds(3);           //delay for tDSO
    temp = pinReadFast(_pin);       //read bit value
    interrupts();                   //turn on interrupts

    delayMicroseconds(49);          //delay for remainder of tSLOT

    return temp;                    //return temp bit value
}

//write a byte
void oneWire_write(uint8_t value, bool pulse) 
{
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)        //for loop for bit shifting 
    {
        if((bitMask & value) == TRUE)       //if bit is a logic 1
        {
            oneWire_write_bit(TRUE);        //write logic 1
        }
        else                                //value is a logic 0
        {
            oneWire_write_bit(FALSE);       //write logic 0
        }
    }

    //wire may need to remain high after write, ex write cycle of eeprom 
    if (pulse == TRUE)                  
    {
        noInterrupts();                 //turn off interrupts
        pinSetFast(_pin);               //set pin high
        HAL_Pin_Mode(_pin, OUTPUT);     //set pin as output
        interrupts();                   //turn on interrupts        
    }
}

//read a byte
uint8_t oneWire_read() 
{
    uint8_t temp = 0;                                           //local variable for the various bit values
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)        //for loop for bit shifting
    {
        if (oneWire_read_bit() == TRUE)                         //if bit = logic 1 
        {
            temp |= bitMask;                                    //or bitmask with temp variable
        }
    }
    return temp;                                                //return byte read
}
