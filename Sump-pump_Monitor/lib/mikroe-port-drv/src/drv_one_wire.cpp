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

#include "drv_one_wire.h"

//global variables
unsigned char rom_num[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;

//set defaul config for 1-wire
void one_wire_configure_default(one_wire_t *obj) 
{
   #warning one_wire_configure_default() function does not do anything since there are no global variables that need to be set
}

//open 1-wire on function parameter pin
int8_t one_wire_open( one_wire_t *obj ) 
{
    oneWire_set_pin(obj->data_pin);
    return ONE_WIRE_SUCCESS;     //return status 
}

//1-wire reset
int8_t one_wire_reset(one_wire_t *obj) 
{   if(oneWire_reset() == TRUE)          //function returns a 1 if device is present, 0 otherwise
   {
        return ONE_WIRE_SUCCESS;        //return status
   }
   return ONE_WIRE_ERROR;               //return status
}

//1-wire write operation
int8_t one_wire_write_byte(one_wire_t *obj, uint8_t *write_data_buffer, size_t write_data_length) 
{
    for (uint16_t ii = 0 ; ii < write_data_length ; ii++)       //iterate through buffer array
    {
        oneWire_write(write_data_buffer[ii]);                   //write rom bytes
    }
    return ONE_WIRE_SUCCESS;                                    //return status
}

//1-wire read operation
int8_t one_wire_read_byte(one_wire_t *obj, uint8_t *read_data_buffer, size_t read_data_length) 
{
    for (uint16_t ii = 0 ; ii < read_data_length; ii++)     //iterate through buffer array
    {
        read_data_buffer[ii] = oneWire_read();              //index pointer array and set to byte read
    }
    return ONE_WIRE_SUCCESS;                                //return status
}

//read ROM of connected device
int8_t one_wire_read_rom(one_wire_t *obj, one_wire_rom_address_t *device_rom_address) 
{
    oneWire_write(ROM_MATCH);                                   //wite Choose ROM cmd
    for (uint16_t ii = 0 ; ii < 8 ; ii++)                       //loop for number of bytes (count = 8)
    {
        device_rom_address->address[ii] = oneWire_read();       //index pointer array and set to byte read
    }
    return ONE_WIRE_SUCCESS;                                    //return status
}

//skip read ROM
int8_t one_wire_skip_rom(one_wire_t *obj) 
{
    oneWire_write(ROM_SKIP);        //write Skip ROM cmd
    return ONE_WIRE_SUCCESS;        //return status
}

//MATCH ROM
int8_t one_wire_match_rom(one_wire_t *obj, one_wire_rom_address_t *device_rom_address)
{
    oneWire_write(ROM_MATCH);                               //wite Choose ROM cmd
    for (uint16_t ii = 0 ; ii < 8 ; ii++)                   //loop for number of bytes (count)
    {
        oneWire_write(device_rom_address->address[ii]);     //write rom bytes
    }
    return ONE_WIRE_SUCCESS;                                //return status
}

//search for first device on 1-wire bus
int8_t one_wire_search_first_device(one_wire_t *obj, one_wire_rom_address_t *one_wire_device_list) 
{
    //reset the search state
    LastDiscrepancy = 0;
    LastDeviceFlag = FALSE;
    LastFamilyDiscrepancy = 0;

    //zero out the array for the ROM address
    for(uint8_t ii = 0; ii < 8; ii++) 
    {
        rom_num[ii] = 0x00;
    }

    if (oneWire_search(one_wire_device_list->address) == TRUE)      //one wire ROM search
    {
        return ONE_WIRE_SUCCESS;                                    //return status
    }   
    return ONE_WIRE_ERROR;                                          //return status
}

//search for next device on 1-wire bus
int8_t one_wire_search_next_device(one_wire_t *obj, one_wire_rom_address_t *one_wire_device_list) 
{
   if (oneWire_search(one_wire_device_list->address) == TRUE)       //one wire ROM search
    {
        return ONE_WIRE_SUCCESS;                                    //return status
    }
    return ONE_WIRE_ERROR;                                          //return status
}

//search device ROM function
uint8_t oneWire_search(uint8_t *newAddr)
{
    //local variables
    uint8_t id_bit_number = 1;
    uint8_t last_zero = 0;
    uint8_t count = 0;
    uint8_t search_result = 0;
    uint8_t id_bit; 
    uint8_t cmp_id_bit;
    unsigned char rom_byte_mask = 1;
    unsigned char search_direction = 0;

    //if the last call was not the last one
    if (!LastDeviceFlag)
    {  
        if (!oneWire_reset())       //1-Wire reset
        {
            //reset the search global variables
            LastDiscrepancy = 0;
            LastDeviceFlag = FALSE;
            LastFamilyDiscrepancy = 0;
            return FALSE;       //return status
        }

        oneWire_write(ROM_SEARCH);      //issue the search command

        //loop to do the search
        do
        {
            id_bit = oneWire_read_bit();            //read a bit
            cmp_id_bit = oneWire_read_bit();        //read its complement

            if ((id_bit == 1) && (cmp_id_bit == 1))     //check for no devices on 1-wire
            {
                break;                                  //break from while loop since no devies are present
            }
            
            else        //device present on the bus
            {        
                if (id_bit != cmp_id_bit)           //all devices coupled have 0 or 1
                {
                    search_direction = id_bit;      //bit write value for search
                }
                else
                {
                    if (id_bit_number < LastDiscrepancy)                                //if this discepancy is before the previous
                    {
                        search_direction = ((rom_num[count] & rom_byte_mask) > 0);      //pick same as last time
                    }
                    else                                                                //discepancy is after the previous                        
                    { 
                        search_direction = (id_bit_number == LastDiscrepancy);          //if equal to last pick 1, if not then pick 0
                    }

                    if (search_direction == 0)                      //if 0 was picked
                    {
                        last_zero = id_bit_number;                  //record position in last_zero
                        if (last_zero < 9)                          //check for Last discrepancy in family
                        {
                            LastFamilyDiscrepancy = last_zero;      //set last discrepancy flag 
                        }
                    }
                }

                //set or clear the bit in the ROM byte count
                if (search_direction == 1)              //if 1 was picked 
                {
                  rom_num[count] |= rom_byte_mask;      //set the bit 
                }
                else                                    //0 was picked
                {
                  rom_num[count] &= ~rom_byte_mask;     //clear the bit
                }

                //write search bit value
                oneWire_write_bit(search_direction);        //serial number search direction write bit
                id_bit_number++;                            //increment byte counter
                rom_byte_mask <<= 1;                        //sift maks to the left by 1          
                if (rom_byte_mask == 0)                     //if the mask is 0
                {
                    count++;                                //increment the count
                    rom_byte_mask = 1;                       //reset mask
                }
            }
        }   while(count < 8);                       //loop until through all ROM bytes 0-7
        

        if (!(id_bit_number < 65))              //search was successful
        {
            LastDiscrepancy = last_zero;        //set last discrepancy flag 
            if (LastDiscrepancy == 0)           //check for last device
            {
                LastDeviceFlag = TRUE;          //set last ROM flag
            }
            search_result = TRUE;               //search was successful
        }
    }
 
    if (!search_result || !rom_num[0])      //if no device found then reset counters
    {
        //reset the search global variables
        LastDiscrepancy = 0;
        LastDeviceFlag = FALSE;
        LastFamilyDiscrepancy = 0;

        search_result = FALSE;      //search result is fail
    }

    for (int i = 0; i < 8; i++)     //iterate over 64 bits 
    {
        newAddr[i] = rom_num[i];        //set poitner function parameter to new ROM value
    }  
    return search_result;       //return search result status
}