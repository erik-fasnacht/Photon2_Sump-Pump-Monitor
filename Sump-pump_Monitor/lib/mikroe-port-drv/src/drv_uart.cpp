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

#include "drv_uart.h"

//global uart variables
static uint32_t global_data_bits;      //data bits variable
static uint32_t global_parity;         //parity bits variable
static uint32_t global_stop_bits;      //stop bits variable
static uint32_t global_baud;           //baud rate variable


//function for setting uart default configuration
void uart_configure_default(uart_config_t *config)
{   
    //set global values
    global_baud = 115200;                           //set baud rate to default, 115200
    global_data_bits = UART_DATA_BITS_DEFAULT;      //set data bits to default
    global_parity = SERIAL_PARITY_NO;               //set parity bit to default
    global_stop_bits = UART_STOP_BITS_DEFAULT;      //set stop bits to default
}

////function for opening uart port
int8_t uart_open(uart_t *obj, uart_config_t *config)
{
    Serial1.begin(global_baud,(uint32_t)(global_data_bits|global_parity|global_stop_bits));     //open serial port
    return UART_SUCCESS;                                                                        //return status
}

//function for setting uart baud rate
int8_t uart_set_baud(uart_t *obj, uint32_t baud)
{
    switch (baud)
    {
        //valid inputs
        case 1200:
        case 2400:
        case 4800:
        case 9600:
        case 19200:
        case 28800:
        case 38400:
        case 57600:
        case 76800:
        case 115200:
        case 230400:
        case 250000:
        case 460800:
        case 921600:
        case 1000000:
                global_baud = baud;         //set global variable to function parameter
                Serial1.begin(baud);        //set new baud rate
            return UART_SUCCESS;            //return status

        //invalid input
        default:
            return UART_ERROR;              //return status
    }
}

//function for setting uart parity bits
int8_t uart_set_parity(uart_t *obj, uart_parity_t parity)
{
    switch (parity)
    {
        case UART_PARITY_EVEN:
                global_parity = SERIAL_PARITY_EVEN;     //set new parity global variable
            break;

        case UART_PARITY_ODD:
                global_parity = SERIAL_PARITY_ODD;      //set new parity global variable
            break;

        case UART_PARITY_NONE:
                global_parity = SERIAL_PARITY_NO;       //set new parity global variable
            break;

        //invalid input
        default:
            return UART_ERROR;                          //return status
    }

    Serial1.begin(global_baud,(uint32_t)(global_data_bits|global_parity|global_stop_bits));     //set new parity
    return UART_SUCCESS;                                                                        //return status
}

//function for setting uart stop bits
int8_t uart_set_stop_bits(uart_t *obj, uart_stop_bits_t stop)
{
    switch (stop)
    {
        case UART_STOP_BITS_HALF:
                global_stop_bits = SERIAL_STOP_BITS_0_5;        //set new stop bit global variable
            break;

        case UART_STOP_BITS_ONE:
                global_stop_bits = SERIAL_STOP_BITS_1;          //set new stop bit global variable
            break;

        case UART_STOP_BITS_ONE_AND_A_HALF:
                global_stop_bits = SERIAL_STOP_BITS_1_5;        //set new stop bit global variable
            break;

        case UART_STOP_BITS_TWO:
                global_stop_bits = SERIAL_STOP_BITS_2;          //set new stop bit global variable
            break;
        
        //invalid input
        default:
            return UART_ERROR;                                  //return status
    }

    Serial1.begin(global_baud,(uint32_t)(global_data_bits|global_parity|global_stop_bits));     //set new stop bit
    return UART_SUCCESS;                                                                        //return status
}

//function for setting uart data bits
int8_t uart_set_data_bits(uart_t *obj, uart_data_bits_t bits)
{
    switch (bits)
    {
        case UART_DATA_BITS_7:
                global_data_bits = SERIAL_DATA_BITS_7;      //set new data bits global variable
            break;

        case UART_DATA_BITS_8:
                global_data_bits = SERIAL_DATA_BITS_8;      //set new data bits global variable
            break;

        case UART_DATA_BITS_9:
                global_data_bits = SERIAL_DATA_BITS_9;      //set new data bits global variable
            break;

        //invalid input
        default:
            return UART_ERROR;                              //return status
    }

    Serial1.begin(global_baud,(uint32_t)(global_data_bits|global_parity|global_stop_bits));     //set new data bits
    return UART_SUCCESS;                                                                        //return status
}

//function for setting blocking, not implemented
void uart_set_blocking(uart_t *obj, bool blocking)
{
    #warning uart_set_blocking() function is not implemented since uart is always non-blocking
}

//function for uart write
int8_t uart_write(uart_t *obj, char *buffer, size_t size)
{
    //local variables
    uint32_t temp = 0;      //temp variable for # of bytes to be written

    temp = Serial1.write(reinterpret_cast<uint8_t *>(buffer),size);     //write bytes over uart. returns # of bytes written
    if(temp == size)                                                    //value returned equals the write size
    {
        return UART_SUCCESS;                                            //return status
    }
    return UART_ERROR;                                                  //return status
}

//function for uart read
int8_t uart_read(uart_t *obj, uint8_t *buffer, size_t size)
{
    if (Serial1.available() > 0)                    //if uart is available 
    {
        for (uint32_t ii = 0; ii < size; ii++)      //for loop for iterating over array
        {
            buffer[ii] = Serial1.read();            //set pointer parameter to byte read from uart
        }
        return UART_SUCCESS;                        //return status
    }
    return UART_ERROR;                              //return status
}

//function for uart print
int8_t uart_print(uart_t *obj, char *text)
{
    //local varialbes
    size_t temp= sizeof(text);      //temp variable for the size of incoming text

    if(temp == Serial1.print(text))     //check to make sure all bytes were written
    {
        return UART_SUCCESS;            //return status
    }   
    return UART_ERROR;                  //return status 
}

//function for uart print line
int8_t uart_println(uart_t *obj, char *text)
{
    //local varialbes
    size_t temp= sizeof(text);      //temp variable for the size of incoming text

    if(temp == Serial1.println(text))       //check to make sure all bytes were written
    {
        return UART_SUCCESS;                //return status
    }   
    return UART_ERROR;                      //return status
}

//function for returning available bytes to be read
size_t uart_bytes_available(uart_t *obj)
{
    return Serial1.available();     //return number of bytes available for reading
}

//function for clearing/flushing uart buffers
void uart_clear(uart_t *obj)
{
    Serial1.flush();        //flush buffers
}

//function for closing uart peripheral
void uart_close(uart_t *obj)
{
    Serial1.end();      //close uart peripheral
}