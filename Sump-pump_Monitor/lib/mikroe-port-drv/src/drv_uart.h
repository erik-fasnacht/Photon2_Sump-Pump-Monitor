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

#ifndef _DRV_UART_H_
#define _DRV_UART_H_

#include "mikro_port.h"

//enum for status codes, added for library compatibility
typedef enum
{
    UART_SUCCESS = 0,       //Success
    UART_ERROR = (-1)       //Error
} uart_err_t;

//enum for various data bits, added for library compatibility
typedef enum
{
    UART_DATA_BITS_7 = 0,                           //7 data bits
    UART_DATA_BITS_8 = 1,                           //8 data bits
    UART_DATA_BITS_9 = 2,                           //9 data bits
    UART_DATA_BITS_DEFAULT = UART_DATA_BITS_8       //Default (8 data bits)
} uart_data_bits_t;

//enum for various parity bits, added for library compatibility
typedef enum
{
    UART_PARITY_NONE = 0,                       //No parity
    UART_PARITY_EVEN = 1,                       //Even parity
    UART_PARITY_ODD = 2,                        //Odd parity
    UART_PARITY_DEFAULT = UART_PARITY_NONE      //Default (no parity)
} uart_parity_t;


 //enum for various stop bits, added for library compatibility
typedef enum
{
    UART_STOP_BITS_HALF = 0,                        //Half stop bit
    UART_STOP_BITS_ONE = 1,                         //One stop bit
    UART_STOP_BITS_ONE_AND_A_HALF = 2,              //One and a half stop bit
    UART_STOP_BITS_TWO = 3,                         //Two stop bits
    UART_STOP_BITS_DEFAULT = UART_STOP_BITS_ONE     //Default (one stop bit)
} uart_stop_bits_t;

//struc for UART configuration, added for library compatibility
typedef struct
{
    pin_name_t tx_pin;              //Tx pin
    pin_name_t rx_pin;              //Rx pin

    uint32_t baud;                  //Baud rate
    uart_data_bits_t data_bits;     //Data bits
    uart_parity_t parity;           //Parity bits
    uart_stop_bits_t stop_bits;     //Stop bits

    size_t tx_ring_size;            //Tx ring size
    size_t rx_ring_size;            //Rx ring size
} uart_config_t;


//struct for various UART function parameters, added for library compatibility
typedef struct
{
    uart_config_t config;           //UART configuration structure

    char *tx_ring_buffer;        //Pointer to Tx ring buffer
    char *rx_ring_buffer;        //Pointer to Rx ring buffer

    bool is_tx_irq_enabled;         //Tx interrupt enabled
    bool is_rx_irq_enabled;         //Rx interrupt enabled

    bool is_blocking;               //boolean Is blocking
} uart_t;

//UART functions
void uart_configure_default(uart_config_t *config);
int8_t uart_open(uart_t *obj, uart_config_t *config);
int8_t uart_set_baud(uart_t *obj, uint32_t baud);
int8_t uart_set_parity(uart_t *obj, uart_parity_t parity);
int8_t uart_set_stop_bits(uart_t *obj, uart_stop_bits_t stop);
int8_t uart_set_data_bits(uart_t *obj, uart_data_bits_t bits);
void uart_set_blocking(uart_t *obj, bool blocking );
int8_t uart_write(uart_t *obj, char *buffer, size_t size);
int8_t uart_print(uart_t *obj, char *text);
int8_t uart_println(uart_t *obj, char *text);
int8_t uart_read(uart_t *obj, char *buffer, size_t size);
size_t uart_bytes_available(uart_t *obj);
void uart_clear(uart_t *obj);
void uart_close(uart_t *obj);

#endif // _DRV_UART_H_