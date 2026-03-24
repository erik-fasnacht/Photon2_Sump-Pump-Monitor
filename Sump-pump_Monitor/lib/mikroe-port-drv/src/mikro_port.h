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

#ifndef _MIKRO_PINS_H_
#define _MIKRO_PINS_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "Particle.h"

//used in libraries for pin selection
#define HAL_MODULE_ERROR (uint8_t)(0xFFFFFFFF)         //general module error
#define HAL_CHANNEL_ERROR (uint8_t)(0xFFFFFFFF)        //channel error, (timer, ADC, etc.)
#define HAL_PIN_NC (uint8_t)(0xFFFFFFFF)               //pin error, wrong pin selected
#define HAL_PORT_NC (uint8_t)(0xFFFFFFFF)              //port error, wrong port selected

//us delays functions for mikroE delays for libray port compability
inline void Delay_1us( ) {delayMicroseconds(1);}
inline void Delay_10us( ) {delayMicroseconds(10);}
inline void Delay_50us( ) {delayMicroseconds(50);}
inline void Delay_100us( ) {delayMicroseconds(100);}
inline void Delay_1000us( ) {delayMicroseconds(1000);}
inline void Delay_us(unsigned long us) {delayMicroseconds(us);}

//ms delays functions for mikroE delays for libray port compability
inline void Delay_1ms( ) {delay(1);}
inline void Delay_10ms( ) {delay(10);}
inline void Delay_100ms( ) {delay(100);}
inline void Delay_1000ms( ) {delay(1000);}
inline void Delay_ms(unsigned long ms) {delay(ms);}

//sec delays functions for mikroE delays for libray port compability
inline void Delay_1sec( ) {delay(1000);}

//1-wire HAL delays, TBD on whether these are needed
inline void Delay_6us( ) {delayMicroseconds(6);}
inline void Delay_9us( ) {delayMicroseconds(9);}
inline void Delay_22us( ) {delayMicroseconds(22);}
inline void Delay_55us( ) {delayMicroseconds(55);}
inline void Delay_64us( ) {delayMicroseconds(64);}
inline void Delay_70us( ) {delayMicroseconds(70);}
inline void Delay_410us( ) {delayMicroseconds(410);}
inline void Delay_480us( ) {delayMicroseconds(480);}

//mikroBUS pin mapping first lookup
#define MIKROBUS_AN   AN_PIN
#define MIKROBUS_RST  RST_PIN
#define MIKROBUS_CS   CS_PIN
#define MIKROBUS_SCK  SCK_PIN
#define MIKROBUS_MISO MISO_PIN
#define MIKROBUS_MOSI MOSI_PIN
#define MIKROBUS_PWM  PWM_PIN
#define MIKROBUS_INT  INT_PIN
#define MIKROBUS_RX   RX_PIN
#define MIKROBUS_TX   TX_PIN
#define MIKROBUS_SCL  SCL_PIN
#define MIKROBUS_SDA  SDA_PIN

//M.2 format SKUs, same pinout for B404/402, B404X, and B524
#if PLATFORM_ID == PLATFORM_BSOM || PLATFORM_ID == PLATFORM_B5SOM        
    //mikroBUS pin mapping for actual pin lookup using BUS1
    #define MIKROBUS_1          1
    #define MIKROBUS_1_AN_PIN   A1          //D18
    #define MIKROBUS_1_RST_PIN  D7
    #define MIKROBUS_1_CS_PIN   SS          //D8
    #define MIKROBUS_1_SCK_PIN  SCK         //D13
    #define MIKROBUS_1_MISO_PIN MISO        //D11
    #define MIKROBUS_1_MOSI_PIN MOSI        //D12
    #define MIKROBUS_1_PWM_PIN  D5
    #define MIKROBUS_1_INT_PIN  D22
    #define MIKROBUS_1_RX_PIN   RX          //D9
    #define MIKROBUS_1_TX_PIN   TX          //D10
    #define MIKROBUS_1_SCL_PIN  SCL         //D1
    #define MIKROBUS_1_SDA_PIN  SDA         //D0

    //mikroBUS pin mapping for actual pin lookup using BUS2
    #define MIKROBUS_2          2
    #define MIKROBUS_2_AN_PIN   A2          //D17
    #define MIKROBUS_2_RST_PIN  D19
    #define MIKROBUS_2_CS_PIN   D4
    #define MIKROBUS_2_SCK_PIN  SCK         //D13
    #define MIKROBUS_2_MISO_PIN MISO        //D11
    #define MIKROBUS_2_MOSI_PIN MOSI        //D12
    #define MIKROBUS_2_PWM_PIN  D6
    #define MIKROBUS_2_INT_PIN  D23
    #define MIKROBUS_2_RX_PIN   RX          //D9
    #define MIKROBUS_2_TX_PIN   TX          //D10
    #define MIKROBUS_2_SCL_PIN  SCL         //D1
    #define MIKROBUS_2_SDA_PIN  SDA         //D0

//Feather format SKUs, Argon and Boron, same pinout for Argon, BRN404/402, and BRN404X
#elif PLATFORM_ID == PLATFORM_ARGON || PLATFORM_ID == PLATFORM_BORON
     //mikroBUS pin mapping for actual pin lookup using BUS1
    #define MIKROBUS_1          1
    #define MIKROBUS_1_AN_PIN   A0          //D19
    #define MIKROBUS_1_RST_PIN  A2          //D17
    #define MIKROBUS_1_CS_PIN   SS          //A5/D14
    #define MIKROBUS_1_SCK_PIN  SCK         //D17  
    #define MIKROBUS_1_MISO_PIN MISO        //D16
    #define MIKROBUS_1_MOSI_PIN MOSI        //D15
    #define MIKROBUS_1_PWM_PIN  A3          //D16
    #define MIKROBUS_1_INT_PIN  A4          //D15
    #define MIKROBUS_1_RX_PIN   RX          //D9
    #define MIKROBUS_1_TX_PIN   TX          //D8
    #define MIKROBUS_1_SCL_PIN  SCL         //D1
    #define MIKROBUS_1_SDA_PIN  SDA         //D0

    //mikroBUS pin mapping for actual pin lookup using BUS2
    #define MIKROBUS_2          2
    #define MIKROBUS_2_AN_PIN   A1          //D18
    #define MIKROBUS_2_RST_PIN  D7
    #define MIKROBUS_2_CS_PIN   D5
    #define MIKROBUS_2_SCK_PIN  SCK         //D17  
    #define MIKROBUS_2_MISO_PIN MISO        //D16
    #define MIKROBUS_2_MOSI_PIN MOSI        //D15
    #define MIKROBUS_2_PWM_PIN  D8
    #define MIKROBUS_2_INT_PIN  D6
    #define MIKROBUS_2_RX_PIN   RX          //D9
    #define MIKROBUS_2_TX_PIN   TX          //D8
    #define MIKROBUS_2_SCL_PIN  SCL         //D1
    #define MIKROBUS_2_SDA_PIN  SDA         //D0

//Feather format SKUs, Photon 2 pinout
#elif PLATFORM_ID == PLATFORM_P2    
    //mikroBUS pin mapping for actual pin lookup using BUS1
    #define MIKROBUS_1          1
    #define MIKROBUS_1_AN_PIN   A0          //D11
    #define MIKROBUS_1_RST_PIN  A2          //D13
    #define MIKROBUS_1_CS_PIN   SS          //S3/D18
    #define MIKROBUS_1_SCK_PIN  SCK         //D17  
    #define MIKROBUS_1_MISO_PIN MISO        //D16
    #define MIKROBUS_1_MOSI_PIN MOSI        //D15
    #define MIKROBUS_1_PWM_PIN  A5          //D14
    #define MIKROBUS_1_INT_PIN  S4          //D19
    #define MIKROBUS_1_RX_PIN   RX          //D9
    #define MIKROBUS_1_TX_PIN   TX          //D8
    #define MIKROBUS_1_SCL_PIN  SCL         //D1
    #define MIKROBUS_1_SDA_PIN  SDA         //D0

    //mikroBUS pin mapping for actual pin lookup using BUS2
    #define MIKROBUS_2          2
    #define MIKROBUS_2_AN_PIN   A1          //D12
    #define MIKROBUS_2_RST_PIN  D7
    #define MIKROBUS_2_CS_PIN   SS1         //D5
    #define MIKROBUS_2_SCK_PIN  SCK         //D17  
    #define MIKROBUS_2_MISO_PIN MISO        //D16
    #define MIKROBUS_2_MOSI_PIN MOSI        //D15
    #define MIKROBUS_2_PWM_PIN  D10         //warning, not a valid PWM pin
    #define MIKROBUS_2_INT_PIN  D6
    #define MIKROBUS_2_RX_PIN   RX          //D9
    #define MIKROBUS_2_TX_PIN   TX          //D8
    #define MIKROBUS_2_SCL_PIN  SCL         //D1
    #define MIKROBUS_2_SDA_PIN  SDA         //D0

//undefined, need to add pinout for platform
#else
    //INSERT PLATFORM PINOUT HERE

#endif

//used to define which mikroBUS is used
#define MIKROBUS(index, pinout) MIKROBUS_IMPL(index, pinout)
#define MIKROBUS_IMPL(index, pinout) MIKROBUS_##index##_##pinout

//typedef for various parameters passed through libraries, added for library compatibility
typedef uint8_t pin_name_t;             //GPIO pin name
typedef uint8_t port_name_t;            //GPIO port name
typedef uint8_t port_size_t;            //GPIO port size
typedef uint16_t hal_base_addr_t;       //Base address, which is size dependant on the architecture
typedef uint16_t hal_channel_t;         //Channel type, which is size dependant on the architecture
typedef uint16_t hal_pin_name_t;        //Pin type, which is size dependant on the architecture
typedef uint16_t hal_port_name_t;       //Port type, which is size dependant on the architecture
typedef uint16_t hal_port_size_t;       //Port width, which is size dependant on the architecture

#endif // _MIKRO_PINS_H_