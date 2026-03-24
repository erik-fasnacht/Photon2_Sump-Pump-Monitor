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

#ifndef OneWire_h
#define OneWire_h

#include <inttypes.h>
#include "application.h"    //needed for dvos hal level functions

//1-wire low-level functions
  void oneWire_set_pin(uint16_t pin);
  uint8_t oneWire_reset();
  void oneWire_write_bit(uint8_t value);
  uint8_t oneWire_read_bit();
  void oneWire_write(uint8_t value, bool pulse = 0);
  uint8_t oneWire_read();
    
#endif // OneWire_h