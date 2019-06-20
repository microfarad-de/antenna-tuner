/* 
 * Abstraction layer for the ATmega328p ADC 
 * Non-blocking read the ADC output as an alternative
 * to the blocking analogRead() method.
 * 
 * This source file is part of the Magnetic Loop Antenna Tuner Arduino firmware
 * found under http://www.github.com/microfarad-de/antenna-tuner
 * 
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 * 
 * Copyright (C) 2019 Karim Hraibi (khraibi at gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "Adc.h"



AdcClass Adc;


void AdcClass::initialize (AdcPrescaler_t prescaler, AdcReference_t reference) {
  this->reference = reference;
  ADCSRA =  _BV (ADEN);   // turn ADC on
  ADCSRA |= prescaler ;
}


void AdcClass::start (uint8_t analogPin) {
  uint8_t adcPin;
  
  if (working) return;
  
  adcPin = analogPin - 14;
  ADMUX  = reference | (adcPin & 0x07);      // select reference and input port
  bitSet (ADCSRA, ADSC);                     // start a conversion 
  working = true;
}


int16_t AdcClass::readVal (void) {
  int16_t rv;
  
  // the ADC clears the bit when done
  if (bit_is_clear(ADCSRA, ADSC) && working) {
    rv = ADC; // read result
    working = false;
    return rv;
  }
  else {
    return -1;
  }
}
