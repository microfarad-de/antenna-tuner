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

#ifndef __ADC_H
#define __ADC_H

#include <Arduino.h>
  

/*
 * ADC prescaler values
 */
enum AdcPrescaler_t {
  ADC_PRESCALER_2  = 1,
  ADC_PRESCALER_4  = 2,
  ADC_PRESCALER_8  = 3,
  ADC_PRESCALER_16 = 4,
  ADC_PRESCALER_32 = 5,
  ADC_PRESCALER_64 = 6,
  ADC_PRESCALER_128 = 7
};

/*
 * Analog reference sources
 */
enum AdcReference_t {
  ADC_EXTERNAL = 0,                       /* external input */
  ADC_DEFAULT  = _BV(REFS0),              /* AVcc */
  ADC_INTERNAL = _BV(REFS0) | _BV(REFS1)  /* internal 1.1V */
};

/*
 * ADC class definition
 */
class AdcClass {

  public:
  
    /* 
     *  Initialize the ADC
     *  Parameters:
     *    prescaler : ADC prescaler value
     *    reference : ADC reference voltage
     */
    void initialize (AdcPrescaler_t prescaler = ADC_PRESCALER_128, AdcReference_t reference = ADC_DEFAULT);

    /*
     * Start ADC conversion
     * Parameters:
     *   analogPin : analog pin number (A0..A7 for ATmega328p)
     */
    void start (uint8_t analogPin);

    /*
     * Read ADC result
     * Return value:
     *   -1      : no result available yet
     *   0..1023 : ADC result
     */
    int16_t readVal (void);


  private:  

    bool working = false;
    AdcReference_t reference;
  
};



/*
 * ADC class instantiated as a singleton
 */
extern AdcClass Adc;

#endif // __ADC_H
