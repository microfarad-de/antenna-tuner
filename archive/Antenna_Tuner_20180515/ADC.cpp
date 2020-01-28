/*
 * Abstraction layer for the ATmega328p ADC 
 *
 * Karim Hraibi - 2018
 */


#include "ADC.h"



AdcClass ADConv;


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

