/*
 * Helper functions 
 *
 * Karim Hraibi - 2018
 */

#ifndef __HELPER_H
#define __HELPER_H

#include <Arduino.h>



/*
 * Led control class
 */
class LedClass {
  public:
    void initialize (uint8_t ledPin);
    void loopHandler (void);
    void turnOn (void);
    void turnOff (void);
    void blink (void);

    bool blinking = false;
    bool active = false;
    
  private:
    bool initialized = false;
    uint8_t ledPin;
    uint8_t index = 0;
    uint32_t blinkTs = 0;
    int32_t *pattern = pattern1;
    //int32_t pattern1[9] = { 100, 50, 100, 50, 100, 50, 100, 50, -1 };  
    int32_t pattern1[5] = { 100, 50, 100, 50, -1};
};


/*
 * Push button implementation class
 */
class ButtonClass {
  public:
    void press (void);
    void release (void);
    bool rising (void);
    bool falling (void);
    bool fallingLongPress (void);
    bool fallingContinuous (void);
    bool longPress (void); 
    bool longPressContinuous (void); 

    bool pressed = false;
    bool wasPressed = false;
    
  private:
    bool longPressed = false;
    bool wasLongPressed = false;
    uint32_t longPressTs = 0;
};


/*
 * FIR Filter class
 */
class FirFilterClass {
  public:
    void initialize (int16_t *memory, uint16_t size);
    int16_t process (int16_t input);
  private:
    bool initialized = false;
    int16_t *memory;
    uint16_t size;
    int16_t index;
};


/*
 * IIR Filter class
 */
class IirFilterClass {
  public:
    int16_t process (int16_t input, uint16_t size);
  private:
    int32_t output = 0;
};


/*
 * Write an array to EEPROM
 */
void eepromWrite (uint16_t addr, uint8_t *buf, uint16_t bufSize); 

/*
 * Read an array from EEPROM
 */
void eepromRead (uint16_t addr, uint8_t *buf, uint16_t bufSize);


/*
 * Return the sign of a value
 */
int8_t sgn (int val);



#endif // __HELPER_H
