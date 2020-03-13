/*
 * Magnetic Loop Antenna Tuner
 * 
 * Karim Hraibi - 2018
 */

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Servo.h>
#include "Helper.h"
#include "ADC.h"


#define SERIAL_DEBUG              // activate debug printing over RS232
#define SERIAL_BAUD 115200          // serial baud rate

// use these macros for printing to serial port
#ifdef SERIAL_DEBUG  
  #define PRINT(...)   Serial.print   (__VA_ARGS__)
  #define PRINTLN(...) Serial.println (__VA_ARGS__)
#else
  #define PRINT(...)
  #define PRINTLN(...)
#endif


#define LED_PIN 13          // output LED pin number (digital pin)
#define PPM_PIN 9           // output pin number for the PPM output (digital pin with PWM support, pin 9 or 10 on ATmega328)
#define MOSFET_PIN 7        // output pin for controlling the power-on MOSFET
#define BUTTON_PWR_PIN 10   // input pin for "power" button
#define BUTTON_INC_PIN 12   // input pin for "increase" button
#define BUTTON_DEC_PIN 11   // input pin for "decrease" button
#define POT_APIN A0         // analog pin connected to the potentiometer

#define SERVO_MIN 500        // minimum servo PPM pulse width (us)
#define SERVO_MAX 2500       // maximum servo PPM pulse width (us)
#define SERVO_FINE_RANGE 100 // range for serve fine adjustment (us)
#define SERVO_SETBACK 60     // setback the servo by this amount when changing direction (us)
#define SERVO_SB_DELAY 60    // delay duration after servo setback (ms)
#define SERVO_DIR_COMP 3     // compensaton value when changing direction (us)

#define BUTTON_DELAY 5       // delay for repeating actions when holding a button (ms)
#define RANGE_ADJ_DELAY  5000 // press the power button for this amount of time to enter the range adjustment mode (ms)

#define FIR_FILTER_TAPS 32   // number of FIR filter taps (for potentiometer reading)
#define IIR_FILTER_TAPS 2    // number of IIR filter taps (for potentiometer reading)

#define AUTO_POWER_OFF_DELAY (10*60000)  // auto power-off timout (ms)


/*
 * Global variables
 */
struct {
  Servo Srv;                                   // Servo object
  LedClass Led;                                // LED object
  ButtonClass ButtonPwr, ButtonInc, ButtonDec; // Push buttons
  FirFilterClass FirFilter;                    // FIR Filter object
  int16_t firMemory [FIR_FILTER_TAPS];         // FIR Filter memory
  int16_t potPosition;                         // Fine tuning potentionmeter position [0..1023]
  uint32_t autoPowerOffTs;                     // timestamp for calculating auto power-off 
  int16_t minPosition;                         // minimum servo postion (us)
  int16_t maxPosition;                         // maximum servo postion (us)
} G;


/*
 * Non-volatile memory contents (EEPROM)
 */
struct {
  int16_t coarsePosition;        // coarse servo position (us)
  int16_t minPosition;           // minimum servo postion (us)
  int16_t maxPosition;           // maximum servo postion (us)
} Nvm;


/*
 * Arduino setup routine
 */
void setup() {
  MCUSR = 0;      // clear MCU status register
  wdt_disable (); // and disable watchdog

#ifdef SERIAL_DEBUG  
  // initialize serial port
  Serial.begin (SERIAL_BAUD);
#endif

  PRINTLN (" ");
  PRINTLN ("+ + +  A N T E N N A  T U N E R  + + +");
  PRINTLN (" ");
 
  pinMode (PPM_PIN, OUTPUT); 
  pinMode (MOSFET_PIN, OUTPUT);
  digitalWrite (MOSFET_PIN, LOW);
  pinMode (BUTTON_PWR_PIN, INPUT_PULLUP);
  pinMode (BUTTON_INC_PIN, INPUT_PULLUP);
  pinMode (BUTTON_DEC_PIN, INPUT_PULLUP);
  //analogReference (INTERNAL);

  // read the non-volatile memory
  eepromRead (0x0, (uint8_t *)&Nvm, sizeof (Nvm));
  if (Nvm.coarsePosition < SERVO_MIN || Nvm.coarsePosition > SERVO_MAX - SERVO_FINE_RANGE) Nvm.coarsePosition = SERVO_MIN;
  if (Nvm.minPosition < SERVO_MIN || Nvm.minPosition > SERVO_MAX) Nvm.minPosition = SERVO_MIN;
  if (Nvm.maxPosition < SERVO_MIN || Nvm.maxPosition > SERVO_MAX) Nvm.maxPosition = SERVO_MAX;
  G.minPosition = Nvm.minPosition;
  G.maxPosition = Nvm.maxPosition;

  // initialize various objects
  ADConv.initialize ();
  G.Led.initialize (LED_PIN);
  G.FirFilter.initialize (G.firMemory, FIR_FILTER_TAPS);

  // intialize the servo object
  G.Srv.attach (PPM_PIN, G.minPosition, G.maxPosition); 
  G.Srv.writeMicroseconds (SERVO_MIN);




  
  // enable the watchdog
  //wdt_enable (WDTO_8S);
}



/*
 * Arduino main loop
 */
void loop() {
  static enum { STARTUP, RUNNING, SHUTDOWN, ADJUST_MIN, ADJUST_MAX } state = STARTUP;
  static uint32_t veryLongPressTs;
  int32_t ts = millis ();

  buttonRead ();
  G.Led.loopHandler ();

  // power button - rising edge
  if ( G.ButtonPwr.rising ()) {
    // do nothing - needed for long press
  }

  // Main state machine
  switch (state) {
    case STARTUP:
      // power button - long press - startup
      if (G.ButtonPwr.longPress ()) {
        G.Led.turnOn ();                 // turn on the led indicator
        digitalWrite (MOSFET_PIN, HIGH); // turn on the MOSFET
        G.autoPowerOffTs = ts;           // reset the auto power-off timer
        state = RUNNING;
      }
      break;

    case RUNNING:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN;
      // auto power-off timeout
      if (ts - G.autoPowerOffTs > AUTO_POWER_OFF_DELAY) state = SHUTDOWN;
      servoControl (false);
      break;

    case ADJUST_MIN:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN;
      // power button - falling - adjust max servo position
      if (G.ButtonPwr.falling ()){
        state = ADJUST_MAX;
        Nvm.coarsePosition = G.maxPosition - SERVO_FINE_RANGE;
        G.minPosition = Nvm.minPosition;
        G.maxPosition = SERVO_MAX;
        break;
      }
      // auto power-off timeout
      if (ts - G.autoPowerOffTs > AUTO_POWER_OFF_DELAY) state = SHUTDOWN;
      Nvm.minPosition = servoControl (true);
      G.Led.blink (1, 1000, 1000);
      break;

    case ADJUST_MAX:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN;
      // power button - falling - adjust min servo position
      if (G.ButtonPwr.falling ()) {
        state = ADJUST_MIN;
        Nvm.coarsePosition = G.minPosition;
        G.minPosition = SERVO_MIN;
        G.maxPosition = Nvm.maxPosition;
        break;
      }
      // auto power-off timeout
      if (ts - G.autoPowerOffTs > AUTO_POWER_OFF_DELAY) state = SHUTDOWN;
      Nvm.maxPosition = servoControl (true);
      G.Led.blink (1, 500, 500);
      break;

    case SHUTDOWN:
      G.Led.turnOff ();                                 // turn off the led indicator
      eepromWrite (0x0, (uint8_t *)&Nvm, sizeof (Nvm)); // write-back NVM settings
      digitalWrite (MOSFET_PIN, LOW);                   // turn off the MOSFET

      veryLongPressTs = millis ();
      while (G.ButtonPwr.pressed) {
        // enter the range adjustment mode if power button was pressed long enough
        if (millis () - veryLongPressTs > RANGE_ADJ_DELAY) {
          state = ADJUST_MIN;
          G.Srv.attach (PPM_PIN, SERVO_MIN, SERVO_MAX); 
          Nvm.coarsePosition = G.minPosition;
          G.minPosition = SERVO_MIN;
          digitalWrite (MOSFET_PIN, HIGH); 
          break;
        }
      }
      if (state == SHUTDOWN) while (1) {};
      break;
    
  }

  // call the power-save routine
  powerSave ();
  
}


/*
 * Read push buttons and potentiometer values
 */
void buttonRead () {
  static IirFilterClass ButtonPwrFilter, ButtonIncFilter, ButtonDecFilter, PotFilter;
  int16_t value, adcVal;

  // read push buttons and digitally debounce
  value = ButtonPwrFilter.process (1023 * digitalRead (BUTTON_PWR_PIN), 8);
  if (value < 512) G.ButtonPwr.press ();
  else G.ButtonPwr.release ();

  value = ButtonIncFilter.process (1023 * digitalRead (BUTTON_INC_PIN), 8);
  if (value < 512) G.ButtonInc.press ();
  else G.ButtonInc.release ();

  value = ButtonDecFilter.process (1023 * digitalRead (BUTTON_DEC_PIN), 8);
  if (value < 512) G.ButtonDec.press ();
  else G.ButtonDec.release ();


  // start ADC for the current button (will be ignored if already started)
  ADConv.start (POT_APIN);

  // check if ADC finished detecting the value
  adcVal = ADConv.readVal ();

  // ADC finished
  if (adcVal >= 0) {
    // smoothen ADC readings
    adcVal = G.FirFilter.process (adcVal);
    G.potPosition = PotFilter.process (adcVal, IIR_FILTER_TAPS);   
  }
  
}


/*
 * Control the servo position
 */
int16_t servoControl (bool adjustRange) {
  static uint32_t buttonTs = 0;
  static int16_t lastServoVal = 0;
  static int16_t lastDelta = 0;
  int16_t servoVal;
  int16_t delta;
  uint32_t ts = millis ();

  if (G.ButtonInc.pressed && ts - buttonTs > BUTTON_DELAY) {
    if (Nvm.coarsePosition < G.maxPosition - SERVO_FINE_RANGE) Nvm.coarsePosition++;
    buttonTs = ts;
  }
  else if (G.ButtonDec.pressed && ts - buttonTs > BUTTON_DELAY) {
    if (Nvm.coarsePosition > G.minPosition) Nvm.coarsePosition--;
    buttonTs = ts;
  }

  // calculate servo position
  servoVal = Nvm.coarsePosition + map (G.potPosition, 0, 1023, 0,  SERVO_FINE_RANGE);
  delta = servoVal - lastServoVal;

  // if PPM value has changed
  if (abs (delta) > 0) {

    // handle changed rotation direction
    if (sgn (delta) == -sgn (lastDelta) && !adjustRange) {
      G.Led.blink (2, 50, 100); // blink twice
      if (delta > 0) {
        G.Srv.writeMicroseconds (lastServoVal - SERVO_SETBACK);
        PRINTLN ("---");
      }
      else  {
        G.Srv.writeMicroseconds (lastServoVal + SERVO_SETBACK);
        PRINTLN ("+++");
      }
      delay (SERVO_SB_DELAY);
    }
    
    if (!adjustRange) G.Led.blink (1, 50, 100); // blink once
    G.autoPowerOffTs = ts;    // reset the auto power-of timer                     
    
    G.Srv.writeMicroseconds (servoVal + sgn(delta) * SERVO_DIR_COMP );

    PRINT ("adcVal = ");
    PRINT (G.potPosition, DEC);
    PRINT ("; servoVal = ");
    PRINT (servoVal, DEC);
    PRINT ("; delta = ");
    PRINTLN (delta, DEC);
    
    lastServoVal = servoVal;
    lastDelta = delta;   
  }
  
  //wdt_reset ();
  return servoVal;
}


/*
 * Power-save routine, enables CPU sleep mode
 */
void powerSave (void) {

  set_sleep_mode (SLEEP_MODE_IDLE); // configure lowest sleep mode that keeps clk_IO for Timer 1

  // enter sleep, wakeup will be triggered by the 
  // next Timer 1 interrupt
  sleep_enable (); 
  sleep_cpu ();
  sleep_disable ();
  
}



/*
 * ISR for servicing Timer 2 interrupts
 */
void timer2ISR (void) {

}


