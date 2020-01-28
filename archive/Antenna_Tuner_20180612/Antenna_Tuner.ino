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


//#define SERIAL_DEBUG              // activate debug printing over RS232
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
#define SERVO_SETBACK_MIN 0  // minimum setback the servo by this amount when changing direction (us)
#define SERVO_SETBACK_MAX 200// maximum setback the servo by this amount when changing direction (us)
#define SERVO_SB_DELAY 60    // delay duration after servo setback (ms)
#define SERVO_COMP_MIN 0     // minimum compensaton value when changing direction (us)
#define SERVO_COMP_MAX 10    // maximum compensaton value when changing direction (us)
#define SERVO_SB_STARTUP 100 // setback the servo by this amount when starting-up (us)

#define BUTTON_DELAY 5       // delay for repeating actions when holding a button (ms)
#define RANGE_ADJ_DELAY  5000 // press the power button for this amount of time to enter the range adjustment mode (ms)

#define FIR_FILTER_TAPS 32   // number of FIR filter taps (for potentiometer reading)
#define IIR_FILTER_TAPS 4    // number of IIR filter taps (for potentiometer reading)

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
  int16_t servoSetback;           // setback the servo by this amount when changing direction (us) 60
  int16_t servoComp;              // compensaton value when changing direction (us) 3
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
  if (Nvm.servoSetback < SERVO_SETBACK_MIN || Nvm.servoSetback > SERVO_SETBACK_MAX) Nvm.servoSetback = SERVO_SETBACK_MIN;
  if (Nvm.servoComp < SERVO_COMP_MIN || Nvm.servoComp > SERVO_COMP_MAX) Nvm.servoComp = SERVO_COMP_MIN;
  G.minPosition = Nvm.minPosition;
  G.maxPosition = Nvm.maxPosition;

  // initialize various objects
  ADConv.initialize ();
  G.Led.initialize (LED_PIN);
  G.FirFilter.initialize (G.firMemory, FIR_FILTER_TAPS);

  // intialize the servo object
  G.Srv.attach (PPM_PIN, G.minPosition, G.maxPosition); 
  if (Nvm.coarsePosition - SERVO_SB_STARTUP > SERVO_MIN)
    G.Srv.writeMicroseconds (Nvm.coarsePosition - SERVO_SB_STARTUP);
  else 
    G.Srv.writeMicroseconds (SERVO_MIN);

  G.autoPowerOffTs = millis();  // reset the auto power-off timer
  
  // enable the watchdog
  //wdt_enable (WDTO_8S);
}



/*
 * Arduino main loop
 */
void loop() {
  static enum { STARTUP, RUNNING_E, RUNNING, SHUTDOWN_E, SHUTDOWN, ADJUST_MIN_E, ADJUST_MIN, 
     ADJUST_MAX_E, ADJUST_MAX, ADJUST_COMP_E, ADJUST_COMP, ADJUST_SETBACK_E, ADJUST_SETBACK } state = STARTUP, lastState = STARTUP;
  static uint32_t veryLongPressTs;
  static int16_t servoVal, lastServoVal;
  int32_t ts = millis ();

  buttonRead ();
  G.Led.loopHandler ();

  // auto power-off timeout
  if (ts - G.autoPowerOffTs > AUTO_POWER_OFF_DELAY) state = SHUTDOWN_E;

  // power button - rising edge
  if ( G.ButtonPwr.rising ()) {
    // do nothing
  }

  // Main state machine
  switch (state) {
    case STARTUP:
      // power button - long press - startup
      if (G.ButtonPwr.longPress ()) state = RUNNING_E;
      break;

    case RUNNING_E:
      G.Led.turnOn ();                 // turn on the led indicator
      digitalWrite (MOSFET_PIN, HIGH); // turn on the power MOSFET
      state = lastState = RUNNING;
    case RUNNING:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN_E;
      servoControl (false, false);
      break;

    case ADJUST_MIN_E:
      G.Led.blinkBlocking (1, 200, 400);
      Nvm.coarsePosition = G.minPosition;
      G.minPosition = SERVO_MIN;
      G.maxPosition = Nvm.maxPosition;
      lastServoVal = servoControl (true, true);
      state = lastState = ADJUST_MIN;
    case ADJUST_MIN:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN_E;
      // power button - falling - adjust max servo position
      if (G.ButtonPwr.falling ()) state = ADJUST_MAX_E;
      servoVal = servoControl (true, false);
      if (servoVal != lastServoVal) {
        Nvm.minPosition = servoVal;
        lastServoVal = servoVal;
      }
      break;
    
    case ADJUST_MAX_E:
      G.Led.blinkBlocking (2, 200, 400);
      Nvm.coarsePosition = G.maxPosition - SERVO_FINE_RANGE;
      G.minPosition = Nvm.minPosition;
      G.maxPosition = SERVO_MAX;
      lastServoVal = servoControl (true, true);
      state = lastState  = ADJUST_MAX;
    case ADJUST_MAX:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN_E;
      // power button - falling - adjust direction compensation value
      if (G.ButtonPwr.falling ()) state = ADJUST_COMP_E;
      servoVal = servoControl (true, false);
      if (servoVal != lastServoVal) {
        Nvm.maxPosition = servoVal;
        lastServoVal = servoVal;
      }
      break;

    case ADJUST_COMP_E:
      G.Led.blinkBlocking (3, 200, 400);
      state = lastState = ADJUST_COMP;
    case ADJUST_COMP:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN_E;
      // power button - falling - adjust setback value
      if (G.ButtonPwr.falling ()) state = ADJUST_SETBACK_E;
      // inc button - rising - increment value
      if (G.ButtonInc.rising ()) {
        if (Nvm.servoComp < SERVO_COMP_MAX) Nvm.servoComp++;
        G.Led.blinkBlocking (Nvm.servoComp, 100, 400);
        G.autoPowerOffTs = ts;
      }
      // dec button - rising - decrement value
      if (G.ButtonDec.rising ()) {
        if (Nvm.servoComp > SERVO_COMP_MIN) Nvm.servoComp--;
        if (Nvm.servoComp > 0 ) G.Led.blinkBlocking (Nvm.servoComp, 100, 300);
        G.autoPowerOffTs = ts;
      }
      break;

    case ADJUST_SETBACK_E:
      G.Led.blinkBlocking (4, 200, 400);
      G.Srv.writeMicroseconds (Nvm.maxPosition);
      delay (SERVO_SB_DELAY);
      Nvm.coarsePosition = Nvm.minPosition;
      servoVal = servoControl (true, true);
      state = lastState = ADJUST_SETBACK;
    case ADJUST_SETBACK:
      // power button - long press - shutdown
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN_E;
      // power button - falling - adjust min servo position
      if (G.ButtonPwr.falling ()) state = ADJUST_MIN_E; 
      // inc button - rising - increment value
      if (G.ButtonInc.rising ()) { 
        Nvm.servoSetback += 5;
        if (Nvm.servoSetback > SERVO_SETBACK_MAX) Nvm.servoSetback = SERVO_SETBACK_MAX;
        else {
          G.Led.blink (1, 100, 100);
          G.Srv.writeMicroseconds (servoVal + Nvm.servoSetback);
          delay (SERVO_SB_DELAY);
          G.Srv.writeMicroseconds (servoVal);
        }
        G.autoPowerOffTs = ts;
      }
      // dec button - rising - decrement value
      if (G.ButtonDec.rising ()) {
        Nvm.servoSetback -= 5;
        if (Nvm.servoSetback < SERVO_SETBACK_MIN) Nvm.servoSetback = SERVO_SETBACK_MIN;
        else {
          G.Led.blink (1, 100, 100);
          G.Srv.writeMicroseconds (servoVal + Nvm.servoSetback);
          delay (SERVO_SB_DELAY);
          G.Srv.writeMicroseconds (servoVal);
        } 
        G.autoPowerOffTs = ts;
      }
      break;

    case SHUTDOWN_E:
      G.Led.toggle (); // toggle LED indicator
      veryLongPressTs = ts;
      state = SHUTDOWN;
    case SHUTDOWN:
      // enter the adjustment mode if power button was pressed long enough
      if ( G.ButtonPwr.pressed && lastState == RUNNING) {
        if (ts - veryLongPressTs > RANGE_ADJ_DELAY) {
          state = ADJUST_MIN_E;
          G.Srv.attach (PPM_PIN, SERVO_MIN, SERVO_MAX); 
          break;
        }
      }
      else {
        eepromWrite (0x0, (uint8_t *)&Nvm, sizeof (Nvm)); // write-back NVM settings
        digitalWrite (MOSFET_PIN, LOW);                   // turn off the power MOSFET
        delay (1000);                                     // wait a seond
        if (G.Led.powerOn) G.Led.turnOff ();              // turn off LED if it was on
        while (1) {};                                     // stay here until power down
      }
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
int16_t servoControl (bool adjustRange, bool noBlink) {
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
      if (!noBlink) G.Led.blink (2, 50, 100); // blink twice
      if (delta > 0) {
        G.Srv.writeMicroseconds (lastServoVal - Nvm.servoSetback);
        PRINTLN ("---");
      }
      else  {
        G.Srv.writeMicroseconds (lastServoVal + Nvm.servoSetback);
        PRINTLN ("+++");
      }
      delay (SERVO_SB_DELAY);
    }
    
    if (!noBlink) G.Led.blink (1, 50, 100); // blink once
    G.autoPowerOffTs = ts;    // reset the auto power-of timer                     
    
    G.Srv.writeMicroseconds (servoVal + sgn(delta) * Nvm.servoComp );

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


