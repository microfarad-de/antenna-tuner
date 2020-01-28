/*
 * Magnetic Loop Antenna Tuner
 * 
 * Karim Hraibi - 2018
 */

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
#define MOSFET_PIN 7        // output pin for controlling the power-on MOSFTE
#define BUTTON_PWR_PIN 10   // input pin for "power" button
#define BUTTON_INC_PIN 12   // input pin for "increase" button
#define BUTTON_DEC_PIN 11   // input pin for "decrease" button
#define POT_APIN A0         // analog pin connected to the potentiometer

#define SERVO_MIN 750        // minimum servo PPM pulse width (us)
#define SERVO_MAX 2250       // maximum servo PPM pulse width (us)
#define SERVO_FINE_RANGE 100 // range for serve fine adjustment (us)
#define SERVO_SETBACK 60     // setback the servo by this amount when changing direction (us)
#define SERVO_SB_DELAY 60    // delay duration after servo setback (ms)
#define SERVO_DIR_COMP 5     // compensaton value when changing direction (us)

#define BUTTON_DELAY 5       // delay for repeating actions when holding a button (ms)

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
} G;


/*
 * Non-volatile memory contents (EEPROM)
 */
struct {
  int16_t coarsePosition;        // coarse serovo position (us)
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
  digitalWrite (MOSFET_PIN, HIGH);
  pinMode (BUTTON_PWR_PIN, INPUT_PULLUP);
  pinMode (BUTTON_INC_PIN, INPUT_PULLUP);
  pinMode (BUTTON_DEC_PIN, INPUT_PULLUP);
  //analogReference (INTERNAL);

  // initialize various objects
  ADConv.initialize ();
  G.Led.initialize (LED_PIN);
  G.FirFilter.initialize (G.firMemory, FIR_FILTER_TAPS);

  // intialize the servo object
  G.Srv.attach (PPM_PIN, SERVO_MIN, SERVO_MAX); 
  G.Srv.writeMicroseconds (SERVO_MIN);

  // read the non-volatile memory
  eepromRead (0x0, (uint8_t *)&Nvm, sizeof (Nvm));
  if (Nvm.coarsePosition < SERVO_MIN || Nvm.coarsePosition > SERVO_MAX - SERVO_FINE_RANGE) Nvm.coarsePosition = SERVO_MIN;
  
  // enable the watchdog
  //wdt_enable (WDTO_8S);
}



/*
 * Arduino main loop
 */
void loop() {
  static enum { STARTUP, RUNNING, SHUTDOWN, AUTO_SHUTDOWN } state = STARTUP;
  int32_t ts = millis ();

  buttonRead ();
  G.Led.loopHandler ();

  G.ButtonPwr.rising ();  // catch the rising edge of the power button

  // Main state machine
  switch (state) {
    case STARTUP:
      // power button - long press
      if (G.ButtonPwr.longPress ()) {
        G.Led.turnOn ();                // turn on the led indicator
        digitalWrite (MOSFET_PIN, LOW); // turn on the MOSFET
        G.autoPowerOffTs = ts;            // reset the auto power-off timer
        state = RUNNING;
      }
      break;

    case RUNNING:

      // power button - long press
      if (G.ButtonPwr.longPress ()) state = SHUTDOWN;

      // auto power-off timeout
      if (ts - G.autoPowerOffTs > AUTO_POWER_OFF_DELAY) state = AUTO_SHUTDOWN;

      servoControl ();
      
      break;

    case SHUTDOWN:
      G.Srv.writeMicroseconds (SERVO_MIN); // move the servo to storage-safe position
    case AUTO_SHUTDOWN:
      G.Led.turnOff ();                    // turn off the led indicator
      eepromWrite (0x0, (uint8_t *)&Nvm, sizeof (Nvm)); // write-back NVM settings
      delay (1000);
      digitalWrite (MOSFET_PIN, HIGH);     // turn off the MOSFET
      while (1) {};
      break;
    
  }

  
  
  
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
void servoControl () {
  static uint32_t buttonTs = 0;
  static int16_t lastServoVal = 0;
  static int16_t lastDelta = 0;
  static IirFilterClass IirFilter;
  int16_t servoVal;
  int16_t delta;
  uint32_t ts = millis ();

  if (G.ButtonInc.pressed && ts - buttonTs > BUTTON_DELAY) {
    if (Nvm.coarsePosition < SERVO_MAX - SERVO_FINE_RANGE) Nvm.coarsePosition++;
    buttonTs = ts;
  }
  else if (G.ButtonDec.pressed && ts - buttonTs > BUTTON_DELAY) {
    if (Nvm.coarsePosition > SERVO_MIN) Nvm.coarsePosition--;
    buttonTs = ts;
  }

  // calculate servo position
  servoVal = Nvm.coarsePosition + map (G.potPosition, 0, 1023, 0,  SERVO_FINE_RANGE);
  delta = servoVal - lastServoVal;

  // if PPM value has changed
  if (abs (delta) > 0) {

    G.Led.blink ();
    G.autoPowerOffTs = ts; // reset the auto power-of timer                     

    // handle changed rotation direction
    if (sgn (delta) == -sgn (lastDelta)) {
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
    
    G.Srv.writeMicroseconds (servoVal + sgn(delta) * SERVO_DIR_COMP);

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
}




