#include <avr/sleep.h>
/*
 * Mazda RX-8 Navigation Hood Control
 * Dason Wong 2016-08-21
 * Version 1.0
 * https://www.youtube.com/watch?v=jJZqU2XOzgE
 */

// Defining Global Constants
const bool OPEN = true;
const bool CLOSE = false;
const bool HOODOPENED = true;
const bool HOODCLOSED = false;
const int ACCPIN = 2;               // ACC pin number
const int MOTORENABLE = 3;          // Motor enable pin number
const int TILTPIN = 4;              // Tilt button pin number
const int OPENPIN = 6;              // Open/close button pin number
const int CHARGEENABLE = 11;        // Charge enable pin number
const int MOTORDIR = 12;            // Motor direction pin number
const int CHARGEON = 13;            // Charge on pin number
const int HOODOPENEDVALUE = 285;    // Analogue potentiometer value when hood is open
const int HOODCLOSEDVALUE = 950;    // Analogue potentiometer value when hood is closed
const int HOODPOSTOLERANCE = 10;    // Analogue potentiometer value tolerance
const int TILTDURATION = 15;        // Time (ms) to run the motor for a single hood tilt
const int BUTTONDELAY = 400;        // Minimum time between button presses
const int ACCDETECTDELAY = 2000;    // Time (ms) that ACC needs to be on before car is considered 'on'
const int CAROFFCHARGETIME = 43200; // Charge tablet when car off duration (s)
const int MAXTILT = 2;              // Max hood tilt level

// Defining Global Variables
boolean carOff = true;                  // Initialise carOff state
boolean onHoodStatus = HOODCLOSED;      // Track desired hood status when car on, controlled by checkOpenButton()
boolean currentHoodStatus = HOODCLOSED; // Track current physical hood status, controlled by operateHood()
int openButtonState = 0;                // Initialise openButtonState
int tiltButtonState = 0;                // Initialise tiltButtonState
int tiltLevel = 0;                      // Initialise hood tilt level (0:none - 2:tilted
int carOffTime = 0;                     // Time since ACC off
int motorRunTime = 0;                   // Time the motor has run for

void setup() {
  /*
   * Setup the Arduino's initial state, runs once on boot or if the device is reset
   */
  // Setup the input pins
  pinMode(OPENPIN, INPUT);        // Set the pushbutton pin as an input:
  pinMode(TILTPIN, INPUT);        // Set the TILTPIN as an input:
  pinMode(ACCPIN, INPUT);         // Set the ACC pin as an input:
  digitalWrite(OPENPIN, HIGH);    // Activate the pull up for the Open button pin
  digitalWrite(TILTPIN, HIGH);    // Activate the pull up for the Tilt button pin
  
  // Setup the motor control pins as outputs
  pinMode (MOTORENABLE, OUTPUT);
  pinMode (MOTORDIR, OUTPUT);       // LOW is down (ie. reverse polarity)
  digitalWrite(MOTORENABLE, LOW);   // Disable the motor
  
  // Setup the tablet charging control pins as outputs:
  pinMode (CHARGEENABLE, OUTPUT);
  pinMode(CHARGEON, OUTPUT);
  digitalWrite(CHARGEENABLE, HIGH);   // Enable tablet charging
  digitalWrite(CHARGEON, HIGH);       // Start tablet charging
  
  // Start serial communication at 9570 bits per second for debugging
  Serial.begin(9570);
  Serial.println("RX-8 Navhood control running...");
}

void loop() {
  /*
   * The main function, runs continuously
   */
  if (digitalRead(ACCPIN) == HIGH) {  // Accessories are on (car on)
    checkResume();                    // Check if we the car was previously off
    checkOpenButton();                // Check if the Open button has been pressed
    checkTiltButton();                // Check if the Tilt button has been pressed
  }else{                              // Accessories are off (car off)
    if (carOff) {
      checkCarOffTime();
    }else{
      checkOff();
    }
  }
}

void checkResume() {
  /*
   * Check if we are resuming from a 'carOff' event, restore previous hood position
   */
  if (carOff == true) {                  // Only run if car off
    delay(ACCDETECTDELAY);               // Wait a ACCDETECTDELAY time
    if (digitalRead(ACCPIN) == HIGH) {   // Check to see if Accessories are still on
      if (onHoodStatus == HOODOPENED) {
        operateHood(OPEN, false);        // Restore the previous hood position
        restorePosition();              
      }
      carOff = false;
      carOffTime = 0;
    }
  }
}

void checkOpenButton() {
  /*
   * Check if the Open button has been pressed and perform the open action
   */
  openButtonState = digitalRead(OPENPIN);                         // Get the current state of the Open button
  if (openButtonState == LOW) {                                   // Button has been pressed
    if (analogRead(A5) < (HOODCLOSEDVALUE - HOODPOSTOLERANCE)) {  // Hood is open
      operateHood(CLOSE, false);
      onHoodStatus = HOODCLOSED;
    }else{                                                        // Hood is closed
      operateHood(OPEN, false);
      restorePosition();
      onHoodStatus = HOODOPENED;
    }
    delay(BUTTONDELAY);                                           // A delay so we have time to capture the button release
  }
}

void checkTiltButton() {
  /*
   * Check if the Tilt button has been pressed and perform the tilt action
   */
  tiltButtonState = digitalRead(TILTPIN);   // Get the current state of the Tilt button
  if (tiltButtonState == LOW) {             // Button has been pressed
    if (currentHoodStatus == HOODOPENED) {  // Hood is currently physically open
      if (tiltLevel == MAXTILT) {           // If at max tilt, return to fully opened
        operateHood(OPEN, false);
        tiltLevel = 0;
      }else{                                // Otherwise, tilt it
        operateHood(CLOSE, true);
        tiltLevel++;
      }
      delay(BUTTONDELAY);                   // A delay so we have time to capture the button release
    }
  }
}

void checkOff() {
  /*
   * Check if the car is off and closes the hood
   */
  delay(ACCDETECTDELAY);                                          // Wait a ACCDETECTDELAY time
  if (digitalRead(ACCPIN) == LOW) {                               // Check if Accessories is still off
    if (analogRead(A5) < (HOODCLOSEDVALUE - HOODPOSTOLERANCE)) {  // Hood is open, close it
      operateHood(CLOSE, false);
    }
    carOff = true;
    carOffTime += 2;
  }
}

void checkCarOffTime() {
  /*
   * Check how long the car has been off and puts the Arduino to sleep
   */
  if (carOffTime > CAROFFCHARGETIME) {                            // Car has been off for > CAROFFCHARGETIME
    digitalWrite(CHARGEENABLE, LOW);                              // Stop charging the tablet
    delay(100);
    sleepNow();                                                   // Make the Arduino go into sleep mode
    digitalWrite(CHARGEENABLE, HIGH);                             // Resume tablet charging when we wakeup
  }
  delay(2000);
  carOffTime += 2;                                                // Increment the car off timer (s)
}

void restorePosition() {
  /*
   * Restore the previous tilt position if any
   */
  for (int i = 0; i < tiltLevel; i++) {   // Tilt to previous desired level if any
    delay(BUTTONDELAY);
    operateHood(CLOSE, true);
  }
}

void operateHood(bool dir, bool tilt) {
  /*
   * Operate the navigation hood by driving the motor
   * 
   * Args:
   *    dir: A boolean determining the direction to drive the hood (OPEN or CLOSE)
   *    tilt: A boolean flag to signify if we only want to tilt the hood
   */
  if (tilt) {                                                         // If we only want to tilt the hood, dir is ignored
    digitalWrite(MOTORDIR, LOW);                                      // Set motor drive direction to close
    digitalWrite(MOTORENABLE, HIGH);                                  // Enable the motor
    delay(TILTDURATION);                                              // Keep motor running for TILTDURATION
    digitalWrite(MOTORENABLE, LOW);                                   // Stop the motor
  }else{
    if (dir) {                                                        // If direction is OPEN
      String debugString = "Opening hood, potentiometer: ";
      Serial.println(debugString + analogRead(A5));
      digitalWrite(MOTORDIR, HIGH);                                   // Set motor drive direction to open
      digitalWrite(MOTORENABLE, HIGH);                                // Enable the motor
      while (analogRead(A5) > HOODOPENEDVALUE && motorRunTime < 20) { // Run until we're fully open. If it's run for over 2s stop
        delay(100);
        motorRunTime++;
      }
      digitalWrite(MOTORENABLE, LOW);                                 // Stop the motor
      motorRunTime = 0;
      currentHoodStatus = HOODOPENED;
    }else{                                                            // If direction is CLOSE
      String debugString = "Closing hood, potentiometer: ";
      Serial.println(debugString + analogRead(A5));
      digitalWrite(MOTORDIR, LOW);                                    // Set motor drive direction to close
      digitalWrite(MOTORENABLE, HIGH);                                // Enable the motor
      while (analogRead(A5) < HOODCLOSEDVALUE && motorRunTime < 20) { // Run until we're fully closed. If it's run for over 2s stop
        delay(100);
        motorRunTime++;
      }
      digitalWrite(MOTORENABLE, LOW);                                 // Stop the motor
      motorRunTime = 0;
      currentHoodStatus = HOODCLOSED;
    }
  }
}

void sleepNow() {
  /*
   * Setup an interrupt and enter sleep mode
   */
  Serial.println("Entering sleep mode");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // Set type of sleep mode
  sleep_enable();                        // Enable sleep mode
  attachInterrupt(0, wakeUp, HIGH);      // Use interrupt 0 (pin 2 ie. ACC input to wake device)
  sleep_mode();                          // Put device to sleep
  sleep_disable();                       // Execution resumes from here after waking up
  detachInterrupt(0);
  Serial.println("Resuming from Sleep");
}

void wakeUp() {
  /*
   * The wakeUp() interrupt service routine will run when we get input from ACC (pin 2)
   * Since we just want the device to wake up we do nothing here
   */
}
