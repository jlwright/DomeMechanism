/*
 * //Code for R2-D2 Dome mechanism by Matthew Zwarts
//Version 1 released 14th September 2019 
//Items used Arduino Mega 2560, L298N Dual channel motor driver, PCA9685 16 channel servo driver, LM2596 DC-DC converter
//servo driver controlled via i2c
//Refer Thingiverse username: Matteous78 for drawing of parts layout, Dome Lift Mechanism Part 7

//NOTE: READ THIS
// I suggest you focus on connecting one mechanism first, the Lifeform Scanner or Periscope, to learn how the code will interact and fault find switch errors
// Wire up the components required and keep the belt tension on the pulleys firm, but not too tight, make sure the lifts slide easily before adding the belt tension
// Run this code to the serial monitor on your computer while plugged in to the arduino Mega 2560, Baud rate 57600, to show the limit switches ZBOT, ZTOP for example.
// Trigger the switches by hand to check they are connected to the arduino correctly and the value will change from 1 to 0, or vice versa
// The button triggers have three states, button count=1, move up , Button count=2, move down, button count=3, reset.
// Add power to the motor driver board and see if when the Button is pressed, PIN 34 on the Arduino to ground,it lifts the motor belt up or down
// If it moves the wrong direction, then just swap the 2 motor IN1 and IN2 pins on the motor driver
// Add one mechanism at a time to avoid having to try and fiugre out the complex wiring later
// Hope you can get it all moving...
// Regards, Matt Zwarts, Melbourne, Australia

// Notation abbreviations, these will be added to functions as a prefix to keep track of all the inputs and outputs
// P = periscope
// BM = Bad Motivator
// Z = Dome Zapper
// LS = Ligthsaber
// LF = Lifeform Scanner
// DS = Drink Server
*/

//Libraries to include
#include <Wire.h>
#include <Communication.h>
//#include <VarSpeedServo.h>
#include <math.h>
//#include <EnableInterrupt.h>
#include <Adafruit_PWMServoDriver.h>

//Call boards for i2c
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);

// Define constants and values to stay the same
#define SERIAL_PORT_SPEED 57600 // Define the port output serial communication speed

// our servo # counter
uint8_t servonum = 0;

#pragma region AppSettings
// Set servo channels (0-15 per board)
//pwm0 (0x40)
#define ZAPCHANNEL 4
#define ZAPTURNCHANNEL 5
#define PTURNCHANNEL 6
#define LFTURNCHANNEL 7
#define ZLEDCHANNEL 8
#define BMLEDCHANNEL 9
#define LFLEDCHANNEL 10
//pwm1 (0x41)
#define LSPPCHANNEL 0 //PP1
#define BMPPCHANNEL 1 //PP5
#define ZPPCHANNEL 2 //PP6
#define P10CHANNEL 3
#define P11CHANNEL 4
#define P13CHANNEL 5
#define HP1XCHANNEL 6
#define HP1YCHANNEL 7
//pwm2 (0x42)
#define LFPPCHANNEL 0 //PP2
#define P1CHANNEL 1
#define P2CHANNEL 2
#define P3CHANNEL 3
#define P4CHANNEL 4
#define P7CHANNEL 5
#define HP2XCHANNEL 6
#define HP2YCHANNEL 7
#define HP3XCHANNEL 8
#define HP3YCHANNEL 9

// These are the servo end points for the pie panels, adjust to open and close more or less
#define BMSERVOMIN  300 // adjust for pie panel position (150 - 600)
#define BMSERVOMAX  450 // adjust for pie panel position
#define ZSERVOMIN  300 // adjust for pie panel position (150 - 600)
#define ZSERVOMAX  450 // adjust for pie panel position
#define LSSERVOMIN  300 // adjust for pie panel position (150 - 600)
#define LSSERVOMAX  450 // adjust for pie panel position
#define LFSERVOMIN  300 // adjust for pie panel position (150 - 600)
#define LFSERVOMAX  450 // adjust for pie panel position

// Servo end points for dome mechanisms
#define ZAPSERVOMIN  350 // adjust Dome Zapper Down position (150 - 600)
#define ZAPSERVOMAX  180 // adjust Dome Zapper Up position
#define PTURNSERVOMIN  180 // adjust Dome Zapper Turn position (150 - 600)
#define PTURNSERVOMAX  450 // adjust Dome Zapper Turn position
#define ZAPTURNSERVOMIN  500 // adjust Dome Zapper Turn position (150 - 600)
#define ZAPTURNSERVOMAX  200 // adjust Dome Zapper Turn position
#define LFTURNSERVOMIN  180 // adjust Dome LF Turn position (150 - 600)
#define LFTURNSERVOMAX  500 // adjust Dome LF Turn position

// Servo end points for dome panels
#define P1MIN 300 // adjust for panel position (150 - 600)
#define P1MAX 450
#define P2MAX 300
#define P2MIN 450
#define P3MIN 300
#define P3MAX 450
#define P4MAX 300
#define P4MIN 450
#define P7MIN 300
#define P7MAX 450
#define P10MAX 300
#define P10MIN 450
#define P11MAX 300
#define P11MIN 450
#define P13MAX 300
#define P13MIN 450

//Servo end points for holo projectors
#define HP1_XMIN 300
#define HP1_XMAX 450
#define HP1_YMIN 300
#define HP1_YMAX 450
#define HP2_XMIN 300
#define HP2_XMAX 450
#define HP2_YMIN 300
#define HP2_YMAX 450
#define HP3_XMIN 300
#define HP3_XMAX 450
#define HP3_YMIN 300
#define HP3_YMAX 450

//LED high low settings below, connected to the Gnd and Signal line on the PCA9685
#define ZLEDSERVOMIN  200 // adjust Dome Zapper Turn position (150 - 600)
#define ZLEDSERVOMAX  500 // adjust Dome Zapper Turn position
#define BMLEDSERVOMIN  200 // adjust Dome Zapper Turn position (150 - 600)
#define BMLEDSERVOMAX  500 // adjust Dome Zapper Turn position

//Motor drivers
#define PIN1 2 //Periscope Motor Driver IN1
#define PIN2 3 //Periscope Motor Driver IN2
#define BMIN1 4 //Bad Motivator Motor Driver IN1
#define BMIN2 5 //Bad Motivator Motor Driver IN2
#define ZIN1 6 //Zapper Motor Driver IN1
#define ZIN2 7 //Zapper Motor Driver IN2
#define LSIN1 8 //Lightsaber Motor Driver IN1
#define LSIN2 9 //Lightsaber Motor Driver IN2
#define LFIN1 10 //Lifeform Motor Driver IN1
#define LFIN2 11 //Lifeform Motor Driver IN2\

//Limit Switches for up and down
#define PTop 22 //Periscope top limit switch
#define PBot 23 //Periscope bottom limit switch
#define BMTop 24 //Bad Motivator top limit switch
#define BMBot 25 //Bad Motivator bottom limit switch
#define ZTop 26 //Zapper top limit switch
#define ZBot 27 //Zapper bottom limit switch
#define LSTop 28 //Lightsaber top limit switch
#define LSBot 29 //Lightsaber bottom limit switch
#define LFTop 30 //Lifeform top limit switch
#define LFBot 31 //Lifeform bottom limit switch

// These pins are connected to ground to trigger, this can be done with a push button or code added later to make this remote from another source
#define buttonPin 38 // button pin to trigger dome zapper lift mechanism
#define buttonPin1 34 // button pin to trigger periscope lift mechanism
#define buttonPin2 37 // button pin to trigger Lifeform scanner lift mechanism
#define buttonPin3 35 // button pin to trigger Lightsaber lift mechanism
#define buttonPin4 36 // button pin to trigger Bad Motivator lift mechanism
#define buttonPin5 39 // button pin to trigger Drink Server lift mechanism
#pragma endregion

#pragma region Timers
// timers
unsigned long currentMillis; // running clock reference

unsigned long zappreviousMillis; // zapper store time
long zapinterval = 30; // time between flashes
unsigned long zapturnpreviousMillis; // zapper turn store time
long zapturninterval = 2000; // time between turning dome zapper
long zapturninterval2 = 4000; //  longer time between turning dome zapper
static unsigned long zap_time_stopped; // time for the dome zapper to stay up
unsigned long zappause; // time for the zapper top position to be stored
byte zapflashcount = 0;   // counter for dome zapper flashes
byte zapperturncount = 0; // counter for dome zapper turns

unsigned long bmpreviousMillis; // bad motivator store time
long bminterval = 200; // time between led flashes

unsigned long pturnpreviousMillis; // Periscope store time
long pturninterval = 1200;  // time between posotional turns
byte pturncount = 0; // count how many turns for the periscope

unsigned long lfturnpreviousMillis; // Lifeform scanner store time
long lfturninterval = 600;  // time between posotional turns
byte lfturncount = 0; // count how many turns for the periscope
const long lfledinterval = 500; // time between Lifeform scanner led flashes
unsigned long lfledpreviousmillis = 0; // storage for led flashes

unsigned long dspreviousMillis; // drink server store time
long dsinterval = 4000; // time between drink server going up

// button timer for debounce if required
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
#pragma endregion

// sates to select from the different case functions
static enum {ZAP_MOVE_TOP, ZAP_TOP} statezapup; //zapper up
static enum {ZAP_MOVE_BOT, ZAP_BOT} statezapdown; // zapper down
static enum {P_MOVE_TOP, P_TOP} statepup; //periscope up
static enum {P_MOVE_BOT, P_BOT} statepdown; //periscope down
static enum {LF_MOVE_TOP, LF_TOP} statelfup; //lifeform scanner up
static enum {LF_MOVE_BOT, LF_BOT} statelfdown; //lifeform scanner down
static enum {BM_MOVE_TOP, BM_TOP} statebmup; //Bad Motivator up
static enum {BM_MOVE_BOT, BM_BOT} statebmdown; //Bad Motivator down
static enum {LS_MOVE_TOP, LS_TOP} statelsup; //Lightsaber up
static enum {LS_MOVE_BOT, LS_BOT} statelsdown; //Lightsaber down

int statez; //zapper arm lift and turn
int statezl; //zapper led
int statept; //periscope turn
int statelf; //lifeform scanner
int statelft; //lifeform scanner turn
int statebml; //bad motivator led

//set integers
// storage for limit switch values
int PTopVal = LOW;
int PBotVal = LOW;
int BMTopVal = LOW;
int BMBotVal = LOW;
int ZTopVal = LOW;
int ZBotVal = LOW;
int LSTopVal = LOW;
int LSBotVal = LOW;
int LFTopVal = LOW;
int LFBotVal = LOW;

// input buttons
int buttonPushCounter = 0;    // counter for the number of button presses
int buttonPushCounter1 = 0;   // counter for the number of button presses
int buttonPushCounter2 = 0;   // counter for the number of button presses
int buttonPushCounter3 = 0;   // counter for the number of button presses
int buttonPushCounter4 = 0;   // counter for the number of button presses
int buttonPushCounter5 = 0;   // counter for the number of button presses
int ledState = LOW;           // the current state of the output pin
int ledState1 = LOW;          // the current state of the output pin
int ledState2 = LOW;          // the current state of the output pin
int buttonState = 0;          // the current state of the button for Dome Zapper
int buttonState1 = 0;         // the current state of the button for Periscope
int buttonState2 = 0;         // the current state of the button for Lifeform Scanner
int buttonState3 = 0;         // the current state of the button for Bad Motivator
int buttonState4 = 0;         // the current state of the button for Lightsaber
int buttonState5 = 0;         // the current state of the button for Drink Server
int lastButtonState = 0;      // the previous reading from the input pin
int lastButtonState1 = 0;     // the previous reading from the input pin
int lastButtonState2 = 0;     // the previous reading from the input pin
int lastButtonState3 = 0;     // the previous reading from the input pin
int lastButtonState4 = 0;     // the previous reading from the input pin
int lastButtonState5 = 0;     // the previous reading from the input pin


void setup() {
  Serial.begin(SERIAL_PORT_SPEED);// serial communicationpwm.begin();
  pwm0.begin();
  pwm0.setPWMFreq(50); //  standard for analog servos
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  //output pins
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(BMIN1, OUTPUT);
  pinMode(BMIN2, OUTPUT);
  pinMode(ZIN1, OUTPUT);
  pinMode(ZIN2, OUTPUT);
  pinMode(LSIN1, OUTPUT);
  pinMode(LSIN2, OUTPUT);
  pinMode(LFIN1, OUTPUT);
  pinMode(LFIN2, OUTPUT);
  //pinMode(ledPin, OUTPUT); // led pin on the arduino for testing
  
  // input pins
  pinMode(PTop, INPUT_PULLUP); // using a limit switch, pin goes to Normally Open and Common goes to ground, value reads LOW when the switch is closed
  pinMode(PBot, INPUT_PULLUP);
  pinMode(BMTop, INPUT_PULLUP);
  pinMode(BMBot, INPUT_PULLUP);
  pinMode(ZTop, INPUT_PULLUP);
  pinMode(ZBot, INPUT_PULLUP);
  pinMode(LSTop, INPUT_PULLUP);
  pinMode(LSBot, INPUT_PULLUP);
  pinMode(LFTop, INPUT_PULLUP);
  pinMode(LFBot, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(buttonPin5, INPUT_PULLUP);

  // Write the motor pins low so they dont start on power up
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  digitalWrite(BMIN1, LOW);
  digitalWrite(BMIN2, LOW);
  digitalWrite(ZIN1, LOW);
  digitalWrite(ZIN2, LOW);
  digitalWrite(LSIN1, LOW);
  digitalWrite(LSIN2, LOW);
  digitalWrite(LFIN1, LOW);
  digitalWrite(LFIN2, LOW);

  //digitalWrite(ledPin, LOW); // set the led off

  //Set the servos to their start positions
  servoSetup(); // view function and end of code

  delay(1000); // wait for everything to get to their positions on power up
}


// Main Code here
void loop() {
  //reset timers
  currentMillis = millis();
  readlimits(); //read and store the limit switches values High or Low, function further down in code
  buttonState = digitalRead(buttonPin); // main trigger for button inputs
  buttonState1 = digitalRead(buttonPin1); // main trigger for button inputs
  buttonState2 = digitalRead(buttonPin2); // main trigger for button inputs
  buttonState3 = digitalRead(buttonPin3); // main trigger for button inputs
  buttonState4 = digitalRead(buttonPin4); // main trigger for button inputs
  buttonState5 = digitalRead(buttonPin5); // main trigger for button inputs

  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      buttonPushCounter++;
    }
  }

  if (buttonState1 != lastButtonState1) {
    if (buttonState1 == LOW) {
      buttonPushCounter1++;
    }
  }

  if (buttonState2 != lastButtonState2) {
    if (buttonState2 == LOW) {
      buttonPushCounter2++;
    }
  }

  if (buttonState3 != lastButtonState3) {
    if (buttonState3 == LOW) {
      buttonPushCounter3++;
    }
  }

  if (buttonState4 != lastButtonState4) {
    if (buttonState4 == LOW) {
      buttonPushCounter4++;
    }
  }
  
  if (buttonState5 != lastButtonState5) {
    if (buttonState5 == LOW) {
      buttonPushCounter5++;
    }
  }

  //Dome Zapper
  if (buttonPushCounter == 1) {
    DomeZapperUp();
    if (ZTopVal == LOW && ZBotVal == HIGH) {
      DomeZapper();
    }
  }
  else if (buttonPushCounter == 2) {
    pwm0.setPWM(5, 0, ZAPTURNSERVOMIN); //turn the zapper arm to original position
    pwm0.setPWM(4, 0, ZAPSERVOMIN); //lower the arm
    pwm0.setPWM(8, 0, 4096); // sets the led LOW
    DomeZapperDown();
  }
  else {
    buttonPushCounter = 0;
    statezapup = ZAP_MOVE_TOP;  // reset states for next lift sequence
    statezapdown = ZAP_MOVE_BOT;
    statez = 1;
    statezl = 0;
  }

  //Periscope
  if (buttonPushCounter1 == 1) {
    PeriscopeUp();
    if (PTopVal == LOW && PBotVal == HIGH) {
      PeriscopeTurn();
    }
  }
  else if (buttonPushCounter1 == 2) {
    pwm0.setPWM(6, 0, PTURNSERVOMIN); //periscope turn to original lift position
    PeriscopeDown();
  }
  else {
    buttonPushCounter1 = 0;
    statepup = P_MOVE_TOP;
    statepdown = P_MOVE_BOT;
    statept = 0;
  }

  //Lifeform scanner
  if (buttonPushCounter2 == 1) {
    LifeformUp();
    if (currentMillis - lfledpreviousmillis >= lfledinterval) {
      pwm0.setPWM(10, 4096 , 0); //Lifeform LED HIGH
      lfledpreviousmillis = currentMillis;
    }
    else {
      pwm0.setPWM(10, 0, 4096); //Lifeform LED LOW
    }
    if (LFTopVal == LOW && LFBotVal == HIGH) {
      LFTurn();
    }
  }
  else if (buttonPushCounter2 == 2) {
    pwm0.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn to original position
    LifeformDown();
  }
  else {
    buttonPushCounter2 = 0;
    statelfup = LF_MOVE_TOP;
    statelfdown = LF_MOVE_BOT;
    statelft = 0;
  }

  //Bad Motivator
  if (buttonPushCounter3 == 1) {
    BadMotivatorUp();
    pwm0.setPWM(9, 4096, 0);
  }
  else if (buttonPushCounter3 == 2) {
    BadMotivatorDown();
    pwm0.setPWM(9, 0, 4096);
  }
  else {
    buttonPushCounter3 = 0;
    statebmup = BM_MOVE_TOP;
    statebmdown = BM_MOVE_BOT;
  }

  //Lightsaber Lifter
  if (buttonPushCounter4 == 1) {
    LightsaberUp();
  }
  else if (buttonPushCounter4 == 2) {
    LightsaberDown();
  }
  else {
    buttonPushCounter4 = 0;
    statelsup = LS_MOVE_TOP;
    statelsdown = LS_MOVE_BOT;
  }

  SerialOut(); // print to serial all values for testing
  lastButtonState = buttonState; // reset the input button
  lastButtonState1 = buttonState1; // reset the input button
  lastButtonState2 = buttonState2; // reset the input button
  lastButtonState3 = buttonState3; // reset the input button
  lastButtonState4 = buttonState4; // reset the input button
  lastButtonState5 = buttonState5; // reset the input button
}


////////////////////////////////////////////////////////////
// Functions below here for features
////////////////////////////////////////////////////////////
void DomeZapperUp() { // this function is for the dome zapper
  switch (statezapup) {
    case ZAP_MOVE_TOP:
      if (ZTopVal != LOW) {
        if (digitalRead(ZTop) == HIGH && digitalRead(ZBot) == LOW) {
          pwm1.setPWM(2, 0, ZSERVOMAX); // open the pie panel
        }
        digitalWrite(ZIN1, HIGH); //turn the dc motor on
        digitalWrite(ZIN2, LOW);
        statezapup = ZAP_TOP;
      }
      break;
    case ZAP_TOP:
      if (ZTopVal == LOW) {
        digitalWrite(ZIN1, LOW); //turn the motor off
        digitalWrite(ZIN2, LOW); //turn the motor off
        //pwm0.setPWM(4, 0, ZAPSERVOMAX); //lift zapper arm
      }
      break;
  }
}

void DomeZapperDown() { // this function is for the dome zapper
  switch (statezapdown) {
    case ZAP_MOVE_BOT:
      if (ZBotVal != LOW) {
        digitalWrite(ZIN1, LOW); //turn the dc motor on
        digitalWrite(ZIN2, HIGH);
        statezapdown = ZAP_BOT;
      }
      break;
    case ZAP_BOT:
      if (ZBotVal == LOW) {
        digitalWrite(ZIN1, LOW); //turn the dc motor on
        digitalWrite(ZIN2, LOW);
        if (digitalRead(ZBot) == LOW && digitalRead(ZTop) == HIGH) {
          pwm1.setPWM(2, 0, ZSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}

void DomeZapper() { //lift zapper arm servo, flash light, rotate to new position and flash, return to first position, arm down
  switch (statez) {
    case 1:
      currentMillis = millis();
      pwm0.setPWM(4, 0, ZAPSERVOMAX); //lift zapper arm
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        statez = 2;
        zapturnpreviousMillis = currentMillis;  
      }
      break;

    case 2:
      currentMillis = millis();
      pwm0.setPWM(5, 0, ZAPTURNSERVOMAX); //turn zapper arm
      ZapLed(); // flash the LED
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        statez = 3;
        zapturnpreviousMillis = currentMillis;
      }
      break;

    case 3:
      currentMillis = millis();
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        pwm0.setPWM(5, 0, ZAPTURNSERVOMIN); //turn
        statez = 0;
        zapturnpreviousMillis = currentMillis;
      }
      break;
  }
}

void ZapLed() {
  switch (statezl)
  {
    case 0:
      currentMillis = millis();
      pwm0.setPWM(8, 4096, 0); // sets the led HIGH from the PCA9685
      if (currentMillis - zappreviousMillis >= zapinterval) {
        zappreviousMillis = currentMillis;
        statezl = 1;
      }
      break;

    case 1:
      currentMillis = millis();
      pwm0.setPWM(8, 0, 4096); // sets the led LOW from the PCA9685
      if (currentMillis - zappreviousMillis >= zapinterval) {
        zappreviousMillis = currentMillis;
        statezl = 2;
      }
      break;

    case 2:
      currentMillis = millis();
      zapflashcount++;
      if (zapflashcount == 80) {
        statezl = 3;
        zapflashcount = 0;
      }
      else {
        statezl = 0;
      }
      break;
  }
}


void PeriscopeUp() { //button tirggered Lift periscope, flash lights, rotate back and forwards, when button triggered again lower again in home position turn off lights
  switch (statepup) {
    case P_MOVE_TOP:
      if (PTopVal != LOW) {
        digitalWrite(PIN1, HIGH); //turn the dc motor on
        digitalWrite(PIN2, LOW);
        statepup = P_TOP;
      }
      break;
    case P_TOP:
      if (PTopVal == LOW) {
        digitalWrite(PIN1, LOW); //turn the motor off
        digitalWrite(PIN2, LOW); //turn the motor off
      }
      break;

  }
}

void PeriscopeDown() { // this function lowers the periscope
  switch (statepdown) {
    case P_MOVE_BOT:
      if (PBotVal != LOW) {
        digitalWrite(PIN1, LOW); //turn the dc motor on
        digitalWrite(PIN2, HIGH);
        statepdown = P_BOT;
      }
      break;
    case P_BOT:
      if (PBotVal == LOW) {
        digitalWrite(PIN1, LOW); //turn the dc motor on
        digitalWrite(PIN2, LOW);
      }
      break;
  }
}

void PeriscopeTurn() { // turn the periscope back and forwards
  switch (statept)
  {
    case 0:
      currentMillis = millis();
      if (currentMillis - pturnpreviousMillis >= pturninterval) {
        pwm0.setPWM(6, 0, PTURNSERVOMAX); //turn
        pturnpreviousMillis = currentMillis;
        statept = 1;
      }
      break;

    case 1:
      currentMillis = millis();
      if (currentMillis - pturnpreviousMillis >= pturninterval) {
        pwm0.setPWM(6, 0, PTURNSERVOMIN); //turn
        pturnpreviousMillis = currentMillis;
        statept = 2;
      }
      break;

    case 2:
      currentMillis = millis();
      pturncount++;
      if (pturncount == 3) {
        statept = 3;
        pturncount = 0;
      }
      else {
        statept = 0;
      }
      break;
  }
}


void LifeformUp() { //button tirggered Lift Lifeform Scanner, when button triggered again lower in home position
  switch (statelfup) {
    case LF_MOVE_TOP:
      if (LFTopVal != LOW) {
        if (digitalRead(LFTop) == HIGH && digitalRead(LFBot) == LOW) {
          pwm2.setPWM(0, 0, LFSERVOMAX); // open the pie panel
        }
        digitalWrite(LFIN1, HIGH); //turn the dc motor on
        digitalWrite(LFIN2, LOW);
        statelfup = LF_TOP;
      }
      break;
    case LF_TOP:
      if (LFTopVal == LOW) {
        digitalWrite(LFIN1, LOW); //turn the motor off
        digitalWrite(LFIN2, LOW); //turn the motor off
      }
      break;
  }
}

void LifeformDown() { // this function lowers the Lifeform scanner
  switch (statelfdown) {
    case LF_MOVE_BOT:
      if (LFBotVal != LOW) {
        digitalWrite(LFIN1, LOW); //turn the dc motor on
        digitalWrite(LFIN2, HIGH);
        statelfdown = LF_BOT;
      }
      break;
    case LF_BOT:
      if (LFBotVal == LOW) {
        digitalWrite(LFIN1, LOW); //turn the dc motor on
        digitalWrite(LFIN2, LOW);
        if (digitalRead(LFBot) == LOW && digitalRead(LFTop) == HIGH) {
          pwm2.setPWM(0, 0, LFSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}

void LFTurn() { // turn the Lifeform scanner back and forwards
  switch (statelft)
  {
    case 0:
      currentMillis = millis();
      if (currentMillis - lfturnpreviousMillis >= lfturninterval) {
        pwm0.setPWM(7, 0, LFTURNSERVOMAX); //Lifeform turn
        lfturnpreviousMillis = currentMillis;
        statelft = 1;
      }
      break;

    case 1:
      currentMillis = millis();
      if (currentMillis - lfturnpreviousMillis >= lfturninterval) {
        pwm0.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn
        lfturnpreviousMillis = currentMillis;
        statelft = 2;
      }
      break;

    case 2:
      currentMillis = millis();
      lfturncount++;
      if (lfturncount == 6) {
        statelft = 3;
        lfturncount = 0;
      }
      else {
        statelft = 0;
      }
      break;
  }
}


void BadMotivatorUp() { //button tirggered Lift Bad Motivator, when button triggered again lower in home position
  switch (statebmup) {
    case BM_MOVE_TOP:
      if (BMTopVal != LOW) {
        if (digitalRead(BMTop) == HIGH && digitalRead(BMBot) == LOW) {
          pwm1.setPWM(1, 0, BMSERVOMAX); // open the pie panel
        }
        digitalWrite(BMIN1, HIGH); //turn the dc motor on
        digitalWrite(BMIN2, LOW);
        statebmup = BM_TOP;
      }
      break;
    case BM_TOP:
      if (BMTopVal == LOW) {
        digitalWrite(BMIN1, LOW); //turn the motor off
        digitalWrite(BMIN2, LOW); //turn the motor off
      }
      break;
  }
}

void BadMotivatorDown() { // this function lowers the Bad Motivator
  switch (statebmdown) {
    case BM_MOVE_BOT:
      if (BMBotVal != LOW) {
        digitalWrite(BMIN1, LOW); //turn the dc motor on
        digitalWrite(BMIN2, HIGH);
        statebmdown = BM_BOT;
      }
      break;
    case BM_BOT:
      if (BMBotVal == LOW) {
        digitalWrite(BMIN1, LOW); //turn the dc motor on
        digitalWrite(BMIN2, LOW);
        if (digitalRead(BMBot) == LOW && digitalRead(BMTop) == HIGH) {
          pwm1.setPWM(1, 0, BMSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}


void LightsaberUp() { //button tirggered Lift Lightsaber, when button triggered again lower
  switch (statelsup) {
    case LS_MOVE_TOP:
      if (LSTopVal != LOW) {
        if (digitalRead(LSTop) == HIGH && digitalRead(LSBot) == LOW) {
          pwm1.setPWM(0, 0, LSSERVOMAX); // open the pie panel
        }
        digitalWrite(LSIN1, HIGH); //turn the dc motor on
        digitalWrite(LSIN2, LOW);
        statelsup = LS_TOP;
      }
      break;
    case LS_TOP:
      if (LSTopVal == LOW) {
        digitalWrite(LSIN1, LOW); //turn the motor off
        digitalWrite(LSIN2, LOW); //turn the motor off
      }
      break;
  }
}

void LightsaberDown() { // this function lowers the Lightsaber
  switch (statelsdown) {
    case LS_MOVE_BOT:
      if (LSBotVal != LOW) {
        digitalWrite(LSIN1, LOW); //turn the dc motor on
        digitalWrite(LSIN2, HIGH);
        statelsdown = LS_BOT;
      }
      break;
    case LS_BOT:
      if (LSBotVal == LOW) {
        digitalWrite(LSIN1, LOW); //turn the dc motor on
        digitalWrite(LSIN2, LOW);
        if (digitalRead(LSBot) == LOW && digitalRead(LSTop) == HIGH) {
          pwm1.setPWM(0, 0, LSSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}


void readlimits() { //this function reads all the limit swtiches and stores their values for compare to end stops in the main loop, it's just written here to keep the loop code clean
  PBotVal = digitalRead(PBot);
  PTopVal = digitalRead(PTop);
  BMTopVal = digitalRead(BMTop);
  BMBotVal = digitalRead(BMBot);
  ZBotVal = digitalRead(ZBot);
  ZTopVal = digitalRead(ZTop);
  LSBotVal = digitalRead(LSBot);
  LSTopVal = digitalRead(LSTop);
  LFBotVal = digitalRead(LFBot);
  LFTopVal = digitalRead(LFTop);
}

void servoSetup() {
  //ServoStartPositions();
  pwm0.setPWM(4, 0, ZAPSERVOMIN); //Zapper arm
  pwm0.setPWM(5, 0, ZAPTURNSERVOMIN); //Zapper arm turn
  pwm0.setPWM(6, 0, PTURNSERVOMIN); //Periscope turn
  pwm0.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn
  pwm0.setPWM(8, 0, 4096); //Zapper LED low
  pwm0.setPWM(9, 0, 4096); //Bad Motiviator LED low
  pwm0.setPWM(10, 0, 4096); //Lifeform LED low

  pwm1.setPWM(0,LSSERVOMIN,LSSERVOMAX); //PP1
  pwm1.setPWM(1,BMSERVOMIN,BMSERVOMAX); //PP5
  pwm1.setPWM(2,ZSERVOMIN,ZSERVOMAX); //PP6
  pwm1.setPWM(3,P10MIN,P10MAX); //P10
  pwm1.setPWM(4,P11MIN,P11MAX); //P11
  pwm1.setPWM(5,P13MIN,P13MAX); //P13
  pwm1.setPWM(6,HP1_XMIN,HP1_XMAX); //HP1_X
  pwm1.setPWM(7,HP1_YMIN,HP1_YMAX); //HP1_Y

  pwm2.setPWM(0,LFSERVOMIN,LFSERVOMAX); //PP2
  pwm1.setPWM(1,P1MIN,P1MAX); //P1
  pwm1.setPWM(2,P2MIN,P2MAX); //P2
  pwm1.setPWM(3,P3MIN,P3MAX); //P3
  pwm1.setPWM(4,P4MIN,P4MAX); //P4
  pwm1.setPWM(5,P7MIN,P7MAX); //P7
  pwm2.setPWM(6,HP2_XMIN,HP2_XMAX); //HP2_X
  pwm2.setPWM(7,HP2_YMIN,HP2_YMAX); //HP2_Y
  pwm2.setPWM(8,HP3_XMIN,HP3_XMAX); //HP3_X
  pwm2.setPWM(9,HP3_YMIN,HP3_YMAX); //HP3_Y
}

//Function to display values for testing, uncheck any values you don't want to or do want to see
void SerialOut() {
  Serial.println("Button Push Counters");
    Serial.println("B0\tB1\tB2\tB3\tB4\tB5");
    Serial.print(buttonPushCounter); Serial.print("\t");
    Serial.print(buttonPushCounter1); Serial.print("\t");
    Serial.print(buttonPushCounter2); Serial.print("\t");
    Serial.print(buttonPushCounter3); Serial.print("\t");
    Serial.print(buttonPushCounter4); Serial.print("\t");
    Serial.print(buttonPushCounter5); Serial.print("\n\n");
  Serial.println("Button States");
    Serial.println("St0\tSt1\tSt2\tSt3\tSt4\tSt5");
    Serial.print(buttonState); Serial.print("\t");
    Serial.print(buttonState1); Serial.print("\t");
    Serial.print(buttonState2); Serial.print("\t");
    Serial.print(buttonState3); Serial.print("\t");
    Serial.print(buttonState4); Serial.print("\t");
    Serial.print(buttonState5); Serial.print("\n\n");
  Serial.println("Limit Switch Values");
    Serial.println("    \tP\tBM\tZ\tLS\tLF");
    Serial.print("Top:\t"); 
      Serial.print(PTopVal); Serial.print("\t"); 
      Serial.print(BMTopVal); Serial.print("\t"); 
      Serial.print(ZTopVal); Serial.print("\t"); 
      Serial.print(LSTopVal); Serial.print("\t"); 
      Serial.print(LFTopVal); Serial.print("\n");
    Serial.print("Bot:\t"); 
      Serial.print(PBotVal); Serial.print("\t"); 
      Serial.print(BMBotVal); Serial.print("\t"); 
      Serial.print(ZBotVal); Serial.print("\t"); 
      Serial.print(LSBotVal); Serial.print("\t"); 
      Serial.print(LFBotVal); Serial.print("\n");
  
//  Serial.print("B0:"); Serial.print(buttonPushCounter); Serial.print("\t");
//  Serial.print("B1:"); Serial.print(buttonPushCounter1); Serial.print("\t");
//  Serial.print("B2:"); Serial.print(buttonPushCounter2); Serial.print("\t");
//  Serial.print("B3:"); Serial.print(buttonPushCounter3); Serial.print("\t");
//  Serial.print("B4:"); Serial.print(buttonPushCounter4); Serial.print("\t");
//  Serial.print("B5:"); Serial.print(buttonPushCounter5); Serial.print("\t");
//  //Serial.print("St0:"); Serial.print(buttonState); Serial.print("\t");
//  //Serial.print("St1:"); Serial.print(buttonState1); Serial.print("\t");
//  //Serial.print("St2:"); Serial.print(buttonState2); Serial.print("\t");
//  //Serial.print("St3:"); Serial.print(buttonState3); Serial.print("\t");
//  //Serial.print("St4:"); Serial.print(buttonState4); Serial.print("\t");
//  //Serial.print("St5:"); Serial.print(buttonState5); Serial.print("\t");
//  Serial.print("ZBot:"); Serial.print(ZBotVal); Serial.print("\t");
//  Serial.print("ZTop:"); Serial.print(ZTopVal); Serial.print("\t");
//  Serial.print("PBot:"); Serial.print(PBotVal); Serial.print("\t");
//  Serial.print("PTop:"); Serial.print(PTopVal); Serial.print("\t");
//  Serial.print("BMBot:"); Serial.print(BMBotVal); Serial.print("\t");
//  Serial.print("BMTop:"); Serial.print(BMTopVal); Serial.print("\t");
//  Serial.print("LFBot:"); Serial.print(LFBotVal); Serial.print("\t");
//  Serial.print("LFTop:"); Serial.print(LFTopVal); Serial.print("\t");
//  Serial.print("LSBot:"); Serial.print(LSBotVal); Serial.print("\t");
//  Serial.print("LSTop:"); Serial.print(LSTopVal); Serial.print("\t");
//  //Serial.print("DSBot:"); Serial.print(DSBotVal); Serial.print("\t");
//  //Serial.print("DSTop:"); Serial.print(DSTopVal); Serial.print("\t");
//  //Serial.print("statezl:"); Serial.print(statezl); Serial.print("\t");
//  Serial.print("Millis:"); Serial.print(currentMillis); Serial.println("\t");
//  Serial.print("-------------------------------------------\n\n");
}
