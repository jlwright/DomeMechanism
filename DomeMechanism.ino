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

#pragma region Includes
#include <Wire.h>
//#include <VarSpeedServo.h>
#include <math.h>
// #include <EnableInterrupt.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#pragma endregion

#pragma region GlobalVariables
//Call boards for i2c
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);

uint8_t servonum = 0;
boolean executingCommand = false; //tracks if a command is being executed
boolean isPLedOn = false;
boolean isLFLedOn = false;
boolean isBMLedOn = false;
#pragma endregion

#pragma region PinSettings
// ---------------------------------------------------------------------------------------
//                          Motor Driver Pins
// ---------------------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------------------
//                          Limit Switch Pins
// ---------------------------------------------------------------------------------------
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
// #define buttonPin 38 // button pin to trigger dome zapper lift mechanism
// #define buttonPin1 34 // button pin to trigger periscope lift mechanism
// #define buttonPin2 37 // button pin to trigger Lifeform scanner lift mechanism
// #define buttonPin3 35 // button pin to trigger Lightsaber lift mechanism
// #define buttonPin4 36 // button pin to trigger Bad Motivator lift mechanism
// #define buttonPin5 39 // button pin to trigger Drink Server lift mechanism
#pragma endregion

#pragma region AppSettings
#define DOMEMECH_DEBUG       //uncomment this for console DEBUG output
#define DOMEMECH_VERBOSE     //uncomment this for console VERBOSE output

#define SERIAL_PORT_SPEED 115200 // Define the port output serial communication speed
#define PWM_FREQ 50

// ---------------------------------------------------------------------------------------
//                          HP/MP Settings
// ---------------------------------------------------------------------------------------
#define NEO_PIXEL

#ifdef NEO_PIXEL
  #define PIN_HP 6 // Which pin on the Arduino is connected to the NeoPixels?
  #define NUMPIXELS_HP 21 // Number of pixels in the ring
  #define PIN_MP 7 // Pin connected to Magic Panel
  #define NUMPIXELS_MP 40 // Number of pixels in the magic panel
  Adafruit_NeoPixel pixelsHP(NUMPIXELS_HP, PIN_HP, NEO_GRB + NEO_KHZ800);
  Adafruit_NeoPixel pixelsMP(NUMPIXELS_MP, PIN_MP, NEO_GRB + NEO_KHZ800);
  Adafruit_PWMServoDriver domePwm0 = Adafruit_PWMServoDriver();
  Adafruit_PWMServoDriver domePwm1 = Adafruit_PWMServoDriver();
  Adafruit_PWMServoDriver domePwm2 = Adafruit_PWMServoDriver();

  // Color settings for pixels (R,G,B)
  #define RED (255,0,0,0)
  #define GREEN (0,255,0,0)
  #define BLUE (0,0,255,0)
  #define WHITE (0,0,0,255)
  #define LIGHTBLUE (0,0,40,255)
  #define PURPLE (255,255,0,0)
  #define PINK (40,0,0,255)

  const char* colorNames[] = {
    "RED",
    "GREEN",
    "BLUE",
    "WHITE",
    "LIGHTBLUE",
    "PURPLE",
    "PINK"
  };

  const int HOLO_DELAY = 20000; //up to 20 second delay
  const int HOLO_SERVO_CTR = 300;

  uint32_t holoFrontRandomTime = 0;
  uint32_t holoBackRandomTime = 0;
  uint32_t holoTopRandomTime = 0;
  int holoBrightness = 5; // Brightness of holos
  uint32_t holoColor = WHITE; // default holo color to WHITE
  boolean areHolosOn = true; // default holos to on
  boolean isHoloAutomationOn = false;

  int magicPanelBrightness = 5; // brightness of magic panel
  uint32_t magicPanelColor = RED; // default magic panel color
  boolean isMagicPanelOn = false; // defautl magic panel to off
#endif

#pragma endregion

#pragma region ServoSettings
// //pwm0 (0x40)
#define ZAPCHANNEL 4
#define ZAPTURNCHANNEL 5
#define PTURNCHANNEL 6
#define LFTURNCHANNEL 7
#define ZLEDCHANNEL 8
#define BMLEDCHANNEL 9
#define LFLEDCHANNEL 10
#define PLEDCHANNEL 11
// //pwm1 (0x41)
#define LSPPCHANNEL 0 //PP1
#define BMPPCHANNEL 1 //PP5
#define ZPPCHANNEL 2 //PP6
#define P10CHANNEL 3
#define P11CHANNEL 4
#define P13CHANNEL 5
#define HP1XCHANNEL 6
#define HP1YCHANNEL 7
// //pwm2 (0x42)
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
//
// Servo end points for dome mechanisms
#define ZAPSERVOMIN  350 // adjust Dome Zapper Down position (150 - 600)
#define ZAPSERVOMAX  180 // adjust Dome Zapper Up position
#define PTURNSERVOMIN  180 // adjust Dome Zapper Turn position (150 - 600)
#define PTURNSERVOMAX  450 // adjust Dome Zapper Turn position
#define ZAPTURNSERVOMIN  500 // adjust Dome Zapper Turn position (150 - 600)
#define ZAPTURNSERVOMAX  200 // adjust Dome Zapper Turn position
#define LFTURNSERVOMIN  180 // adjust Dome LF Turn position (150 - 600)
#define LFTURNSERVOMAX  500 // adjust Dome LF Turn position
//
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
//
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
//
//LED high low settings below, connected to the Gnd and Signal line on the PCA9685
#define ZLEDSERVOMIN  200
#define ZLEDSERVOMAX  500 
#define BMLEDSERVOMIN  200
#define BMLEDSERVOMAX  500
#define LFLEDSERVOMIN 200
#define LFLEDSERVOMAX 500
#define PLEDSERVOMIN 200
#define PLEDSERVOMAX 500

//indicies for array
#define ZAP 0
#define ZAPTURN 1
#define PTURN 2
#define LFTURN 3
#define ZLED 4
#define BMLED 5
#define LFLED 6
#define PLED 7
#define PP1 8
#define PP5 9
#define PP6 10
#define P10 11
#define P11 12
#define P13 13
#define HP1_X 14
#define HP1_Y 15
#define PP2 16
#define P1 17
#define P2 18
#define P3 19
#define P4 20
#define P7 21
#define HP2_X 22
#define HP2_Y 23
#define HP3_X 24
#define HP3_Y 25

int panelMap[26][5] = { // [panel name][pwm address][pwm channel][servo min][servo max]
//name      addr  channel         min               max
  {ZAP,     0x40, ZAPCHANNEL,     ZAPSERVOMIN,      ZAPSERVOMAX},
  {ZAPTURN, 0x40, ZAPTURNCHANNEL, ZAPTURNSERVOMIN,  ZAPTURNSERVOMAX},
  {PTURN,   0x40, PTURNCHANNEL,   PTURNSERVOMIN,    PTURNSERVOMAX},
  {LFTURN,  0x40, LFTURNCHANNEL,  LFTURNSERVOMIN,   LFTURNSERVOMAX},
  {ZLED,    0x40, ZLEDCHANNEL,    ZLEDSERVOMIN,     ZLEDSERVOMAX},
  {BMLED,   0x40, BMLEDCHANNEL,   BMLEDSERVOMIN,    BMLEDSERVOMAX},
  {LFLED,   0x40, LFLEDCHANNEL,   LFLEDSERVOMAX,    LFLEDSERVOMAX},
  {PLED,    0x40, PLEDCHANNEL,    PLEDSERVOMAX,     PLEDSERVOMAX},
  {PP1,     0x41, LSPPCHANNEL,    LSSERVOMIN,       LSSERVOMAX}, //Lightsaber
  {PP5,     0x41, BMPPCHANNEL,    BMSERVOMIN,       BMSERVOMAX}, //Bad Motivator
  {PP6,     0x41, ZPPCHANNEL,     ZSERVOMIN,        ZSERVOMAX}, //Zapper
  {P10,     0x41, P10CHANNEL,     P10MIN,           P10MAX},
  {P11,     0x41, P11CHANNEL,     P11MIN,           P11MAX},
  {P13,     0x41, P13CHANNEL,     P13MIN,           P13MAX},
  {HP1_X,   0x41, HP1XCHANNEL,    HP1_XMIN,         HP1_XMAX}, //front hp
  {HP1_Y,   0x41, HP1YCHANNEL,    HP1_YMIN,         HP1_YMAX},
  {PP2,     0x42, LFPPCHANNEL,    LFSERVOMIN,       LFSERVOMAX}, //Lifeform Scanner
  {P1,      0x42, P1CHANNEL,      P1MIN,            P1MAX},
  {P2,      0x42, P2CHANNEL,      P2MIN,            P2MAX},
  {P3,      0x42, P3CHANNEL,      P3MIN,            P3MAX},
  {P4,      0x42, P4CHANNEL,      P4MIN,            P4MAX},
  {P7,      0x42, P7CHANNEL,      P7MIN,            P7MAX},
  {HP2_X,   0x42, HP2XCHANNEL,    HP2_XMIN,         HP2_XMAX}, //back hp
  {HP2_Y,   0x42, HP2YCHANNEL,    HP2_YMIN,         HP2_YMAX},
  {HP3_X,   0x42, HP3XCHANNEL,    HP3_XMIN,         HP3_XMAX}, //top hp
  {HP3_Y,   0x42, HP3YCHANNEL,    HP3_YMIN,         HP3_YMAX}
};
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
long bminterval = 3000; //how long the bad motivator is raised
long bmflashinterval = 200; // time between led flashes

unsigned long lspreviousMillis; //lightsaber store time
long lsinterval = 3000; //how long the lightsaber is raised

unsigned long pturnpreviousMillis; // Periscope store time
long pturninterval = 1200;  // time between posotional turns
byte pturncount = 0; // count how many turns for the periscope

unsigned long lfturnpreviousMillis; // Lifeform scanner store time
long lfturninterval = 600;  // time between posotional turns
byte lfturncount = 0; // count how many turns for the periscope
const long lfledinterval = 500; // time between Lifeform scanner led flashes
unsigned long lfledpreviousmillis = 0; // storage for led flashes

long overloadinterval = 5000; //time delay for overload action

// button timer for debounce if required
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
#pragma endregion

#pragma region States
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
// int buttonPushCounter = 0;    // counter for the number of button presses
// int buttonPushCounter1 = 0;   // counter for the number of button presses
// int buttonPushCounter2 = 0;   // counter for the number of button presses
// int buttonPushCounter3 = 0;   // counter for the number of button presses
// int buttonPushCounter4 = 0;   // counter for the number of button presses
// int buttonPushCounter5 = 0;   // counter for the number of button presses
int ledState = LOW;           // the current state of the output pin
int ledState1 = LOW;          // the current state of the output pin
int ledState2 = LOW;          // the current state of the output pin
// int buttonState = 0;          // the current state of the button for Dome Zapper
// int buttonState1 = 0;         // the current state of the button for Periscope
// int buttonState2 = 0;         // the current state of the button for Lifeform Scanner
// int buttonState3 = 0;         // the current state of the button for Bad Motivator
// int buttonState4 = 0;         // the current state of the button for Lightsaber
// int buttonState5 = 0;         // the current state of the button for Drink Server
// int lastButtonState = 0;      // the previous reading from the input pin
// int lastButtonState1 = 0;     // the previous reading from the input pin
// int lastButtonState2 = 0;     // the previous reading from the input pin
// int lastButtonState3 = 0;     // the previous reading from the input pin
// int lastButtonState4 = 0;     // the previous reading from the input pin
// int lastButtonState5 = 0;     // the previous reading from the input pin
#pragma endregion

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);// serial communication

  // Initialize holos and magic panel and set to default colors
  pixelsHP.begin();
  pixelsHP.setBrightness(holoBrightness);
  pixelsHP.fill(holoColor, 0, NUMPIXELS_HP);
  pixelsMP.begin();
  pixelsMP.setBrightness(magicPanelBrightness);
  pixelsMP.clear();

  pwm0.begin();
  pwm0.setPWMFreq(PWM_FREQ); // standard for analog servos
  pwm1.begin();
  pwm1.setPWMFreq(PWM_FREQ);
  pwm2.begin();
  pwm2.setPWMFreq(PWM_FREQ);

  // set defaults for motor states
  statezapup = ZAP_MOVE_TOP;
  statezapdown = ZAP_BOT;
  statepup = P_MOVE_TOP;
  statepdown = P_BOT;
  statelfup = LF_MOVE_TOP;
  statelfdown = LF_BOT;
  statebmup = BM_MOVE_TOP;
  statebmdown = BM_BOT;
  statelsup = LS_MOVE_TOP;
  statelsdown = LS_BOT;

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
  // pinMode(ledPin, OUTPUT); // led pin on the arduino for testing
  
  // input pins
  pinMode(PTop, INPUT_PULLUP); // pin goes to NO and C goes to ground, value reads LOW when the switch is closed
  pinMode(PBot, INPUT_PULLUP);
  pinMode(BMTop, INPUT_PULLUP);
  pinMode(BMBot, INPUT_PULLUP);
  pinMode(ZTop, INPUT_PULLUP);
  pinMode(ZBot, INPUT_PULLUP);
  pinMode(LSTop, INPUT_PULLUP);
  pinMode(LSBot, INPUT_PULLUP);
  pinMode(LFTop, INPUT_PULLUP);
  pinMode(LFBot, INPUT_PULLUP);
  // pinMode(buttonPin, INPUT_PULLUP);
  // pinMode(buttonPin1, INPUT_PULLUP);
  // pinMode(buttonPin2, INPUT_PULLUP);
  // pinMode(buttonPin3, INPUT_PULLUP);
  // pinMode(buttonPin4, INPUT_PULLUP);
  // pinMode(buttonPin5, INPUT_PULLUP);

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

  servoSetup(); // Set the servos to their start positions
  // SerialOut(); // uncomment for value debugging
  delay(1000); // wait for everything to get to their positions on power up
  #ifdef DOMEMECH_VERBOSE
    Serial.println("Dome board is ready.");
  #endif
}

void loop() {
  //reset timers
  currentMillis = millis();
  readlimits(); //read and store the limit switches values High or Low, function further down in code
  SerialOut();
  if (!executingCommand && Serial.available() > 0) { //if not already executing a command and Serial data is available
    receiveDataFromMainBoard(); //reads any data in serial
  }

  Serial.print("in loop() statelfup = "); Serial.println(statelfup);
  Serial.print("in loop() statelfdown = "); Serial.println(statelfdown);

  // PeriscopeUp();
  // BadMotivatorUp();
  // DomeZapperUp();
  // LightsaberUp();
  Serial.println("***LifeformUp()");
  LifeformUp();
  SerialOut();
  delay(5000);
  Serial.println("***LifeformDown()");
  LifeformDown();
  SerialOut();
  delay(5000);

  //Button functionality
  // buttonState = digitalRead(buttonPin); // main trigger for button inputs
  // buttonState1 = digitalRead(buttonPin1); // main trigger for button inputs
  // buttonState2 = digitalRead(buttonPin2); // main trigger for button inputs
  // buttonState3 = digitalRead(buttonPin3); // main trigger for button inputs
  // buttonState4 = digitalRead(buttonPin4); // main trigger for button inputs
  // buttonState5 = digitalRead(buttonPin5); // main trigger for button inputs
//
  // if (buttonState != lastButtonState) {
  //   if (buttonState == LOW) {
  //     buttonPushCounter++;
  //   }
  // }
//
  // if (buttonState1 != lastButtonState1) {
  //   if (buttonState1 == LOW) {
  //     buttonPushCounter1++;
  //   }
  // }
//
  // if (buttonState2 != lastButtonState2) {
  //   if (buttonState2 == LOW) {
  //     buttonPushCounter2++;
  //   }
  // }
//
  // if (buttonState3 != lastButtonState3) {
  //   if (buttonState3 == LOW) {
  //     buttonPushCounter3++;
  //   }
  // }
//
  // if (buttonState4 != lastButtonState4) {
  //   if (buttonState4 == LOW) {
  //     buttonPushCounter4++;
  //   }
  // }
//
  // if (buttonState5 != lastButtonState5) {
  //   if (buttonState5 == LOW) {
  //     buttonPushCounter5++;
  //   }
  // }
//
  // //Dome Zapper
  // if (buttonPushCounter == 1) {
  //   DomeZapperUp();
  //   if (ZTopVal == LOW && ZBotVal == HIGH) {
  //     DomeZapper();
  //   }
  // }
  // else if (buttonPushCounter == 2) {
  //   pwm0.setPWM(5, 0, ZAPTURNSERVOMIN); //turn the zapper arm to original position
  //   pwm0.setPWM(4, 0, ZAPSERVOMIN); //lower the arm
  //   pwm0.setPWM(8, 0, 4096); // sets the led LOW
  //   DomeZapperDown();
  // }
  // else {
  //   buttonPushCounter = 0;
  //   statezapup = ZAP_MOVE_TOP;  // reset states for next lift sequence
  //   statezapdown = ZAP_MOVE_BOT;
  //   statez = 1;
  //   statezl = 0;
  // }
//
  // //Periscope
  // if (buttonPushCounter1 == 1) {
  //   PeriscopeUp();
  //   if (PTopVal == LOW && PBotVal == HIGH) {
  //     PeriscopeTurn();
  //   }
  // }
  // else if (buttonPushCounter1 == 2) {
  //   pwm0.setPWM(6, 0, PTURNSERVOMIN); //periscope turn to original lift position
  //   PeriscopeDown();
  // }
  // else {
  //   buttonPushCounter1 = 0;
  //   statepup = P_MOVE_TOP;
  //   statepdown = P_MOVE_BOT;
  //   statept = 0;
  // }
//
  // //Lifeform scanner
  // if (buttonPushCounter2 == 1) {
  //   LifeformUp();
  //   if (currentMillis - lfledpreviousmillis >= lfledinterval) {
  //     pwm0.setPWM(10, 4096 , 0); //Lifeform LED HIGH
  //     lfledpreviousmillis = currentMillis;
  //   }
  //   else {
  //     pwm0.setPWM(10, 0, 4096); //Lifeform LED LOW
  //   }
  //   if (LFTopVal == LOW && LFBotVal == HIGH) {
  //     LFTurn();
  //   }
  // }
  // else if (buttonPushCounter2 == 2) {
  //   pwm0.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn to original position
  //   LifeformDown();
  // }
  // else {
  //   buttonPushCounter2 = 0;
  //   statelfup = LF_MOVE_TOP;
  //   statelfdown = LF_MOVE_BOT;
  //   statelft = 0;
  // }

  // delay(5000);
  // Serial.println("BM up");
  // buttonPushCounter3 = 1;
  // delay(5000);
  // Serial.println("BM down");
  // buttonPushCounter3 = 2;
  // delay(5000);
  // Serial.println("BM reset");
  // buttonPushCounter3 = 0;
  // delay(5000);

  //Bad Motivator
  // if (buttonPushCounter3 == 1) {
  //   BadMotivatorUp();
  //   pwm0.setPWM(9, 4096, 0);
  // }
  // else if (buttonPushCounter3 == 2) {
  //   BadMotivatorDown();
  //   pwm0.setPWM(9, 0, 4096);
  // }
  // else {
  //   buttonPushCounter3 = 0;
  //   statebmup = BM_MOVE_TOP;
  //   statebmdown = BM_MOVE_BOT;
  // }
//
  //Lightsaber Lifter
  // if (buttonPushCounter4 == 1) {
  //   LightsaberUp();
  // }
  // else if (buttonPushCounter4 == 2) {
  //   LightsaberDown();
  // }
  // else {
  //   buttonPushCounter4 = 0;
  //   statelsup = LS_MOVE_TOP;
  //   statelsdown = LS_MOVE_BOT;
  // }
  
  // SerialOut(); // print to serial all values for testing
  // lastButtonState = buttonState; // reset the input button
  // lastButtonState1 = buttonState1; // reset the input button
  // lastButtonState2 = buttonState2; // reset the input button
  // lastButtonState3 = buttonState3; // reset the input button
  // lastButtonState4 = buttonState4; // reset the input button
  // lastButtonState5 = buttonState5; // reset the input button
}

void readlimits() { //reads limit swtiches and stores values for compare to end stops in the main loop
  #ifdef DOMEMECH_VERBOSE
    Serial.println("Reading limit switches.");
  #endif
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
  #ifdef DOMEMECH_VERBOSE
    Serial.println("Setting servo start positions.");
  #endif
  //ServoStartPositions();
  pwm0.setPWM(ZAPCHANNEL, 0, ZAPSERVOMIN); //Zapper arm
  pwm0.setPWM(ZAPTURNCHANNEL, 0, ZAPTURNSERVOMIN); //Zapper arm turn
  pwm0.setPWM(PTURNCHANNEL, 0, PTURNSERVOMIN); //Periscope turn
  pwm0.setPWM(LFTURNCHANNEL, 0, LFTURNSERVOMIN); //Lifeform turn
  pwm0.setPWM(ZLEDCHANNEL, 0, 4096); //Zapper LED low
  pwm0.setPWM(BMLEDCHANNEL, 0, 4096); //Bad Motiviator LED low
  pwm0.setPWM(LFLEDCHANNEL, 0, 4096); //Lifeform LED low
  pwm0.setPWM(PLEDCHANNEL, 0, 4096); //Periscope LED low

  pwm1.setPWM(LSPPCHANNEL,LSSERVOMIN,LSSERVOMAX); //PP1
  pwm1.setPWM(BMPPCHANNEL,BMSERVOMIN,BMSERVOMAX); //PP5
  pwm1.setPWM(ZPPCHANNEL,ZSERVOMIN,ZSERVOMAX); //PP6
  pwm1.setPWM(P10CHANNEL,P10MIN,P10MAX); //P10
  pwm1.setPWM(P11CHANNEL,P11MIN,P11MAX); //P11
  pwm1.setPWM(P13CHANNEL,P13MIN,P13MAX); //P13
  pwm1.setPWM(HP1XCHANNEL,HP1_XMIN,HP1_XMAX); //HP1_X
  pwm1.setPWM(HP1YCHANNEL,HP1_YMIN,HP1_YMAX); //HP1_Y

  pwm2.setPWM(LFPPCHANNEL,LFSERVOMIN,LFSERVOMAX); //PP2
  pwm1.setPWM(P1CHANNEL,P1MIN,P1MAX); //P1
  pwm1.setPWM(P2CHANNEL,P2MIN,P2MAX); //P2
  pwm1.setPWM(P3CHANNEL,P3MIN,P3MAX); //P3
  pwm1.setPWM(P4CHANNEL,P4MIN,P4MAX); //P4
  pwm1.setPWM(P7CHANNEL,P7MIN,P7MAX); //P7
  pwm2.setPWM(HP2XCHANNEL,HP2_XMIN,HP2_XMAX); //HP2_X
  pwm2.setPWM(HP2YCHANNEL,HP2_YMIN,HP2_YMAX); //HP2_Y
  pwm2.setPWM(HP3XCHANNEL,HP3_XMIN,HP3_XMAX); //HP3_X
  pwm2.setPWM(HP3YCHANNEL,HP3_YMIN,HP3_YMAX); //HP3_Y
  // Serial.println("done setting up pwm servos");
}

#pragma region CommunicationFunctions
void receiveDataFromMainBoard() {
  #ifdef DOMEMECH_VERBOSE
    Serial.println("Receiving serial data from main board");
  #endif
  String command = Serial.readString(); //pull string from Serial
  if (command.startsWith("CMD:", 0)) {
    executingCommand = true;
    processCommand(command); // get last char of command and parse to int
  } else {
    #ifdef DOMEMECH_DEBUG
      Serial.println("This is not a dome command");
    #endif
  }
}

//define actions for all possible commands from body to dome
void processCommand(String command) {
  #ifdef DOMEMECH_VERBOSE
    Serial.println("in processCommand()");
  #endif
  #ifdef DOMEMECH_DEBUG
    Serial.println("Processing " + command + " command.");
  #endif
  //TODO: Can this be changed to a switch statement for better processing time?
  if (command == "CMD:PERISCOPE") {
      PeriscopeUp();
      if (PTopVal == LOW && PBotVal == HIGH) {
        PeriscopeTurn();
      }
      pwm0.setPWM(PTURNCHANNEL, 0, PTURNSERVOMIN); //periscope turn to original lift position
      PeriscopeDown();

    statepup = P_MOVE_TOP;
    statepdown = P_MOVE_BOT;
    statept = 0;
  } else if (command == "CMD:LIFEFORMSCANNER") {
    LifeformUp();
    if (currentMillis - lfledpreviousmillis >= lfledinterval) {
      pwm0.setPWM(LFLEDCHANNEL, 4096 , 0); //Lifeform LED HIGH
      lfledpreviousmillis = currentMillis;
    } else {
      pwm0.setPWM(LFLEDCHANNEL, 0, 4096); //Lifeform LED LOW
    }
    if (LFTopVal == LOW && LFBotVal == HIGH) {
      LFTurn();
    }
    pwm0.setPWM(LFTURNCHANNEL, 0, LFTURNSERVOMIN); //Lifeform turn to original position
    LifeformDown();

    statelfup = LF_MOVE_TOP;
    statelfdown = LF_MOVE_BOT;
    statelft = 0;
  } else if (command == "CMD:ZAPPER") {
    DomeZapperUp();
    if (ZTopVal == LOW && ZBotVal == HIGH) { //if zapper is raised
      DomeZapper();
    }
    pwm0.setPWM(ZAPTURNCHANNEL, 0, ZAPTURNSERVOMIN); //turn the zapper arm to original position
    pwm0.setPWM(ZAPCHANNEL, 0, ZAPSERVOMIN); //lower the arm
    pwm0.setPWM(ZLEDCHANNEL, 0, 4096); // sets the led LOW
    DomeZapperDown();

    statezapup = ZAP_MOVE_TOP;  // reset states for next lift sequence
    statezapdown = ZAP_MOVE_BOT;
    statez = 1;
    statezl = 0;
  } else if (command == "CMD:BADMOTIVATOR") {
    BadMotivatorUp();
    pwm0.setPWM(BMLEDCHANNEL, 4096, 0); //BM led on
    delay(bminterval); //wait for interval before lowering bad motivator
    BadMotivatorDown();
    pwm0.setPWM(BMLEDCHANNEL, 0, 4096); //BM led off

    statebmup = BM_MOVE_TOP;
    statebmdown = BM_MOVE_BOT;
  } else if (command == "CMD:LIGHTSABER") {
    LightsaberUp();
    delay(lsinterval); //wait for interval before lowering lightsaber
    LightsaberDown();

    statelsup = LS_MOVE_TOP;
    statelsdown = LS_MOVE_BOT;
  } else if (command == "CMD:OVERLOAD") {
    //open all panels

    delay(1000); // wait for all panels to open

    //raise all mechanisms
    PeriscopeUp();
    LifeformUp();
    DomeZapperUp();
    DomeZapper();
    BadMotivatorUp();
    LightsaberUp();

    toggleMagicPanel(); // magic panel on
    setHoloColor(RED); // holos red
    if (!areHolosOn) {
      toggleHoloLights();
    }
    // play scream sound
    delay(overloadinterval);
    setHoloColor(WHITE); // holos normal
    toggleMagicPanel(); // magic panel off

    // lower all mechanisms
    PeriscopeDown();
    LifeformDown();
    DomeZapperDown();
    BadMotivatorDown();
    LightsaberDown();
    delay(1000); // wait for all mechanisms to go down
    
    //close all panels

  } else if (command == "CMD:PANELWAVE") {
    panelWave(); // open panels in sequence
  } else if (command == "CMD:TOGGLEMAGICPANEL") {
    toggleMagicPanel();
  } else if (command == "CMD:TOGGLEHOLOLIGHTS") {
    toggleHoloLights();
  } else if (command == "CMD:TOGGLEHOLOAUTOMATION") {
    toggleHoloAutomation();
  } else if (command.substring(0,20) == "CMD:CHANGEHOLOCOLOR") { // ignore color name in command
    String color = command.substring(20);
    holoColor = getColorFromString(color);
    setHoloColor(holoColor);
  } else {
    Serial.println("*** Unknown dome command");
    executingCommand = false;
    return;
  }
  printAck(command);
  executingCommand = false; //finished executing command
}

void printAck(String command) {
  // Serial.println("in printAck()");
  Serial.print("*** ");
  Serial.print(command);
  Serial.print(" command executed\n");  //print ACK to serial
}
#pragma endregion

#pragma region HoloFunctions
// =======================================================================================
//                          Holo Functions
// =======================================================================================
int getColorFromString(String strColor) {
  for (int i = 0; i < sizeof(colorNames); i++) {
    if ((String)colorNames[i] = strColor) {
      #ifdef DOMEMECH_DEBUG
        Serial.println("Color string: " + strColor + ", colorNames index: " + i);
      #endif
      return i;
    }
  }
  #ifdef DOMEMECH_DEBUG
    Serial.println("Color name not Found!");
  #endif
  return 0;
}

void toggleHoloLights() {
  if (areHolosOn) { // if holos are currently on
    pixelsHP.clear();
    areHolosOn = false;
  } else { // if holos are currently off
    #ifdef DOMEMECH_DEBUG
      Serial.println("Holos are currently off, turning them on with default color.");
    #endif
    setHoloColor(WHITE);
    areHolosOn = true;
  }
}

void setHoloColor(int holoColor) {
  #ifdef DOMEMECH_DEBUG
    Serial.println("Setting Holo color to " + *colorNames[holoColor]);
  #endif
  pixelsHP.fill(holoColor, 0, NUMPIXELS_HP);
  pixelsHP.show();   // Send the updated pixel colors to the hardware.
}

void moveHoloServo(Adafruit_PWMServoDriver pwmNum, int pwmPin, int pulse) {
  pwmNum.setPWM(pwmPin, 0, pulse);
}

void toggleHoloAutomation() {
  if (isHoloAutomationOn) {
    isHoloAutomationOn = false;
    turnHoloAutomationOff();
  } else {
    isHoloAutomationOn = true;
    turnHoloAutomationOn();
  }
}

void turnHoloAutomationOn() {
  currentMillis = millis();
  if (!areHolosOn) {
    toggleHoloLights();
  }

  if (currentMillis > holoFrontRandomTime) {
    holoFrontRandomTime = currentMillis + random(HOLO_DELAY);
    moveHoloServo(domePwm1, HP1XCHANNEL, random(HP1_XMIN,HP1_XMAX));
    moveHoloServo(domePwm1, HP1YCHANNEL, random(HP1_YMIN,HP1_YMAX));
  }
  if (currentMillis > holoBackRandomTime) {
    holoBackRandomTime = currentMillis + random(HOLO_DELAY);
    moveHoloServo(domePwm2, HP2XCHANNEL, random(HP2_XMIN,HP2_XMAX));
    moveHoloServo(domePwm2, HP2YCHANNEL, random(HP2_YMIN,HP2_YMAX));
  }
  if (currentMillis > holoTopRandomTime) {
    holoTopRandomTime = currentMillis + random(HOLO_DELAY);
    moveHoloServo(domePwm2, HP3XCHANNEL, random(HP3_XMIN,HP3_XMAX));
    moveHoloServo(domePwm2, HP3YCHANNEL, random(HP3_YMIN,HP3_YMAX));
  }

  // switch (holoprojector) {
  //   case HOLO_FRONT:   
  //     holoFrontRandomTime = currentMillis + random(HOLO_DELAY);
  //     //TODO:  Determine range of Holoprojector X/Y better
  //         //hpY=random(80,120);
  //         //hpX=random(80,120); 
  //     moveHoloServo(HOLO_FRONT_X_PWM_PIN, random(HOLO_FRONT_X_SERVO_MIN,HOLO_FRONT_X_SERVO_MAX));
  //     moveHoloServo(HOLO_FRONT_Y_PWM_PIN, random(HOLO_FRONT_Y_SERVO_MIN,HOLO_FRONT_Y_SERVO_MAX));
  //     int ledState = random(1,10);
  //     switch(ledState) {
  //       case 3:
  //           holoLightFrontStatus = HOLO_LED_OFF;
  //           holoLightsOff();
  //           break;
  //       case 7:
  //           holoLightFrontStatus = HOLO_LED_ON;
  //           holoLightsOn();
  //           break;
  //       default:
  //           holoLightFrontStatus = HOLO_LED_FLICKER;
  //           break;
  //     }
  //     if (holoLightFrontStatus == HOLO_LED_FLICKER) {
  //         holoLightFlicker(HOLO_FRONT_RED_PWM_PIN, HOLO_FRONT_GREEN_PWM_PIN, HOLO_FRONT_BLUE_PWM_PIN);
  //     }        
  //     break;
  //   case HOLO_BACK:
  //     if (currentMillis > holoBackRandomTime) {  
  //       holoBackRandomTime = currentMillis + random(HOLO_DELAY*1.5);
  //       //TODO:  Determine range of Holoprojector X/Y better
  //           //hpY=random(80,120);
  //           //hpX=random(80,120); 
  //       moveHoloServo(HOLO_BACK_X_PWM_PIN, random(HOLO_BACK_X_SERVO_MIN,HOLO_BACK_X_SERVO_MAX));
  //       moveHoloServo(HOLO_BACK_Y_PWM_PIN, random(HOLO_BACK_Y_SERVO_MIN,HOLO_BACK_Y_SERVO_MAX));
  //       int ledState = random(1,10);
  //       switch(ledState) {
  //         case 4:
  //           holoLightBackStatus = HOLO_LED_OFF;
  //           holoLightsOff();
  //           break;
  //         case 8:
  //           holoLightBackStatus = HOLO_LED_ON;
  //           holoLightsOn();
  //           break;
  //         default:
  //           holoLightBackStatus = HOLO_LED_FLICKER;
  //           break;
  //       }
  //     }
  //     if (holoLightBackStatus == HOLO_LED_FLICKER) {
  //       holoLightFlicker(HOLO_BACK_RED_PWM_PIN, HOLO_BACK_GREEN_PWM_PIN, HOLO_BACK_BLUE_PWM_PIN);
  //     }        
  //     break;
  //   case HOLO_TOP:  
  //     if (currentMillis > holoTopRandomTime) {  
  //       holoTopRandomTime = currentMillis + random(HOLO_DELAY*1.5);
  //       //TODO:  Determine range of Holoprojector X/Y better
  //           //hpY=random(80,120);
  //           //hpX=random(80,120); 
  //       moveHoloServo(HOLO_TOP_X_PWM_PIN, random(HOLO_TOP_X_SERVO_MIN,HOLO_TOP_X_SERVO_MAX));
  //       moveHoloServo(HOLO_TOP_Y_PWM_PIN, random(HOLO_TOP_Y_SERVO_MIN,HOLO_TOP_Y_SERVO_MAX));
  //       int ledState = random(1,10);
  //       switch(ledState) {
  //         case 5:
  //           holoLightTopStatus = HOLO_LED_OFF;
  //           holoLightsOff();
  //           break;
  //         case 8:
  //           holoLightTopStatus = HOLO_LED_ON;
  //           holoLightsOn();
  //           break;
  //         default:
  //           holoLightTopStatus = HOLO_LED_FLICKER;
  //           break;
  //       }
  //     }
  //     if (holoLightTopStatus == HOLO_LED_FLICKER) {
  //       holoLightFlicker(HOLO_TOP_RED_PWM_PIN, HOLO_TOP_GREEN_PWM_PIN, HOLO_TOP_BLUE_PWM_PIN);
  //     }        
  //     break;
  // }
}

void turnHoloAutomationOff() {
  moveHoloServo(domePwm1, HP1XCHANNEL, HOLO_SERVO_CTR);
  moveHoloServo(domePwm1, HP1YCHANNEL, HOLO_SERVO_CTR);

  moveHoloServo(domePwm2, HP2XCHANNEL, HOLO_SERVO_CTR);
  moveHoloServo(domePwm2, HP2YCHANNEL, HOLO_SERVO_CTR);

  moveHoloServo(domePwm2, HP3XCHANNEL, HOLO_SERVO_CTR);
  moveHoloServo(domePwm2, HP3YCHANNEL, HOLO_SERVO_CTR);
}
#pragma endregion

#pragma region MagicPanelFunctions
// =======================================================================================
//                          Magic Panel Functions
// =======================================================================================
void toggleMagicPanel() {
  if (isMagicPanelOn) { // if magic panel is currently on
    pixelsMP.clear();
    isMagicPanelOn = false;
  } else { // if magic panel is currently off
    #ifdef DOMEMECH_DEBUG
      Serial.println("Magic Panel is currently off, turning it on.");
    #endif
    setMagicPanelColor(RED);
    isMagicPanelOn = true;
  }
}

void setMagicPanelColor(int magicPanelColor) {
  #ifdef DOMEMECH_DEBUG
    Serial.println("Setting Holo color to " + *colorNames[magicPanelColor]);
  #endif
  pixelsMP.fill(magicPanelColor, 0, NUMPIXELS_MP);
  pixelsMP.show();   // Send the updated pixel colors to the hardware.
}
#pragma endregion

#pragma region DomeFunctions
// =======================================================================================
//                          Dome Functions
// =======================================================================================
void panelWave() {
  //Panel order 10, 11, 13, 1, 2, 3, 4, PP2, PP1, PP6, PP5
  #ifdef DOMEMECH_DEBUG
    Serial.println("Waving movable dome panels.");
  #endif
  int panels[] = {P10, P11, P13, P1, P2, P3, P4, PP2, PP1, PP6, PP5};
  for (int i = 0; i < sizeof(panels); i++) {
    openPanel(panels[i]);
  }
  for (int i = 0; i < sizeof(panels); i++) {
    closePanel(panels[i]);
  }
  for (int i = sizeof(panels); i >= 0; i--) {
    openPanel(panels[i]);
  }
  for (int i = sizeof(panels); i >= 0; i--) {
    closePanel(panels[i]);
  }
}

void openPanel(int panelName) {
  #ifdef DOMEMECH_DEBUG
    Serial.println("Opening panel " + panelName);
  #endif
  if (getPwmAddress(panelName) == 0x40) {
    pwm0.setPWM(getPwmChannel(panelName), 0, panelMap[panelName][4]);
  } else if (getPwmAddress(panelName) == 0x41) {
    pwm1.setPWM(getPwmChannel(panelName), 0, panelMap[panelName][4]);
  } else if (getPwmAddress(panelName) == 0x42) {
    pwm2.setPWM(getPwmChannel(panelName), 0, panelMap[panelName][4]);
  }
}

void closePanel(int panelName) {
  #ifdef DOMEMECH_DEBUG
    Serial.println("Closing panel " + panelName);
  #endif
  if (getPwmAddress(panelName) == 0x40) {
    pwm0.setPWM(getPwmChannel(panelName), 0, panelMap[panelName][3]);
  } else if (getPwmAddress(panelName) == 0x41) {
    pwm1.setPWM(getPwmChannel(panelName), 0, panelMap[panelName][3]);
  } else if (getPwmAddress(panelName) == 0x42) {
    pwm2.setPWM(getPwmChannel(panelName), 0, panelMap[panelName][3]);
  }
}

void panelWaveHello() {
  #ifdef DOMEMECH_DEBUG
    Serial.println("Waving hello with panel 11.");
  #endif
  //wave panel 11 or 13?
  openPanel(P11); //open/max
  closePanel(P11); //close/min
  openPanel(P11); //open/max
  closePanel(P11); //close/min
}

void panelDance() {
  #ifdef DOMEMECH_DEBUG
    Serial.println("Running panel dance sequence.");
  #endif
  //open/close in sequence with cantina dance
}

int getPwmAddress(int panelName) {
  return panelMap[panelName][1];
}

int getPwmChannel(int panelName) {
  return panelMap[panelName][2];
}

// void automateDome() {
//   //TODO: review automate dome code, add auto home?
//   //automate dome movement
//   if (isAutomateDomeOn) {
//     long rndNum;
//     int domeSpeed;
//     if (domeStatus == 0) { // Dome is currently stopped - prepare for a future turn
//       if (domeTargetPosition == 0) { // Dome is currently in the home position - prepare to turn away
//         domeStartTurnTime = millis() + (random(3, 10) * 1000);
//         rndNum = random(5,354);
//         domeTargetPosition = rndNum;  // set the target position to a random degree of a 360 circle - shaving off the first and last 5 degrees
//         if (domeTargetPosition < 180) { // Turn the dome in the positive direction
//           domeTurnDirection = 1;
//           domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurnRight);
//         } else { // Turn the dome in the negative direction
//           domeTurnDirection = -1;
//           domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurnLeft);
//         }
//       } else { // Dome is not in the home position - send it back to home
//         domeStartTurnTime = millis() + (random(3, 10) * 1000);
//         if (domeTargetPosition < 180) {
//           domeTurnDirection = -1;
//           domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurnLeft);
//         } else {
//           domeTurnDirection = 1;
//           domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurnRight);
//         }
//         domeTargetPosition = 0;
//       }
//       domeStatus = 1;  // Set dome status to preparing for a future turn
//       #ifdef SHADOW_DEBUG
//         output += "Dome Automation: Initial Turn Set\r\n";
//         output +=  "Current Time: ";
//         output +=  millis();
//         output += "\r\n Next Start Time: ";
//         output += domeStartTurnTime;
//         output += "\r\n";
//         output += "Next Stop Time: ";
//         output += domeStopTurnTime;
//         output += "\r\n";          
//         output += "Dome Target Position: ";
//         output += domeTargetPosition;
//         output += "\r\n";          
//       #endif
//     }
//     if (domeStatus == 1) { // Dome is prepared for a future move - start the turn when ready
//       if (domeStartTurnTime < millis()) {
//         domeStatus = 2; 
//         #ifdef SHADOW_DEBUG
//           output += "Dome Automation: Ready To Start Turn\r\n";
//         #endif
//       }
//     }
//     if (domeStatus == 2) { // Dome is now actively turning until it reaches its stop time
//       if (domeStopTurnTime > millis()) {
//         domeSpeed = domeAutoSpeed * domeTurnDirection;
//         SyR->motor(domeSpeed);
//         #ifdef SHADOW_DEBUG
//           output += "Turning Now!!\r\n";
//         #endif
//       } else { // turn completed - stop the motor
//         domeStatus = 0;
//         SyR->stop();
//         #ifdef SHADOW_DEBUG
//           output += "STOP TURN!!\r\n";
//         #endif
//       }
//     }
//   }
// }
#pragma endregion

#pragma region MechFunctions
////////////////////////////////////////////////////////////
// Functions below here for features
////////////////////////////////////////////////////////////
void DomeZapperUp() { // this function is for the dome zapper
  switch (statezapup) {
    case ZAP_MOVE_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the dome zapper to the top.");
      #endif
      if (ZTopVal != LOW) {
        if (digitalRead(ZTop) == HIGH && digitalRead(ZBot) == LOW) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Opening dome zapper pie panel.");
          #endif
          Serial.print("opening zapper pie panel");
          pwm1.setPWM(ZPPCHANNEL, 0, ZSERVOMAX); // open the pie panel
        }
        digitalWrite(ZIN1, HIGH); //turn the dc motor on
        Serial.println("zapper motor on");
        digitalWrite(ZIN2, LOW);
        statezapup = ZAP_TOP;
      }
      break;
    case ZAP_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Dome zapper is at the top.");
      #endif
      if (ZTopVal == LOW) {
        digitalWrite(ZIN1, LOW); //turn the motor off
        digitalWrite(ZIN2, LOW); //turn the motor off
        pwm0.setPWM(ZAPCHANNEL, 0, ZAPSERVOMAX); //lift zapper arm
      }
      break;
  }
}

void DomeZapperDown() { // this function is for the dome zapper
  switch (statezapdown) {
    case ZAP_MOVE_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the dome zapper to the bottom.");
      #endif
      if (ZBotVal != LOW) {
        digitalWrite(ZIN1, LOW); //turn the dc motor on
        digitalWrite(ZIN2, HIGH);
        statezapdown = ZAP_BOT;
      }
      break;
    case ZAP_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Dome zapper is at the bottom.");
      #endif
      if (ZBotVal == LOW) {
        digitalWrite(ZIN1, LOW); //turn the dc motor on
        digitalWrite(ZIN2, LOW);
        if (digitalRead(ZBot) == LOW && digitalRead(ZTop) == HIGH) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Closing dome zapper pie panel.");
          #endif
          pwm1.setPWM(ZPPCHANNEL, 0, ZSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}

void DomeZapper() { //lift zapper arm servo, flash light, rotate to new position and flash, return to first position, arm down
  switch (statez) {
    case 1:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Lifting the dome zapper arm.");
      #endif
      currentMillis = millis();
      pwm0.setPWM(ZAPCHANNEL, 0, ZAPSERVOMAX); //lift zapper arm
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        statez = 2;
        zapturnpreviousMillis = currentMillis;  
      }
      break;
    case 2:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning the dome zapper arm to MAX.");
      #endif
      currentMillis = millis();
      pwm0.setPWM(ZAPTURNCHANNEL, 0, ZAPTURNSERVOMAX); //turn zapper arm
      ZapLed(); // flash the LED
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        statez = 3;
        zapturnpreviousMillis = currentMillis;
      }
      break;
    case 3:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning the dome zapper arm to MIN.");
      #endif
      currentMillis = millis();
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        pwm0.setPWM(ZAPTURNCHANNEL, 0, ZAPTURNSERVOMIN); //turn
        statez = 0;
        zapturnpreviousMillis = currentMillis;
      }
      break;
  }
}

void ZapLed() {
  switch (statezl) {
    case 0:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning on the dome zapper LED.");
      #endif
      currentMillis = millis();
      pwm0.setPWM(ZLEDCHANNEL, 4096, 0); // sets the led HIGH from the PCA9685
      if (currentMillis - zappreviousMillis >= zapinterval) {
        zappreviousMillis = currentMillis;
        statezl = 1;
      }
      break;
    case 1:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning off the dome zapper LED.");
      #endif
      currentMillis = millis();
      pwm0.setPWM(ZLEDCHANNEL, 0, 4096); // sets the led LOW from the PCA9685
      if (currentMillis - zappreviousMillis >= zapinterval) {
        zappreviousMillis = currentMillis;
        statezl = 2;
      }
      break;
    case 2:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Flashing on the dome zapper LED.");
      #endif
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
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the periscope to the top.");
      #endif
      if (PTopVal != LOW) {
        digitalWrite(PIN1, HIGH); //turn the dc motor on
        digitalWrite(PIN2, LOW);
        statepup = P_TOP;
        togglePLed();
      }
      break;
    case P_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Periscope is at the top.");
      #endif
      if (PTopVal == LOW) {
        pwm0.setPWM(PLEDCHANNEL, 4096, 0); // sets the led HIGH from the PCA9685
        digitalWrite(PIN1, LOW); //turn the motor off
        digitalWrite(PIN2, LOW); //turn the motor off
      }
      break;
  }
}

void PeriscopeDown() { // this function lowers the periscope
  switch (statepdown) {
    case P_MOVE_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the periscope to the bottom.");
      #endif
      if (PBotVal != LOW) {
        pwm0.setPWM(PLEDCHANNEL, 0, 4096); // sets the led LOW from the PCA9685
        digitalWrite(PIN1, LOW); //turn the dc motor on
        digitalWrite(PIN2, HIGH);
        statepdown = P_BOT;
      }
      break;
    case P_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Periscope is at the bottom.");
      #endif
      if (PBotVal == LOW) {
        pwm0.setPWM(PLEDCHANNEL, 0, 4096); // sets the led LOW from the PCA9685
        digitalWrite(PIN1, LOW); //turn the dc motor on
        digitalWrite(PIN2, LOW);
        togglePLed();
      }
      break;
  }
}

void PeriscopeTurn() { // turn the periscope back and forwards
  switch (statept) {
    case 0:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning the periscope to MAX.");
      #endif
      currentMillis = millis();
      if (currentMillis - pturnpreviousMillis >= pturninterval) {
        pwm0.setPWM(PTURNCHANNEL, 0, PTURNSERVOMAX); //turn
        pturnpreviousMillis = currentMillis;
        statept = 1;
      }
      break;
    case 1:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning the periscope to MIN.");
      #endif
      currentMillis = millis();
      if (currentMillis - pturnpreviousMillis >= pturninterval) {
        pwm0.setPWM(PTURNCHANNEL, 0, PTURNSERVOMIN); //turn
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
      } else {
        statept = 0;
      }
      break;
  }
}

void togglePLed() {
  if (isPLedOn) {
    #ifdef DOMEMECH_DEBUG
      Serial.println("Periscope LED is on, turning it off.");
    #endif
    pwm0.setPWM(PLEDCHANNEL, 0, 4096); // sets the led LOW from the PCA9685
  } else {
    #ifdef DOMEMECH_DEBUG
      Serial.println("Periscope LED is off, turning it on.");
    #endif
    pwm0.setPWM(PLEDCHANNEL, 4096, 0); // sets the led HIGH from the PCA9685
  }
}


void LifeformUp() {
  Serial.print("statelfup = "); Serial.println(statelfup);
  Serial.print("statelfdown = "); Serial.println(statelfdown);
  switch (statelfup) {
    case LF_MOVE_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the lifeform scanner to the top.");
      #endif
      if (LFTopVal != LOW) {
        if (digitalRead(LFTop) == HIGH && digitalRead(LFBot) == LOW) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Opening lifeform scanner pie panel.");
          #endif
          pwm2.setPWM(LFPPCHANNEL, 0, LFSERVOMAX); // open the pie panel
        }
        digitalWrite(LFIN1, HIGH); //turn the dc motor on
        digitalWrite(LFIN2, LOW);
        statelfup = LF_TOP;
        statelfdown = LF_MOVE_BOT;
        Serial.print("statelfup = "); Serial.println(statelfup);
        Serial.print("statelfdown = "); Serial.println(statelfdown);
        toggleLFLed();
      }
      break;
    case LF_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Lifeform scanner is at the top.");
      #endif
      if (LFTopVal == LOW) {
        digitalWrite(LFIN1, LOW); //turn the motor off
        digitalWrite(LFIN2, LOW); //turn the motor off
      }
      break;
  }
}

void LifeformDown() {
  Serial.print("statelfup = "); Serial.println(statelfup);
  Serial.print("statelfdown = "); Serial.println(statelfdown);
  switch (statelfdown) {
    case LF_MOVE_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the lifeform scanner to the bottom.");
      #endif
      if (LFBotVal != LOW) {
        digitalWrite(LFIN1, LOW); //turn the dc motor on
        digitalWrite(LFIN2, HIGH);
        statelfdown = LF_BOT;
        statelfup = LF_MOVE_TOP;
        Serial.print("statelfup = "); Serial.println(statelfup);
        Serial.print("statelfdown = "); Serial.println(statelfdown);
      }
      break;
    case LF_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Lifeform scanner is at the bottom.");
      #endif
      if (LFBotVal == LOW) {
        digitalWrite(LFIN1, LOW); //turn the dc motor off
        digitalWrite(LFIN2, LOW);
        if (digitalRead(LFBot) == LOW && digitalRead(LFTop) == HIGH) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Closing lifeform scanner pie panel.");
          #endif
          pwm2.setPWM(LFPPCHANNEL, 0, LFSERVOMIN); // close the pie panel
        }
        toggleLFLed();
      }
      break;
  }
}

void LFTurn() {
  switch (statelft) {
    case 0:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning the lifeform scanner to MAX.");
      #endif
      currentMillis = millis();
      if (currentMillis - lfturnpreviousMillis >= lfturninterval) {
        pwm0.setPWM(LFTURNCHANNEL, 0, LFTURNSERVOMAX); //Lifeform turn
        lfturnpreviousMillis = currentMillis;
        statelft = 1;
      }
      break;
    case 1:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Turning the lifeform scanner to MIN.");
      #endif
      currentMillis = millis();
      if (currentMillis - lfturnpreviousMillis >= lfturninterval) {
        pwm0.setPWM(LFTURNCHANNEL, 0, LFTURNSERVOMIN); //Lifeform turn
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

void toggleLFLed() {
  if (isLFLedOn) {
    #ifdef DOMEMECH_DEBUG
      Serial.println("Lifeform LED is on, turning it off.");
    #endif
    pwm0.setPWM(LFLEDCHANNEL, 0, 4096); // sets the led LOW from the PCA9685
  } else {
    #ifdef DOMEMECH_DEBUG
      Serial.println("Lifeform LED is off, turning it on.");
    #endif
    pwm0.setPWM(LFLEDCHANNEL, 4096, 0); // sets the led HIGH from the PCA9685
  }
}


void BadMotivatorUp() {
  switch (statebmup) {
    case BM_MOVE_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the bad motivator to the top.");
      #endif
      if (BMTopVal != LOW) {
        if (digitalRead(BMTop) == HIGH && digitalRead(BMBot) == LOW) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Opening bad motivator pie panel.");
          #endif
          pwm1.setPWM(BMPPCHANNEL, 0, BMSERVOMAX); // open the pie panel
        }
        digitalWrite(BMIN1, HIGH); //turn the dc motor on
        digitalWrite(BMIN2, LOW);
        statebmup = BM_TOP;
        toggleBadMotivatorLed();
      }
      break;
    case BM_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Bad motivator is at the top.");
      #endif
      if (BMTopVal == LOW) {
        digitalWrite(BMIN1, LOW); //turn the motor off
        digitalWrite(BMIN2, LOW); //turn the motor off
      }
      break;
  }
}

void BadMotivatorDown() {
  switch (statebmdown) {
    case BM_MOVE_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the bad motivator to the bottom.");
      #endif
      if (BMBotVal != LOW) {
        digitalWrite(BMIN1, LOW); //turn the dc motor on
        digitalWrite(BMIN2, HIGH);
        statebmdown = BM_BOT;
      }
      break;
    case BM_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Bad motivator is at the bottom.");
      #endif
      if (BMBotVal == LOW) {
        digitalWrite(BMIN1, LOW); //turn the dc motor on
        digitalWrite(BMIN2, LOW);
        if (digitalRead(BMBot) == LOW && digitalRead(BMTop) == HIGH) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Cosing bad motivator pie panel.");
          #endif
          pwm1.setPWM(BMPPCHANNEL, 0, BMSERVOMIN); // close the pie panel
        }
        toggleBadMotivatorLed();
      }
      break;
  }
}

void toggleBadMotivatorLed() {
  if (isBMLedOn) {
    #ifdef DOMEMECH_DEBUG
      Serial.println("Bad Motivator LED is on, turning it off.");
    #endif
    pwm0.setPWM(BMLEDCHANNEL, 0, 4096); // sets the led LOW from the PCA9685
  } else {
    #ifdef DOMEMECH_DEBUG
      Serial.println("Bad Motivator LED is off, turning it on.");
    #endif
    pwm0.setPWM(BMLEDCHANNEL, 4096, 0); // sets the led HIGH from the PCA9685
  }
}


void LightsaberUp() {
  switch (statelsup) {
    case LS_MOVE_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the lightsaber to the top.");
      #endif
      if (LSTopVal != LOW) {
        if (digitalRead(LSTop) == HIGH && digitalRead(LSBot) == LOW) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Opening lightsaber pie panel.");
          #endif
          pwm1.setPWM(LSPPCHANNEL, 0, LSSERVOMAX); // open the pie panel
        }
        digitalWrite(LSIN1, HIGH); //turn the dc motor on
        digitalWrite(LSIN2, LOW);
        statelsup = LS_TOP;
      }
      break;
    case LS_TOP:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Lightsaber is at the top.");
      #endif
      if (LSTopVal == LOW) {
        digitalWrite(LSIN1, LOW); //turn the motor off
        digitalWrite(LSIN2, LOW); //turn the motor off
      }
      break;
  }
}

void LightsaberDown() {
  switch (statelsdown) {
    case LS_MOVE_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Moving the lightsaber to the bottom.");
      #endif
      if (LSBotVal != LOW) {
        digitalWrite(LSIN1, LOW); //turn the dc motor on
        digitalWrite(LSIN2, HIGH);
        statelsdown = LS_BOT;
      }
      break;
    case LS_BOT:
      #ifdef DOMEMECH_DEBUG
        Serial.println("Lightsaber is at the bottom.");
      #endif
      if (LSBotVal == LOW) {
        digitalWrite(LSIN1, LOW); //turn the dc motor on
        digitalWrite(LSIN2, LOW);
        if (digitalRead(LSBot) == LOW && digitalRead(LSTop) == HIGH) {
          #ifdef DOMEMECH_DEBUG
            Serial.println("Closing lightsaber pie panel.");
          #endif
          pwm1.setPWM(LSPPCHANNEL, 0, LSSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}
#pragma endregion

//Function to display values for testing, uncheck any values you don't want to or do want to see
void SerialOut() {
  Serial.println("------------------------------------------"),
  // Serial.println("Button Push Counters");
  //   Serial.println("B0\tB1\tB2\tB3\tB4\tB5");
  //   Serial.print(buttonPushCounter); Serial.print("\t");
  //   Serial.print(buttonPushCounter1); Serial.print("\t");
  //   Serial.print(buttonPushCounter2); Serial.print("\t");
  //   Serial.print(buttonPushCounter3); Serial.print("\t");
  //   Serial.print(buttonPushCounter4); Serial.print("\t");
  //   Serial.print(buttonPushCounter5); Serial.print("\n\n");
  // Serial.println("Button States");
  //   Serial.println("St0\tSt1\tSt2\tSt3\tSt4\tSt5");
  //   Serial.print(buttonState); Serial.print("\t");
  //   Serial.print(buttonState1); Serial.print("\t");
  //   Serial.print(buttonState2); Serial.print("\t");
  //   Serial.print(buttonState3); Serial.print("\t");
  //   Serial.print(buttonState4); Serial.print("\t");
  //   Serial.print(buttonState5); Serial.print("\n\n");
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
  Serial.println("------------------------------------------\n");
  
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
//  Serial.print("statezl:"); Serial.print(statezl); Serial.print("\t");
//  Serial.print("Millis:"); Serial.print(currentMillis); Serial.println("\t");
//  Serial.print("-------------------------------------------\n\n");
}
