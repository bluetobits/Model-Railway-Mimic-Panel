//pointControlBassic.ino Steve Lomax 2021 not for commercial use. Adafruit acknowledgement below
//please acnowledge author if redistributing or modifing
//storing servo microseconds rather than degrees
//
// up to 16 servos to pwm board
//Rotary enc for point calibrtionon D2&D3 with switch on D4
//i2c on A4 & A5 for 16 servo PWM Adafruit PCA9685  module .

/*
   using a 4x4 matrix of 16 x (on-off)switches. Each switch has a 10K pull-up resistor and a diode
   D6, D7, D8 & D9   

   D6 ──────┬───0──────┬───0─────────┬───0───────┬───0
            10k▓    /        10k▓    /           10k ▓    /         10k▓    /
        Diode  ▓   /     Diode  ▓   /        Diode   ▓   /      Diode  ▓   /
      ┌─┤<├─┴─0sw0 ┌─┤<├─┴─0 sw1   ┌──┤<├─┴─0sw2 ┌──┤<├─┴─0 sw3      
      │            \   │            \      │             \   │             \
      │             \  │             \     │              \  │              \
      │  0v ──────0──────────0─────────────0───────────0──>  
      │                │                   │                 │
   D7───────┬───0───────┬───0───────┬───0───────┬───0
      │     10k▓    /  │     10k▓    /    │    10k▓    /   │     10k▓    /
      │        ▓   /   │        ▓   /     │       ▓   /    │        ▓   /
      ├─|<├──┴─0sw4 ├──|<├─┴─0 sw5  ├──|<├┴─0 sw6  ├──|<├─┴─0 sw7      
      │            \    │            \     │            \    │            \
      │             \   │             \    │             \   │             \
      │  0v ──────0───────────0───────────0───────────0──>
      │                 │                  │                 │
   D8───────┬───0───────┬───0───────┬───0───────┬───0
      │     10k▓    /  │     10k▓    /    │    10k▓    /   │     10k▓    /
      │        ▓   /   │        ▓   /     │       ▓   /    │        ▓   /
      ├─|<├──┴─0 sw8├──|<├─┴─0 sw9  ├──|<├┴─0sw10 ├──|<├─┴─0 sw11      
      │            \    │            \     │            \   │            \
      │             \   │             \    │             \  │             \
      │  0v ──────0───────────0───────────0───────────0──>
      │                 │                  │                │
      │        ^        │       ^                 ^                  ^
   D9───────┬───0───────┬───0───────┬───0───────┬───0
      │     10k▓    /  │     10k▓    /    │   10k▓    /   │     10k▓    /
      │        ▓   /   │        ▓   /     │      ▓   /    │        ▓   /
      ├─|<├──┴─0sw12├──|<├─┴─0 sw13 ├──|<├┴─0sw14 ├──|<├─┴─0 sw15      
      │            \    │            \     │            \   │            \
      │             \   │             \    │             \  │             \
      │  0v ──────0───────────0───────────0───────────0──>
      │                 │                  │                │
      A0                A1                  A2                A3

      0  1  2  3
      4  5  6  7
      8  9  10 11
      12 13 14 15


      prototype used DPDT switches with LED to indicate control was
      switched to serial RS-485 input from PC
   Parts
   Arduino uno,nano etc &  breakout board
   PCA9685  PWM (servo driver) module
   up to 16 x SPST 
   up to 16 x servos
   2 x 2k resistor
   16 x 10K resistor
   4 x 100K reststor
   2 x 200R resistor
   16 x 1N2007 diode or similar
   5V DC 1A PSU for electronics
   5V DC 2A PSU for servos 
   servo mounting hardware
   servo knob (alomst essential)
   switch panel for mounting hardware
   module mounting hardware
   rotary encoder module
   2 LEDs
   connecting wire
   reset button
    power jack socket
   

    Arduino PIN connections
   
    3 => Encoder DT
    4 => Encoder CLK (reverse 3&4 if operation is backward)
    5 => Encoder push switch (to GND)
    6 => switch matrix row 0 (sw0-3 point closed)
    7 => switch matrix row 1 (sw4-7 point closed)
    8 => switch matrix row 2 (sw8-11 point closed)
    9 => switch matrix row 3 (sw12-15 point closed)
    10 => PCA9685 OE(ENABLE)
    11 => NOT USED
    12 => LED (PCA9685 ACTIVE - POINTS MOVING) via 200R resistor
    13 => LED (CALIBRATING - ENCODER ACTIVE) via 200R resistor
    A0 => switch matrix COL 0 (sw 0,4,8,12) + 100K pullup resistor
    A1 => switch matrix COL 1 (sw 1,5,9,13) + 100K pullup resistor
    A2 => switch matrix COL 2 (sw 2,6,10,14) + 100K pullup resistor
    A3 => switch matrix COL 3 (sw 3,7,11,15) + 100K pullup resistor
    A4 => PCA9685 SDA + 2K pullup resistor
    A5 => PCA9685 SCL + 2K pullup resistor
    A6 => NOT USED
    A7 => NOT USED
    RST => RESET PUSH SWITCH TO GROUND
    GND => COMMON GROUND ALL SUPPLIES ALL MODULES
    VCC => 5V REG INPUT, to encoder VCC, PCA9685 VCC
    RAW/IN =>ALTERNATIVE 6-12V INPUT

    note
    the sketch and hardware counts switches, channels and servos starting at 0
    (e.g. 16 switches numbered 0 to 15) the 1st channel is channel 0

   Operation and features :
 * *points (turnouts) can be thrown/closed manually 
   The red light will extinguish after each power-up or reset when points have moved to
      current switch positions
 * *point movement may be interrupted by switching the swtch at ant time
 * *switching a switch to the closed position will move the respective point to the
       internally stored closed position for that point
   switching a switch to the thrown position will move the respective point to the
      internally stored thrown position for that point
 * *The internally stored thrown and closed positions can be adjusted in real time without reprogramming

 
   CALIBRATING POINT POSITIONS
   to enter calibration, press and hold dowm the encoder button for around 2 seconds until the
      calibration LED lights steady.
   The LED will briefly flash to signify the initial press and then stay lit if calibration mode is successful.
   The last switch moved (which may be changed by JMRI) will be the point currently being calibrated
   Moving the encoder will change the current point position of the last switch moved for that switch position (thrown or closed)
   Changing a switch (manually or by JMRI) will make that point the current point under calibration.
   Previously recalibrated point positions will be temporarly stored for the remainder of the calibration mode.
   It is possible to set a closed point position to a thrown switch position and vice-versa.
      (useful if the mainline is on a thrown point)
   One, many or all points may be calibrated and adjusted any number of times in one calibrating session.
   There are 2 ways to exit from calibration mode:
    1) a long encoder press will store all new positions to internal memory and exit calibration all previous positions will be permanently overwritten
    2) a short press will keep the new positions only until the device is reset or restarted. The previously stored positions will then be loaded and the new positions deleted
   point moving speed is maximum when calibrating.


NOTE The servo enable feature of the PCA9865 was found to sometimes cause servos to jitter when disabled. 
This is subject to local environment inteference. If it becaomes an issue then disconnect the pin from the PWM module.   
  
*/

#include <Encoder.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>


#define SERVO_ENABLE_PIN 10// connect to module en
const bool DEBUG_SERIAL = 0;
bool DEBUG = 0;
Encoder encoder(3, 4);


Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

int row[] = {6, 7, 8, 9};
int col[] = {A0, A1, A2, A3};
const int ACTIVE = 12;
const int LED = 13;
int i;
const int ROT_ENC_SW_PIN = 5;//rotary encoder switch  for calibrating
int pointVal[16][3];// turnout [Number 0-16] [0=servo value for the Closed position, 1=servo val for Thrown, 2=Current value]
int incomingStatus[32];// will be 0 or 1 unless corrupt
const int TOP_PULSE_LEN = 2400;
const int BOTTOM_PULSE_LEN = 600;
bool mimicSwitch[16];// 0 = switch closed, 1 = switch thrown,
unsigned int newSwitchStatus = -999;// initialise a dummy value
unsigned int oldSwitchStatus;
int lastPointMoved;
bool moving;
int increment = 10;// the distance the point blades will move each program loop 
unsigned long moveDelay = 10000;
boolean cal;
bool pointChanged;
bool lastPointPos;
unsigned long moveTimeout;
unsigned long disableTimeout;
unsigned long disableDelay = 1500;// delay before servos are disabled after moving
int newEncPosition;
int oldEncPosition;
bool advPush = false;
unsigned long push;
bool refresh;

unsigned long flashTimeout;
bool flash;


// ===============================MEMORY WRITE=============================
// each EEPROM memory address can store a byte (8 bits. An integer is 2 Bytes so 2 addresses are needed
void memoryWrite(byte addr, int data) {
  EEPROM.write(addr, data >> 8);//high address  
  EEPROM.write(addr + 100, (data & 0xFF));//low address is 100 above the high address
  
  delay(20);
}

// ===============================MEMORY READ=============================
int memoryRead(byte addr) {
  int  rdata = 0xFF;
  rdata = EEPROM.read(addr);
  rdata = (rdata << 8) + (EEPROM.read(addr + 100) & 0xFF);
  return rdata;
}


//============== calibrate ==============
void calibrate() {
  if (cal) {

    if (pointChanged) {
      pointChanged = false;// drop changed flag
      encoder.write(pointVal[lastPointMoved][lastPointPos]);// set encoder to value of pos of last mimicSw moved

    }
    newEncPosition = encoder.read();
    if (newEncPosition > TOP_PULSE_LEN) { //cap over range and reset encoder
      newEncPosition = TOP_PULSE_LEN;
      encoder.write(newEncPosition);
    }
    if (newEncPosition < BOTTOM_PULSE_LEN) { //cap under range and reset encoder
      newEncPosition = BOTTOM_PULSE_LEN;
      encoder.write(newEncPosition);
    }

    if (newEncPosition != oldEncPosition) { // update encoder position
      oldEncPosition = newEncPosition;
      pointVal[lastPointMoved][lastPointPos] = newEncPosition;// store new pos for switch & position to array
      if (DEBUG) {
        Serial.print("Point ");
        Serial.print(lastPointMoved);
        Serial.print(" position ");
        Serial.print(lastPointPos);
        Serial.print(" EncPosition ");
        Serial.println(newEncPosition);
      }
      servo.writeMicroseconds(lastPointMoved, newEncPosition);
    }
    //test for exit calibration routine
    if ( digitalRead(ROT_ENC_SW_PIN) == LOW) {
      digitalWrite(LED, LOW);//flash led
      delay(100);
      digitalWrite(LED, HIGH);
      unsigned long calPressTime = millis();//start timer for enc button press
      while (digitalRead(ROT_ENC_SW_PIN) == LOW) {//while button is held in
        if (calPressTime + 2000 < millis()) { // for more tnan 2 seconds
          digitalWrite(LED, LOW);
          for (i = 0; i < 16; i++) {// write all to eeprom
            memoryWrite(i, pointVal[i][0]); // closed value
            memoryWrite(i + 20, pointVal[i][1]); // Thrown value
          }
          while (digitalRead(ROT_ENC_SW_PIN) == LOW) {} //wait for releade button is held in
          delay(30); //debounce
        }
      }
      cal = false;
      delay(30); //debounce
      digitalWrite(LED, LOW);
      digitalWrite(SERVO_ENABLE_PIN, HIGH); //disable servos
      digitalWrite(ACTIVE, LOW);
    }

  } else { //not cal
    if (digitalRead(ROT_ENC_SW_PIN) == LOW) {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      unsigned long calPressTime = millis();//start timer for enc button press
      while (digitalRead(ROT_ENC_SW_PIN) == LOW) {//while button is held in
        if (calPressTime + 2000 < millis()) { // for more tnan 2 seconds
          digitalWrite(LED, HIGH);
          while (digitalRead(ROT_ENC_SW_PIN) == LOW) {} //while button is held in
          delay(30); //debounce
          cal = true;// start calibrating

          digitalWrite(SERVO_ENABLE_PIN, LOW); //enable servo driver
          digitalWrite(ACTIVE, HIGH);

          if (DEBUG) {
            Serial.print("calibrating ");
            //Serial.print(lastPointMoved);
            //Serial.print(" position ");
            //Serial.println(mimicSwitch[lastPointMoved]);
          }
          encoder.write(pointVal[lastPointMoved][2]);// set encoder to current value of pos of last mimicSw moved
        }
      }
    }
  }
}

//================ GET SWITCHES =========================
void getSwitches() {
  int j;
  int switchNo;
  int switchValue;

  newSwitchStatus = 0;

  for (i = 0; i < 4; i++) {

    digitalWrite(row[i], HIGH);//for each i row high
    for (j = 0; j < 4; j++) {// read each j col
      switchNo = 4 * i + j;//allocating switch Nos to matrix rows & cols

      mimicSwitch[switchNo] = 0;// initialise point for current switch closed
      switchValue = analogRead(col[j]);// get the analog value
      
      if (switchValue < 200) mimicSwitch[switchNo] = 1; // switch is low so make point thrown

      newSwitchStatus = newSwitchStatus << 1;// this sets a 16 bit integer with each bit representing the position of each point
      newSwitchStatus += mimicSwitch[switchNo];





      if ((newSwitchStatus & 1) != (oldSwitchStatus >> (15 - switchNo) & 1)) {
        lastPointMoved = switchNo;
        lastPointPos = mimicSwitch[switchNo];
        pointChanged = true;


      }
    }
    digitalWrite(row[i], LOW);//return row low
  }


  if (oldSwitchStatus != newSwitchStatus) { // a switch changed. need to move
   
    digitalWrite(SERVO_ENABLE_PIN, LOW);
    digitalWrite(ACTIVE, HIGH);
    moving = true;
    oldSwitchStatus = newSwitchStatus;

  }
}
//========================MOVEPOINTS===============================
void movePoints() {
 
  if (disableTimeout > millis())  moving = false;
  for (i = 0; i < 16; i++) {
    int dtg = pointVal[i][2] - pointVal[i][mimicSwitch[i]]; //distance to go = current pos - target

    if (dtg  < 0) { //need to increase
      pointVal[i][2] += increment;
      if (pointVal[i][2] > pointVal[i][mimicSwitch[i]]) { // if new current pos met or moved past final switch value
        pointVal[i][2] = pointVal[i][mimicSwitch[i]];//set new current position to final position
      }
      servo.writeMicroseconds(i, pointVal[i][2] );//move servo
      moving = true;
      disableTimeout = millis() + disableDelay;

    }
    if (dtg  > 0) { //need to decrease
    
      pointVal[i][2] -= increment;
      if (pointVal[i][2] < pointVal[i][mimicSwitch[i]]) { // if new current pos met or moved past final switch value

        pointVal[i][2] = pointVal[i][mimicSwitch[i]];//set new current position to final position
      }
      servo.writeMicroseconds(i, pointVal[i][2] );//move servo
      moving = true;
      disableTimeout = millis() + disableDelay;

    }
  }
  if (moving) {

    moveTimeout = micros() + moveDelay;// reset moving speed timeout

  } else {
    digitalWrite(SERVO_ENABLE_PIN, HIGH); // no points are moving so turn off points
    digitalWrite(ACTIVE, LOW);

  }


}

void advanced() {
  if (digitalRead(ROT_ENC_SW_PIN) == 0) {
    digitalWrite(LED, LOW);
    delay (3000);
    advPush = false;
    if (DEBUG)  Serial.println (advPush);
    memoryWrite(16, moveDelay);

  } else {
    moveDelay = encoder.read() * 100;
    digitalWrite(LED, HIGH);
    if (DEBUG)   Serial.println (moveDelay);

  }
}
// =============================== SETUP =============================

void setup() {

  servo.begin();
  servo.setPWMFreq(50);
  yield();// wait for pwm to acknowledge it is ready

  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  pinMode(ACTIVE, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LED, OUTPUT);//CAL LED
  pinMode(ROT_ENC_SW_PIN, INPUT_PULLUP);
  for (int k = 0; k < 4; k++) {
    pinMode(row[k], OUTPUT);
    pinMode(col[k], INPUT);
  }
  moveDelay = memoryRead (16);// the speed the points will move
  if (moveDelay > 100000)moveDelay = 100000;
  if (moveDelay < 1000)moveDelay = 1000;
  push = millis() + 2000;

  // ADVANCED MODE most of the anvanced move features have been stripped out of this wersion.
  //NOTE it is NOT POSSIBLE to leave advanced mode without restarting (resetting) the arduino.
  while (digitalRead(ROT_ENC_SW_PIN) == 0) {// while encoder pushed in during powerup or reset
    if (millis() > push) {// if held in for more than 2 seconds
      advPush = true;// 
      encoder.write(moveDelay / 100); //load the encoder with the current move delay speed. 
      digitalWrite(LED, HIGH);
      if (millis() > push + 2000) DEBUG = 1;// if still held in for another 2 seconds turn debug on.
      advPush = true;
      delay(100);

    }
  }
  if (DEBUG) Serial.begin(115200);
  if (DEBUG)   Serial.println ("PointControlBasic Steve Lomax");

  if (DEBUG)   Serial.println ("getSwitches");
  getSwitches();

  digitalWrite(SERVO_ENABLE_PIN, LOW );
  digitalWrite(ACTIVE, HIGH );
  if (DEBUG)   Serial.println ("move points to switch memory");
  for (int k = 0; k < 16; k++) {
    pointVal[k][0] = memoryRead(k); // servo closed value
    pointVal[k][1] = memoryRead(k + 20); // servo Thrown value
    // setting defaults if data not present or out of range
    if (pointVal[k][0] > TOP_PULSE_LEN || pointVal[k][0] < BOTTOM_PULSE_LEN) pointVal[k][0] = 1000;
    if (pointVal[k][1] > TOP_PULSE_LEN || pointVal[k][1] < BOTTOM_PULSE_LEN) pointVal[k][1] = 2000;
    pointVal[k][2] = pointVal[k][mimicSwitch[k]];// Set val of each point position to val of switch position
    
    servo.writeMicroseconds(k, pointVal[k][2]);
   
  }

  delay (1000);
  digitalWrite(SERVO_ENABLE_PIN, HIGH);
  digitalWrite(ACTIVE, LOW);



  if (DEBUG)   {
    Serial.print ("moveDelay= ");
    Serial.println (moveDelay);
  }
  if (DEBUG) Serial.println ("done setup");
}


// =============================== LOOP =============================
// ==================================================================
void loop() {
  getSwitches();
  if (advPush) {
    advanced();
  } else {
    calibrate();
  }
  if (!cal && moving && micros() > moveTimeout) movePoints();
 

}
