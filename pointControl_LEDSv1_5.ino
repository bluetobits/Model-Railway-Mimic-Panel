// pointControl_LEDS.ino Steve Lomax 2021 not for commercial use. Adafruit acknowledgement below
// please acknowledge author if redistributing or modifying
// I know it is bad coding practice but i have liberally commented to give a novice programmer
// a chance of getting some degree of understanding of my inexperienced and inefficient coding.
// RS485 module DI-> TXO, DE+RE-> 2, RO->RXI
// up to 16 servos to PWM module  board
// Rotary enc for point calibration on pins 3&4 with switch on pin 5
// i2c on A4 & A5 for 16 servo PWM Adafruit PCA9685  module .
// Up to 32 addressable LEDs for mimic (prototype utilises fibre optic light-pipe for panel "indicator lamps")
/*
   using a 4x4 matrix of 16 x double throw centre off (on-off-on)switches.

   D6 ──────┬───0──────┬───0──>
            10k▓    /        10k▓    /
        Diode  ▓   /     Diode  ▓   /
      ┌──|<|─┼─0 sw0┌──|<|─┼─0 sw1      sw2 & sw3 < Ditto for A2 & A3
      │        ▓   \   │       ▓   \
      │     10k▓    \  │    10k▓    \
      │  0v ──┴───0──────┴───0──>
      │                 │
   D7──────┬───0───────┬───0──>
      │    10k▓    /   │     10k▓    /
      │Diode  ▓   /    │ Diode  ▓   /
      ├──|<|─┼─0 sw4 ├──|<|─┼─0 sw5     sw6 & sw7 < Ditto for A2 & A3
      │        ▓   \   │        ▓   \
      │     10k▓    \  │     10k▓    \
      │  0v ──┴───0──────┴───0──>
      │        ^        │       ^                 ^                  ^
   D8───>  ditto       │     ditto         ^   ditto          ^   ditto
      │   for D8 & D9   │   for D8          │  for D8         │for D8
      │  sw8            │      sw 9         │    sw10         │  sw11
   D9 ───>             │      & D9         │    & D9         │  & D9
      │  sw12           │      sw13         │    sw14         │  sw15
      A0                A1                   A2                A3

      0  1  2  3
      4  5  6  7
      8  9  10 11
      12 13 14 15


      prototype used DPDT switches with LED to indicate control was
      switched to serial RS-485 input from PC
   Parts
   Arduino uno, nano etc &  breakout board
   USB TO  RS-485 module
   RS-485 to UART module
   PCA9685  PWM (servo driver) module
   up to 16 x DPSTCO OR *DPDTCO toggle switches thre prototype uses the other half to LED indicate control was switched to JMRI.
   up to 16 x servos
   2 x 2k resistor
   32 x 10K resistor
   4 x 100K resistor
   2 x 200R resistor
   16 x 1N2007 diode or similar
    5V DC 1A PSU for electronics
   5V DC 2A PSU for servos & WS2812B LEDs (preferred to common PSU)
   servo mounting hardware
   servo knob (almost essential)
   switch panel for mounting hardware
   module mounting hardware
   rotary encoder module
   2 different colour LEDs
   connecting wire
   reset button
    power jack socket
    led strip WS2812B 2 LEDs per point
    2 metres 2mm fibre optic light pipe for led to mimic
 * *optional

    Arduino PIN connections
    0 (TX) => RS-485 DI
    1 (RX) => RS-485 RO
    2 => RS-485 DE&RE (linked)
    3 => Encoder DT
    4 => Encoder CLK (reverse 3&4 if operation is backward)
    5 => Encoder push switch (to GND)
    6 => switch matrix row 0 (sw0-3 point closed)
    7 => switch matrix row 1 (sw4-7 point closed)
    8 => switch matrix row 2 (sw8-11 point closed)
    9 => switch matrix row 3 (sw12-15 point closed)
    10 => PCA9685 OE(ENABLE)
    11 => WS2812B LED strip Data in
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
    A suitable 5V servo supply is required.


    the sketch and hardware counts switches, channels and servos starting at 0
    (e.g. 16 switches numbered 0 to 15) the 1st channel is channel 0

   Operation and features :
 * *points (turnouts) can be thrown/closed manually or from JMRI software using C/MRI over RS-485
   The red light will extinguish after each power-up or reset when points have moved to
      current switch positions
 * *point movement may be interrupted by switching the swtch at ant time
 * *switching a switch to the closed position will move the respective point to the
       internally stored closed position for that point
   switching a switch to the thrown position will move the respective point to the
      internally stored thrown position for that point
 * *The internally stored thrown and closed positions can be adjusted in real time without reprogramming
 * *The servo motors are switched off between point movements to eliminate servo chatter.
 * *the point movement speed can be aj=djusted by changing MOVE_DELAY value from 12000 higher for slower and lower for faster
 * *changing INCREMENT from 10 higher will move points faster but less smooth, lower will be slower and smoother
   The Arduino has only one serial port. ALL DEBUG flags must be set to 0 to allow RS-485 data.
 * *Advanced mode: There is no indication that the user has selected advanced mode other than "strange" behaviour. Press and hol

   CALIBRATING POINT POSITIONS
   to enter calibration, press and hold down the encoder button for around 2 seconds until the
      calibration LED lights steady.
   The LED will briefly flash to signify the initial press and then stay lit if calibration mode is successful.
   The last switch moved (which may be changed by JMRI) will be the point currently being calibrated
   Moving the encoder will change the current point position of the last switch moved for that switch position (thrown or closed)
   Changing a switch (manually or by JMRI) will make that point the current point under calibration.
   Previously recalibrated point positions will be temporarily stored for the remainder of the calibration mode.
   It is possible to set a closed point position to a thrown switch position and vice-versa.
      (useful if the mainline is on a thrown point)
   One, many or all points may be calibrated and adjusted any number of times in one calibrating session.
   There are 2 ways to exit from calibration mode:
    1) a long encoder press will store all new positions to internal memory and exit calibration all previous positions will be permanently overwritten
    2) a short press will keep the new positions only until the device is reset or restarted. The previously stored positions will then be loaded and the new positions deleted
   point moving speed is maximum when calibrating.

  Fastled led :  A WS2812B 32 LED strip Data connected to pin 11 for mimic pointy positions:
  2 LEDS per point, SEE COMMENTS IN "SET LEDS" BELOW
*/

#include <Encoder.h>
#include <CMRI.h>
#include <Auto485.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include <FastLED.h>

#define DATA_PIN 11 //LED data pin
#define NUM_LEDS 26// NUMBER OF WS2812b leds
#define DE_PIN 2 //for Auto485.h 
#define ROT_ENCA_PIN 4// Rotary encoder A
#define ROT_ENCB_PIN 3// Rotary encoder B
#define CMRI_ADDR 0// This will be CMRI module 0
#define SERVO_ENABLE_PIN 10// connect to PWM module en
const bool DEBUG_SERIAL = 0;
bool DEBUG = 0;
const int NO_OF_POINTS = 13;
// Arduino has only 1 serial data connection we can't send debug text to the PC and send data to JMRI

Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 24, 48, bus); // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs
Encoder encoder(3, 4);
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
CRGB leds[NUM_LEDS];  //LED array

// ledsMimic[] order of LEDs for each point leg 0 closed, 0 thrown, 1 closed, 1 thrown, 2 closed, 2 thrown etc... 
int ledsMimic[] = {2, 1, 25, 24, 23, 0, 3, 22, 21, 4, 20, 5, 6, 8, 19, 7, 18, 17, 9, 16, 15, 14, 10, 11, 12, 13};

int row[] = {6, 7, 8, 9};
int col[] = {A0, A1, A2, A3};
const int ACTIVE = 12;// point servos acive led
const int LED = 13; // calibrate LED
int i;
const int ROT_ENC_SW_PIN = 5;//rotary encoder switch  for calibrating
int pointVal[16][3];// a 2 dimension array of 16 points each with 3 different position values.
/*   Holds 3 values for 16 turnouts.(48 values in total)
      the point [Number] which is  0-16 and
      the [position value] for that point when it is
      Closed(in location 0), Thrown(in location 1), and its Current value(in location 2)
 *    */
int incomingStatus[32];// the data stream from JMRI via CMRI. will be 0 or 1 unless corrupt
const int TOP_PULSE_LEN = 2400;// setting the maximum cw servo position(actual = 2500 but not all servos are the same)
const int BOTTOM_PULSE_LEN = 600;//setting the minimum ccw servo position
bool mimicSwitch[16];// 0 = closed point, 1 = thrown point,
unsigned int newSwitchStatus = -999;//initialised less than 0
unsigned int oldSwitchStatus;
int lastPointMoved;// remembering the last point moved for calibratong
bool moving;// the points are moving at this moment
int increment = 10;// the distance the points will move for each move delay
unsigned long moveDelay = 10000; // the delay for each increment. these are microseconds.
boolean cal;// flag for "currently calibrating"
bool pointChanged; // flag for noring a point has changed
bool lastPointPos;// the position the last point moved was in
unsigned long moveTimeout; // used with moveDelay to trigger the next increment movement
unsigned long disableTimeout;// timeout for disableDelay
unsigned long disableDelay = 1500;// delay before servos are disabled after moving
int newEncPosition;// after the encoder was turned
int oldEncPosition;// before the encoder was turned
bool advPush = false; //advanced features like timing. not yet fully implemented
unsigned long push;// monitors the length of time the encoder is pushed in during startup to enable advanced mode.
bool refresh;
uint8_t onHue = 255;// LED primary hue value
uint8_t onLev = 200;// LED brightness level
uint8_t onSat = 255;// LED colour saturation level
unsigned long flashTimeout;//led flash timeout
bool flash;// flash status 1=on, 0=off.
int calIncrement = 5; // increment per encoder notch 600-2400
int servoEnableActive;

// ===============================MEMORY WRITE=============================
// writes calibrated point positions to the Arduino's internal 'disc drive'
void memoryWrite(byte addr, int data) {
  EEPROM.write(addr, data >> 8);//high address
  /*if (DEBUG) {
    Serial.print("Write high ");
    Serial.print(addr);
    Serial.print(" Data: ");
    Serial.println(data >> 8);

    }

  */
  EEPROM.write(addr + 100, (data & 0xFF));//low address
  /*
    if (DEBUG) {
    Serial.print("Write low ");
    Serial.print(addr + 100);
    Serial.print(" Data: ");
    Serial.print(data & 0xFF);
    Serial.print(" =  total Data: ");
    Serial.println(data);

    }
  */
  delay(20);
}

// ===============================MEMORY READ=============================
int memoryRead(byte addr) {
  int  rdata = 0xFF;
  rdata = EEPROM.read(addr);
  //if (rdata > 180) rdata = 0;
  /*if (DEBUG) {
    Serial.print("Read high from ")  ;
    Serial.print(addr);
    Serial.print(". data = ");
    Serial.print(rdata);
    Serial.print(". low from ");
    Serial.print(addr + 100);
    Serial.print(". data=  ");
    }
  */
  rdata = (rdata << 8) + (EEPROM.read(addr + 100) & 0xFF);
  //if (rdata > 180) rdata = 0;
  //if (rdata < 0) rdata = 0;
  /*if (DEBUG) {
    Serial.print(rdata & 0xFF);
    Serial.print(". Total data = ");
    Serial.println(rdata);
    Serial.println();
    }
  */
  return rdata;
}

// ================Get?set Data to and from JMRI========================
void getData() {
  if (!DEBUG_SERIAL) {

    cmri.process();// get JMRI data via CMRI bits

    for (i = 0; i < 16; i++) {// just using 16 outputs
      incomingStatus[i] = cmri.get_bit(i); // get new incoming status for point positions
      if (incomingStatus[i] > 1) {// it won't be but you never know
        incomingStatus[i] = 1;
      }

      cmri.set_bit(i , mimicSwitch[i] ); //set CMRI bits
    }

  }
}
//============== calibrate ==============
void calibrate() {
  if (cal) {// if already calibrating

    if (pointChanged) {
      pointChanged = false;// drop changed flag
      encoder.write(pointVal[lastPointMoved][lastPointPos] / calIncrement); // set encoder to value of pos of last mimicSw moved

    }
    newEncPosition = encoder.read() * calIncrement;
    if (newEncPosition >= TOP_PULSE_LEN) { //cap over range and reset encoder
      newEncPosition = TOP_PULSE_LEN;
      encoder.write(newEncPosition / calIncrement);
      oldEncPosition = newEncPosition;
      digitalWrite(LED, flash);
    }
    if (newEncPosition <= BOTTOM_PULSE_LEN) { //cap under range and reset encoder
      newEncPosition = BOTTOM_PULSE_LEN;
      encoder.write(newEncPosition / calIncrement);
      oldEncPosition = newEncPosition;
      digitalWrite(LED, flash);
    }

    if (newEncPosition != oldEncPosition) { // update encoder position
      oldEncPosition = newEncPosition;
      pointVal[lastPointMoved][lastPointPos] = newEncPosition;// store new pos for switch & position to array
      digitalWrite(LED, HIGH);
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
      if (servoEnableActive) {
        digitalWrite(SERVO_ENABLE_PIN, HIGH); //disable servos
        digitalWrite(ACTIVE, LOW);
      }
    }

  } else { //not calibrating
    if (digitalRead(ROT_ENC_SW_PIN) == LOW) {// pushed
      digitalWrite(LED, HIGH);//flash led
      delay(100);
      digitalWrite(LED, LOW);
      unsigned long calPressTime = millis();//start timer for enc button press
      while (digitalRead(ROT_ENC_SW_PIN) == LOW) {//while button is held in
        if (calPressTime + 2000 < millis()) { // for more tnan 2 seconds
          digitalWrite(LED, HIGH);
          while (digitalRead(ROT_ENC_SW_PIN) == LOW) {} //while button is held in
          delay(30); //debounce
          cal = true;// start calibrating
          if (servoEnableActive) {
            digitalWrite(SERVO_ENABLE_PIN, LOW); //enable servo driver
            digitalWrite(ACTIVE, HIGH);
          }

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
//================ SET LEDS =========================
// sends data via pin 11 to the strip of 32 leds
void setLeds() {
  if (millis() > flashTimeout + 400) {// every 400ms change flash on to flash off or vice versa
    flash = !flash;// whatever flash state is (on or off), make it the opposite
    flashTimeout = millis();//reset the timer
  }
  onSat = 255;// degree of colour saturation. 255 = full saturated colour 0 = white
  onLev = 100;// brightness level of LEDs. 0 = off 255 max brightness. change this to suite
  // colour hue is rainbow red.or,gn.bl.ind,viol,red between0 (red) and 255(back to red)
  // see https://github.com/FastLED/FastLED/wiki/FastLED-HSV-Colors
  for (int k = 0; k < NO_OF_POINTS; k++) {
    // 2 leds for each point 0 (through/closed ) & 1 (branch/thrown)  for point 0, 1&2 for point 1, 2&3 for point 2 etc
    //    if (mimicSwitch[k] == 1) { // set through to red and branch to green
    //      leds[2 * ledsMimic[k*2]] = CHSV(0, onSat, onLev);
    //      leds[ledsMimic[(2 * k) + 1]] = CHSV(90, onSat, onLev);
    //    } else {
    //      leds[2 * k] = CHSV(90, onSat, onLev);// set through to green and branch to red
    //      leds[(2 * k) + 1] = CHSV(0, onSat, onLev);
    //    }
    //  }
    if (mimicSwitch[k] == 1) { // set through to red and branch to green

      leds[ledsMimic[2 * k]] = CHSV(0, onSat, onLev);
      leds[ledsMimic[2 * k + 1]] = CHSV(90, onSat, onLev);
    } else {
      leds[ledsMimic[2 * k]] = CHSV(90, onSat, onLev);// set through to green and branch to red
      leds[ledsMimic[2 * k + 1]] = CHSV(0, onSat, onLev);
    }
  }
  if (!cal) {
    onLev = 200 + (55 * flash);
    onHue = 35 + (!flash * 55);

    // this section was written specifically for the prototype layout.
    //  if the position one point will cause derailment without flipping another. The leds for those pointe will flash yellow.
    //  eg in the line below, if point 0 is thrown and point 2 is closed then
    //  the point 1 branch LED will flash. likewise the reverse is true in the line below that.
    if (mimicSwitch[0] == 1 && mimicSwitch[2] == 0) leds[ledsMimic[1]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[2] == 1 && mimicSwitch[0] == 0) leds[ledsMimic[5]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[3] == 1 && mimicSwitch[10] == 0) leds[ledsMimic[7]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[10] == 1 && mimicSwitch[3] == 0) leds[ledsMimic[21]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[6] == 0 && mimicSwitch[7] == 0) leds[ledsMimic[12]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[7] == 1 && mimicSwitch[6] == 1) leds[ledsMimic[15]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[8] == 1 && mimicSwitch[9] == 0) leds[ledsMimic[17]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[9] == 1 && mimicSwitch[8] == 0) leds[ledsMimic[19]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[11] == 0 && mimicSwitch[12] == 0) leds[ledsMimic[22]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[12] == 1 && mimicSwitch[11] == 1) leds[ledsMimic[25]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[8] == 1 && mimicSwitch[10] == 0) leds[ledsMimic[20]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[8] == 0 && mimicSwitch[10] == 1) leds[ledsMimic[16]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[4] == 1 && mimicSwitch[3] == 0) leds[ledsMimic[6]] = CHSV(onHue, onSat, onLev);
    if (mimicSwitch[4] == 0 && mimicSwitch[3] == 1) leds[ledsMimic[8]] = CHSV(onHue, onSat, onLev);

    if (moving && flash) {
      leds[ledsMimic[lastPointMoved * 2 + 1]] = CHSV(160, onSat, 255);// bright blue whilst point is moving
      leds[ledsMimic[lastPointMoved * 2 ]] = CHSV(160, onSat, 255);

    }
  } else {
    leds[ledsMimic[lastPointMoved * 2 + lastPointPos]] = CHSV(200, onSat, onLev);//bright purple whilst calibrating
  }

  FastLED.show(); //sends the current leds[array] values to the leds. leds will remain at that colour until the next .show() command.


}
//================ GET SWITCHES =========================
void getSwitches() {
  int j;
  int switchNo;
  int switchValue;
  newSwitchStatus = 0;
  for (i = 0; i < 4; i++) {// taking a row at a time
    digitalWrite(row[i], HIGH);//make the current row high (row {0 to 3]
    for (j = 0; j < 4; j++) {// read each j col
      switchNo = 4 * i + j;//allocating switch Nos to matrix rows & cols
      mimicSwitch[switchNo] = 0;// initialise point for current switch closed
      switchValue = analogRead(col[j]);// get the analog value
      if (switchValue > 500) {//switch is high, point value should be from serial 485
        mimicSwitch[switchNo] = incomingStatus[switchNo]; //make turnout to incoming auto
      }
      if (switchValue < 200) mimicSwitch[switchNo] = 1; // switch is low so make point thrown
      newSwitchStatus = newSwitchStatus << 1;//shifts the binary value of the newswitchstatus 1 place left
      newSwitchStatus += mimicSwitch[switchNo]; // adds the value of the current switch to the end of the newSwitchStatus

      if ((newSwitchStatus & 1) != (oldSwitchStatus >> (15 - switchNo) & 1)) {// shift each bit and check for change
        lastPointMoved = switchNo;
        lastPointPos = mimicSwitch[switchNo];
        pointChanged = true;
      }
    }
    digitalWrite(row[i], LOW);//return row low
  }


  if (oldSwitchStatus != newSwitchStatus) { // a switch changed. need to move point
    if (DEBUG) {

      //          Serial.print (oldSwitchStatus, BIN);
      //          Serial.println (" old ");
      for (int m = 15; m > 0; m--) {
        if (newSwitchStatus >> m == 0) {
          Serial.print ("0");
        } else {
          m = 0;
        }
      }
      Serial.print (newSwitchStatus, BIN);
      Serial.print (" new and last point moved= ");
      Serial.println (lastPointMoved);
    }
    if (servoEnableActive) {
      digitalWrite(SERVO_ENABLE_PIN, LOW);
      digitalWrite(ACTIVE, HIGH);
    }
    moving = true;
    oldSwitchStatus = newSwitchStatus;

  }
}
//========================MOVEPOINTS===============================
void movePoints() {

  if (disableTimeout > millis())  moving = false;// if nothing has moved for a while,

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
      //      if (DEBUG) {
      //        Serial.print (i);
      //        Serial.print (" ");
      //        Serial.print (dtg);
      //        Serial.print (" ");
      //        Serial.print (pointVal[i][2]);
      //        Serial.print (" ");
      //        Serial.print (pointVal[i][mimicSwitch[i]]);
      //        Serial.println ();
      //      }
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
    if (servoEnableActive) {
      digitalWrite(SERVO_ENABLE_PIN, HIGH); // no points are moving so turn off points
      digitalWrite(ACTIVE, LOW);
    }

  }


}
// =============================== ADVANCED =============================
void advanced() {// not yet fully implemented but sort-of working
  // Activate/ deactivate servo enable after timeout
  digitalWrite (ACTIVE, servoEnableActive);       // set ACTIVE led to current status
  if (digitalRead(ROT_ENC_SW_PIN) == 0) {         // if pushed
    push = millis() + 2000;                       // start timer
    while (digitalRead(ROT_ENC_SW_PIN) == 0) {}   // wait for button release
    if (millis() > push) {                        // if button was held for more than push
      servoEnableActive = !servoEnableActive;     // change servoEnableActive status
      digitalWrite(ACTIVE, servoEnableActive);    // set ACTIVE led to current status
      memoryWrite (17, servoEnableActive);        // Write status to EEPROM
      if (DEBUG && servoEnableActive)   Serial.println  ("servo Enable Active");
      if (DEBUG && !servoEnableActive)  Serial.println  ("servo Enable Inactive");

    } else {                                      // if just ashort press
      delay (200);                                // debounce
      advPush = false;                            // turn off advanced
      if (DEBUG)  {
        Serial.print ("Saving Move Delay = ");
        Serial.println (moveDelay);
        Serial.print ("Saving servo enable = ");
        if (!servoEnableActive)  Serial.print ("in");
        Serial.print ("active.");
      }
      memoryWrite(16, moveDelay);           // write current moveDelay to EEPROM
      memoryWrite(17, servoEnableActive);   // write current servoEnableActive to EEPROM

    }
  } else { // not pressed the encoder button
    unsigned long oldmoveDelay = moveDelay;
    moveDelay = encoder.read() * 100; // set moveDelay to encoder position
    digitalWrite(LED, HIGH); //Keep the LED on
    if (DEBUG && moveDelay != oldmoveDelay)   Serial.println (moveDelay);

  }
}
// =============================== SETUP =============================

void setup() {
  // initialising the PCA9865
  servo.begin();
  servo.setPWMFreq(50);
  yield();
  // Initialising the LEDstrip
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, 32);
  // setting up arduino pins
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  pinMode(ACTIVE, OUTPUT);
  pinMode(LED, OUTPUT);
  //pinMode(LED, OUTPUT);//CAL LED
  pinMode(ROT_ENC_SW_PIN, INPUT_PULLUP);
  for (int k = 0; k < 4; k++) {
    pinMode(row[k], OUTPUT);
    pinMode(col[k], INPUT);
  }

  //reading the move speed values from memory. The move Delay is the delay in nanoseconds between each
  moveDelay = memoryRead(16);
  servoEnableActive = memoryRead (17); // reading active status of servo timeout
  if (servoEnableActive > 1) servoEnableActive = 1;
  if (moveDelay > 100000)moveDelay = 100000;// setting a slow limit
  if (moveDelay < 1000)moveDelay = 1000;//setting a fast limit
  push = millis() + 2000; //for detecting a long button press
  while (digitalRead(ROT_ENC_SW_PIN) == 0) {
    if (millis() > push) {
      advPush = true;// 2 second long push. Now selected advanced mode
      encoder.write(moveDelay / 100);// set the encoder to the current move speed.
      digitalWrite(LED, HIGH);
      if (millis() > push + 2000) DEBUG = 1;//4 second long push
      advPush = true;
      delay(100);

    }
  }



  bus.begin(19200);
  //if (DEBUG) Serial.begin(19200);
  if (DEBUG)   Serial.println ("pointControl_LEDS Steve Lomax");

  if (DEBUG)   Serial.println ("getSwitches");
  getSwitches();
  if (DEBUG && servoEnableActive)   Serial.println  ("servo Enable Active");
  if (DEBUG && !servoEnableActive)  Serial.println  ("servo Enable Inactive");

  digitalWrite(SERVO_ENABLE_PIN, LOW );
  digitalWrite(ACTIVE, HIGH );
  if (DEBUG)   {
    Serial.println ("point | closed val | thrown val | current val");
    Serial.println ("------|------------|------------|------------");
  }
  for (int k = 0; k < 16; k++) {
    pointVal[k][0] = memoryRead(k); // closed value
    pointVal[k][1] = memoryRead(k + 20); // Thrown value
    //if values never set then load default values
    if (pointVal[k][0] > TOP_PULSE_LEN || pointVal[k][0] < BOTTOM_PULSE_LEN) pointVal[k][0] = 1000;
    if (pointVal[k][1] > TOP_PULSE_LEN || pointVal[k][1] < BOTTOM_PULSE_LEN) pointVal[k][1] = 2000;
    pointVal[k][2] = pointVal[k][mimicSwitch[k]];// Set val of each point position to val of switch position

    servo.writeMicroseconds(k, pointVal[k][2]);// move points to start-up switch positions
    if (DEBUG)   {
      Serial.print (k);
      Serial.print ("       ");
      Serial.print (pointVal[k][0]);
      Serial.print ("       ");
      Serial.print (pointVal[k][1]);
      Serial.print ("       ");
      Serial.println (pointVal[k][2]);
    }
  }

  delay (1000);
  if (servoEnableActive) {
    digitalWrite(SERVO_ENABLE_PIN, HIGH);
    digitalWrite(ACTIVE, LOW);
  }


  if (DEBUG)   {
    Serial.print ("moveDelay= ");
    Serial.println (moveDelay);
    if (advPush)Serial.print ("Anvanced Setting Mode. \nShort press enc to exit. \nrotate to set speed, \nlong press to toggle servo disable");
  }
  if (DEBUG) Serial.println ("\ndone setup");
}


// =============================== LOOP =============================
// ==================================================================
// The main program that loops (very) fast
void loop() {
  getData();// from JMRI. also sends to JMRI manual operations as sensor changes
  getSwitches();// scan all the switches for Thrown, Closed or Auto(JMRI controlled)
  if (advPush) {
    advanced();// additional features such as throw timing and servo enabling/disabling
  } else {
    calibrate();// calibrate and store point positions for closed and thrown
  }
  if (!cal && moving && micros() > moveTimeout) movePoints();// move any points not at target position necessary
  setLeds(); //update mimic LEDS


}
