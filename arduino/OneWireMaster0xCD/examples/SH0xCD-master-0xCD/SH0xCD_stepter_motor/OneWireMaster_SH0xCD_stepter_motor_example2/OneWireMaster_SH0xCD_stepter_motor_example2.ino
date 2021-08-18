#include <OneWire.h>
//#include "OneWireMaster0xCD.h"
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
//
// Exmaple description:
// EXAMPLE HOW-TO USE STEPTER MOTOR WITH SEAHU 1-Wire MOTOR DRIVER
// ---------------------------------------------------------------
//  in this example repeate move motor to 500 steps into left wait to end moving and then move 500 steps into right and wait do end moving for next repeate
//
// Type device: 1-Wire
// Family code : 0xCD
// Device description: SEAHU 1-Wire stepter motor driver C2020 by Ing. Ondrej Lycka
// Number sections: 15
//      section0 :  MoveRelative       - movving relative distance in steps
//                                        -2147483646 steps => never ending move to left
//                                        +2147483646 steps => never ending move to right
//      section1 :  MoveAbsolute       - movving absolute distance in steps
//      section2 :  Speed              - moving speed in steps/sec
//      section3 :  Accel              - moving acceleration in steps/(sec*sec)
//      section4 :  Decel              - moving deceleration in steps/(sec*sec)
//      section5 :  AbsoluteCoordinate - actual absolute coordinate
//      section6 :  TimeToDisableMotor - delay in us to turns off the engine after inactivity
//      section7 :  ConfigFlags        - configure bits falgs
//                                          1.bit (+1) - 0= normal motor direction, 1= inverse motor direction
//                                          2.-4.bit   - select step table used for stepter motor 0=8 steps, 1=8 inverse steps, 2=4 steps, 3=4 inverse steps
//                                              432 1      432 1     432 1     432 1  bit.
//                                             =000 0     =001 0    =010 0    =011 0
//                                             (+0)       (+2)      (+4)      (+5)
//                                             A  B       A  B      A  B      A  B
//                                             11 10      00 01     11 10     00 01
//                                             11 00      00 11     11 01     00 10
//                                             11 01      00 10     10 11     01 00
//                                             10 01      01 10     01 11     10 00
//                                             10 11      01 00
//                                             00 11      11 00
//                                             01 11      10 00
//                                             01 10      10 01
//                                             
//                                          5.bit (+16) - 0= disable switch detect route begin, 1= enable switch detect route begin
//                                          6.bit (+32) - 0= disable switch detect route end, 1= enable switch detect route end
//                                          7.bit (+64) - 0= disable detect route begin by motor current, 1= enable detect route begin by motor current
//                                          8.bit (+128)- 0= disable detect route end by motor current, 1= enable detect route end by motor current
//                                          9.bit (+256)- 0= disable software control begin of route, 1=enable software control begin of route with value in min_alarm_value from section AbsoluteCoordinate
//                                          10.bit (+512)- 0= disable software control end of route, 1=enable software control end of route with value in max_alarm_value from section AbsoluteCoordinate
//                                          11.bit (+1024)- 0= do nothing, 1=start proces to autodetec boundary (autodec route begin and end, begin will be 0 in absolute coordinate)
//                                          12.bit (+2048)- 0=do nothing, 1=start proces to autodetec speed (optimal will be stred into section2 and maximal will be stored into section9)
//                                          13.bit (+4096)- 0=do nothing, 1=start proces to autodetec acceleration (optimal will be stred into section3 and section4)
//                                          etc. inverse motor direction and enable detec begin and end by motor current = 1+8+16
//      section8  :  ForceHalt          - 0=do nothing, 1=halt moving
//      section9  :  Current            - actual current
//      section10 :  MaxSteps           - max. number of step in one direction 0=no limit
//      section11 :  MaxAmp             - max. ampher enabled to motor consume, more stop motor
//      section12 :  MaxSpeed           - max. speed of motor detected by AutoCalibrateSpeedAndAccel (only for read)
//
//      section13 :  led idetification  - identify of 1-Wire device 1=on 0-off
//      section14 :  Save               - 0=do nothing, 1=save actual section values


// configure constatnats
#define REVERSE_DIRECTION         1
#define STEPS_TABLE               14
#define ENABLE_PIN_BEGENIN        16
#define ENABLE_PIN_END            32
#define ENABLE_AMP_BEGENIN        64
#define ENABLE_AMP_END            128
#define ENABLE_SW_BEGENIN         256
#define ENABLE_SW_END             512
#define START_AUTODETECT_BOUNDARY 1024
#define START_AUTODETECT_SPEED    2048
#define START_AUTODETECT_ACCEL    4096

// defaul config and motor values - can set this values by oneself
#define DEFAULT_DIRECTION 0
#define DEFAULT_STEP_TABLE 0
#define DEFAULT_MAX_STEPS 40000
#define DEFAULT_SPEED 1000
#define DEFAULT_ACCEL 4000
#define DEFAULT_DECEL 4000
#define DEFAULT_MAX_AMP 20
//#define DEFAULT_CONFIG DEFAULT_DIRECTION | DEFAULT_STEP_TABLE*2 | ENABLE_SW_BEGENIN | ENABLE_AMP_END // use definitions from previous section - //configure constants 
#define DEFAULT_CONFIG DEFAULT_DIRECTION | DEFAULT_STEP_TABLE*2 // use definitions from previous section - //configure constants 

// vales for never ending values (for relative moving)
#define NEVER_ENDING_LEFT_MOVING -2147483646
#define NEVER_ENDING_RIGHT_MOVING 2147483646

// define section number
//typedef enum { MOVE_RELATIVE, MOVE_ABSOLUTE, SPEED_, ACCEL_, DECEL_, ABSOLUTE_COORDINATE, TIME_TO_DISABLE_MOTOR, CONFIG_FLAGS,  FORCE_HALT, MAX_SPEED, MAX_STEPS, MAX_AMP, CURRENT, INFO_LED, SAVE }; // enumerate of intem in section array for clear human read code
#define MOVE_RELATIVE 0
#define MOVE_ABSOLUTE 1
#define SPEED_ 2
#define ACCEL_ 3
#define DECEL_ 4
#define ABSOLUTE_COORDINATE 5
#define TIME_TO_DISABLE_MOTOR 6
#define CONFIG_FLAGS 7
#define FORCE_HALT 8
#define MAX_SPEED 9
#define MAX_STEPS 10
#define MAX_AMP 11
#define CURRENT 12
#define INFO_LED 13
#define SAVE 14

// steps move to one key push on this example
#define STEPS 500

byte addr[10];
byte alarm_addr[10];


OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
OneWireMaster0xCD du(&ds);

int32_t m_absolute_coordination; // for remember absolute coordination

/*
 * Find device address
 * if find then set address into global variable "addr" and register "addr" for OneWireMaster0xCD librrary
 * If you do not know device address, you can find it
 */
void find_device() {
  uint8_t i;
  for (;;){ // find right w1 device
    for (;;){
      Serial.println("Search device.");
      if ( ds.search(addr)) break;
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
    }
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println();
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      continue;
    }
    else {
      Serial.println("OK");
      return;
    }
  }
  
}

bool is_device_alarm() { // find alarm or conditional devicesa and compare his addr with my device addr
  uint8_t i;
  ds.reset_search();
  for (;;){ // find right w1 device
    for (;;){
      Serial.println("Search device.");
      if ( ds.search(alarm_addr, false)) break;
      // no find
      return false;
    }
    //Serial.print("ROM =");
    //for( i = 0; i < 8; i++) {
    //  Serial.write(' ');
    //  Serial.print(addr[i], HEX);
    //}
    //Serial.println();
    if (OneWire::crc8(addr, 7) != addr[7]) {
    //  Serial.println("CRC is not valid!");
        continue;
    }
    else {
      //Serial.println("OK");
      if ( memcmp(addr,alarm_addr,9)==0 ) {
        Serial.println("Alarm");
        return true;
      }
      else return false;
    }
  }
}



void setup(void) {
  Serial.begin(9600);
  // set and register addr of device
  find_device();
  du.setAddr(addr);
  Serial.println("Device was be selected for communication.");
  //du.set_actual_value(value,section_num);
  du.set_actual_value(DEFAULT_MAX_STEPS,MAX_STEPS); // set max steps in one direction
  du.set_actual_value(DEFAULT_SPEED,SPEED_); // set speed
  du.set_actual_value(DEFAULT_ACCEL,ACCEL_); // set acceleration
  du.set_actual_value(DEFAULT_DECEL,DECEL_); // set deceleration
  du.set_actual_value(DEFAULT_MAX_AMP,MAX_AMP); // set max curent consume (it is relaive no ampers, if you plan use to mus be set by own tests)
  du.set_actual_value(DEFAULT_CONFIG,CONFIG_FLAGS); // set deceleration
  du.set_status_byte(du.get_status_byte(MOVE_RELATIVE)|0b11000000, MOVE_RELATIVE); // enable alarm on section MOVE_RELATIVE (alarm on end of moving)
  du.set_status_byte(du.get_status_byte(MAX_AMP)&0b00111111, MAX_AMP); // disable max amp alarm
  du.set_status_byte(du.get_status_byte(ABSOLUTE_COORDINATE)&0b00111111, ABSOLUTE_COORDINATE); // disable absolute coordinate  alarm
  
}

 
void loop(void) {
        Serial.print(F("Relative left move steps:"));
        Serial.println(-1*STEPS);
        du.set_actual_value(-1*STEPS,MOVE_RELATIVE);
        while(is_device_alarm()==false){};
        Serial.print(F("Relative right move steps:"));
        Serial.println(STEPS);
        du.set_actual_value(STEPS,MOVE_RELATIVE);
        while(is_device_alarm()==false){};
}
