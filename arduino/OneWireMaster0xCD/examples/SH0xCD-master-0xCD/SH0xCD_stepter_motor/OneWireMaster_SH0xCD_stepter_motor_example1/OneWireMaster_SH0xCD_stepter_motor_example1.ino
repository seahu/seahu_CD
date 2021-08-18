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
#define DEFAULT_CONFIG DEFAULT_DIRECTION | DEFAULT_STEP_TABLE*2 | ENABLE_SW_BEGENIN | ENABLE_AMP_END // use definitions from previous section - //configure constants 

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

void print_help(){
  Serial.println(F("HELP"));
  Serial.println(F("----"));
  Serial.println(F("h - this help."));
  Serial.println(F("i - List actual setting."));
  Serial.println(F("b - Start autodetect boundary by motor cuurent consume."));
  Serial.println(F("d - Start autodetect speed."));
  Serial.println(F("a - Start autodetect acceleration."));
  Serial.println(F("o - Set origin for absolute coordination."));
  Serial.println(F("z - Set beginin trace (in absolute coordinaton)."));
  Serial.println(F("x - Set end trace (in absolute coordinaton)."));
  Serial.println(F("c - Switch Enable/Disable sowtware trace boundary."));
  Serial.println(F("e - Switch Disable(halt)/Enable motor."));
  Serial.println(F("f - Store actual absolute coordination."));
  Serial.println(F("g - Go to stored absolute coordination."));
  Serial.println(F("n - Relative left move steps."));
  Serial.println(F("m - Relative right move steps."));
  Serial.println(F("N - Never ending left move."));
  Serial.println(F("M - Relative right move steps."));
  Serial.println(F("l - Switch ON/OFF indication led."));
  Serial.println(F("s - Save actual section values."));
  Serial.println(F("note: More setting in source by edit 'defaul config and motor values' define values."));
  Serial.println(F("--"));
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
  print_help();
}

 
void loop(void) {
  char str,stat;
  int32_t value;
  uint32_t u_value;
  uint32_t n_value;
  val v_value;
  while (Serial.available() > 0) {
    str=Serial.read();
    Serial.println(str);
    switch (str){
      case 'h': // help
        print_help();
        break;
      case 'i': // info list setting and actual values
        du.start_measure_all();
        du.get_actual_value((val*)&value,CONFIG_FLAGS);
        Serial.println(F("List actual setting"));
        Serial.println(F("-------------------"));
        Serial.println(F("Config:"));
        Serial.print(F("Reverse direction:"));
        Serial.println( (value&REVERSE_DIRECTION) ? "Y" : "N");
        Serial.print(F("Steps table:"));
        Serial.println( (value&STEPS_TABLE)>>1);
        Serial.print(F("Enable begenin by PIN:"));
        Serial.println( (value&ENABLE_PIN_BEGENIN) ? "Y" : "N");
        Serial.print(F("Enable end by PIN:"));
        Serial.println( (value&ENABLE_PIN_END) ? "Y" : "N");
        Serial.print(F("Enable begenin by current:"));
        Serial.println( (value&ENABLE_AMP_BEGENIN) ? "Y" : "N");
        Serial.print(F("Enable end by current:"));
        Serial.println( (value&ENABLE_AMP_END) ? "Y" : "N");
        Serial.print(F("Enable begenin by software from min_lalarm_value in section ABSOLUTE_COORDINATE:"));
        Serial.println( (value&ENABLE_SW_BEGENIN) ? "Y" : "N");
        Serial.print(F("Enable end by software from max_lalarm_value in section ABSOLUTE_COORDINATE:"));
        Serial.println( (value&ENABLE_SW_END) ? "Y" : "N");
        du.get_min_alarm_value((val*)&value,ABSOLUTE_COORDINATE);
        Serial.print(F("Min alarm value:"));
        Serial.println(value);
        du.get_max_alarm_value( (val*)&value,ABSOLUTE_COORDINATE);
        Serial.print(F("Min alarm value:"));
        Serial.println(value);
        stat=du.get_status_byte(ABSOLUTE_COORDINATE);
        Serial.print(F("Is set alarm for absolute min_alarm_value:"));
        Serial.println( (stat&0b01000000) ? "Y" : "N");
        Serial.print(F("Is set alarm for absolute max_alarm_value:"));
        Serial.println( (stat&0b10000000) ? "Y" : "N");
        stat=du.get_status_byte(MOVE_RELATIVE);
        Serial.print(F("Is set alarm for end relative moving:"));
        Serial.println( (stat&0b11000000) ? "Y" : "N");
        du.get_actual_value((val*)&value,ABSOLUTE_COORDINATE);
        Serial.print(F("Actual absolute coorinate:"));
        Serial.println(value);
        du.get_actual_value((val*)&u_value,SPEED_);
        Serial.print(F("Speed:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,ACCEL_);
        Serial.print(F("Accel:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,DECEL_);
        Serial.print(F("Decel:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,TIME_TO_DISABLE_MOTOR);
        Serial.print(F("Time to disable motor after n activity:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,MAX_STEPS);
        Serial.print(F("Max steps in one moving:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,MAX_SPEED);
        Serial.print(F("Max speed discoverd by autocalibration speed proces:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,MAX_AMP);
        Serial.print(F("Max current for stop motor:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,CURRENT);
        Serial.print(F("Actual current:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,INFO_LED);
        Serial.print(F("Status of info led:"));
        Serial.println(u_value);
        break;
      case 'b':
        Serial.println(F("Start autodetect boundary by motor cuurent consume"));
        //du.set_actual_value(DEFAULT_CONFIG | ENABLE_AMP_BEGENIN | ENABLE_AMP_END | START_AUTODETECT_BOUNDARY,CONFIG_FLAGS); // start proces to find bounadary by amp
        du.set_actual_value(DEFAULT_CONFIG | START_AUTODETECT_BOUNDARY,CONFIG_FLAGS); // start proces to find bounadary by amp
        break;
      case 'd':
        Serial.println(F("Start autodetect speed."));
        du.set_actual_value(DEFAULT_CONFIG | ENABLE_AMP_BEGENIN | ENABLE_AMP_END | START_AUTODETECT_SPEED,CONFIG_FLAGS); // start proces to autodetect max speed
        break;
      case 'a':
        Serial.println(F("Start autodetect acceleration."));
        du.set_actual_value(DEFAULT_CONFIG | ENABLE_AMP_BEGENIN | ENABLE_AMP_END | START_AUTODETECT_ACCEL,CONFIG_FLAGS); // start proces to autodetect max accleration
        break;
      case 'o':
        Serial.print(F("Set origin for absolute coordination:"));
        du.get_actual_value((val*)&value,ABSOLUTE_COORDINATE); // for coretly write new value must first read actual value (because device use shadow register who actualice by reading, otherway new value will be compared with no actual value)
        du.set_actual_value(0,ABSOLUTE_COORDINATE); // set zero to absolute coordinate
        Serial.println(F("OK"));
        break;
      case 'z':
        Serial.print(F("Set beginin trace (in absolute coordinaton):"));
        du.start_measure(ABSOLUTE_COORDINATE); // force to actualize actual coordination section
        du.get_actual_value((val*)&value,ABSOLUTE_COORDINATE); // get actual absolute coordination value
        du.set_min_alarm_value((val*)&value,ABSOLUTE_COORDINATE); // set begenin of trace
        du.get_min_alarm_value((val*)&value,ABSOLUTE_COORDINATE); // for check read stored begenin trace
        Serial.println(value);
        break;
      case 'x':
        Serial.print(F("Set end trace (in absolute coordinaton):"));
        du.start_measure(ABSOLUTE_COORDINATE); // force to actualize actual coordination section
        du.get_actual_value((val*)&value,ABSOLUTE_COORDINATE); // get actual absolute coordination value
        du.set_max_alarm_value((val*)&value,ABSOLUTE_COORDINATE); // set end of trace
        du.get_max_alarm_value((val*)&value,ABSOLUTE_COORDINATE); // for check read stored end trace
        Serial.println(value);
        break;
      case 'c':
        Serial.print(F("Switch Enable/Disable sowtware trace boundary:"));
        du.get_actual_value((val*)&u_value,CONFIG_FLAGS); // get actual config
        if ( (u_value & ENABLE_SW_BEGENIN) && (u_value & ENABLE_SW_END) ) { // actualy is enable -> disable
          u_value=(u_value & ~ENABLE_SW_BEGENIN ); // prepare new value for disable sowtware check begenin
          du.set_actual_value((val*)&u_value,CONFIG_FLAGS); // set new config
          u_value=(u_value & ~ENABLE_SW_END); // prepare new value for disable sowtware check end
          du.set_actual_value((val*)&u_value,CONFIG_FLAGS); // set new config
          Serial.print(F("Disable"));
        }
        else { // actualy is disable -> enable
          u_value=(u_value | ENABLE_SW_BEGENIN ); // prepare new value for enable sowtware check begenin
          du.set_actual_value((val*)&n_value,CONFIG_FLAGS); // set new config
          u_value=(u_value | ENABLE_SW_END); // prepare new value for enable sowtware check end
          du.set_actual_value((val*)&u_value,CONFIG_FLAGS); // set new config
          Serial.print(F("Enable"));
        }
        break;
      case 'e':
        Serial.print(F("Switch Disable(halt)/Enable motor:"));
        du.get_actual_value((val*)&u_value,FORCE_HALT); // get actual config
        if (u_value==1 ) { // actualy is disable -> 0=enable
          u_value=0;
          Serial.print(F("Enable"));
        }
        else {  // actualy is enable -> 1=disable
          u_value=1;
          Serial.print(F("Disable"));
        }
        du.set_actual_value((val*)&u_value,CONFIG_FLAGS); // send new value
        break;
      case 'f':
        Serial.print(F("Store actual absolute coordination:"));
        du.start_measure(ABSOLUTE_COORDINATE);
        du.get_actual_value((val*)&m_absolute_coordination,ABSOLUTE_COORDINATE); // get actual config
        Serial.println(m_absolute_coordination);
        break;
      case 'g':
        Serial.print(F("Go to stored absolute coordination:"));
        Serial.println(m_absolute_coordination);
        du.set_actual_value(m_absolute_coordination,MOVE_ABSOLUTE);
        break;
      case 'n':
        Serial.print(F("Relative left move steps:"));
        Serial.println(-1*STEPS);
        du.set_actual_value(-1*STEPS,MOVE_RELATIVE);
        break;
      case 'm':
        Serial.print(F("Relative right move steps:"));
        Serial.println(STEPS);
        du.set_actual_value(STEPS,MOVE_RELATIVE);
        break;
      case 'N':
        Serial.println(F("Never ending left move:"));
        du.set_actual_value(NEVER_ENDING_LEFT_MOVING,MOVE_RELATIVE);
        break;
      case 'M':
        Serial.println(F("Never ending right move:"));
        du.set_actual_value(NEVER_ENDING_RIGHT_MOVING,MOVE_RELATIVE);
        break;
      case 's':
        Serial.println(F("Save actual section values:"));
        du.set_actual_value(1,SAVE);
        break;
      case 'l':
        Serial.print(F("Switch ON/OFF indication led:"));
        du.get_actual_value((val*)&u_value,INFO_LED);
        if (u_value==true) {
          Serial.println("switch OFF");
          du.set_actual_value(false,INFO_LED);
        }
        else {
          Serial.println("switch ON");
          du.set_actual_value(true,INFO_LED);
        }
        break;
    }
  }
}
