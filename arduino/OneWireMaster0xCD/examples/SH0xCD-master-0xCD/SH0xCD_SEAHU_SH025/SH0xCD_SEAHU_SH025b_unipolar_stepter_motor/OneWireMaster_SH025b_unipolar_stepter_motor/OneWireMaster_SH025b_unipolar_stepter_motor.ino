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
// Device description: SEAHU SH025b 1-Wire unipolar stepter motor driver C2021
// Number sections: 9
//      section0 :  MoveRelative       - movving relative distance in steps
//                                        -2147483646 steps => never ending move to left
//                                        +2147483646 steps => never ending move to right
//      section1 :  MoveAbsolute       - movving absolute distance in steps
//      section2 :  Speed              - moving speed in steps/sec
//      section3 :  AbsoluteCoordinate - actual absolute coordinate
//      section4 :  TimeToDisableMotor - delay in us to turns off the engine after inactivity
//      section5 :  ConfigFlags        - configure bits falgs
//                                          1.bit (+1) - 0= normal motor direction, 1= inverse motor direction
//                                          2.-4.bit   - select step table used for stepter motor:
//                                            (+0) = one phase at a time, siples less torque less electric consume (32 steps per ratio and after gearbox 32x64=2048 steps per ratio)
//                                            (+2) = two hases at a time, more torque butmore eletric consume (32 steps per ratio and after gearbox 32x64=2048 steps per ratio)
//                                            (+4) = two or one phase, two times more  precise (smaller step) (64 steps per ratio and after gearbox 64x64=4096 steps per ratio)
//                                              4321       4321      4321 bit.
//                                             =000X      =001X     =010X
//                                             (+0)       (+2)      (+4)
//                                             A  B       A  B      A  B
//                                             00 01      00 11     00 01
//                                             00 10      01 10     00 11
//                                             01 00      11 00     00 10
//                                             10 00      10 01     01 10
//                                                                  01 00
//                                                                  11 00
//                                                                  10 00
//                                                                  10 01                                
//      section6  :  ForceHalt          - 0=do nothing, 1=halt moving
//
//
//      section7  :  led idetification  - identify of 1-Wire device 1=on 0-off (on my board indication led share same pin as motr pin B1, therefore must be off to enable corect funtion of motor)
//      section8  :  Save               - 0=do nothing, 1=save actual section values
//

// configure constatnats
#define REVERSE_DIRECTION         1
#define STEPS_TABLE               14

// defaul config and motor values - can set this values by oneself
#define DEFAULT_DIRECTION 0
#define DEFAULT_SPEED 50
#define DEFAULT_CONFIG DEFAULT_DIRECTION  // use definitions from previous section - //configure constants 

// vales for never ending values (for relative moving)
#define NEVER_ENDING_LEFT_MOVING -2147483646
#define NEVER_ENDING_RIGHT_MOVING 2147483646

// define section number
//typedef enum { MOVE_RELATIVE, MOVE_ABSOLUTE, SPEED_, ACCEL_, DECEL_, ABSOLUTE_COORDINATE, TIME_TO_DISABLE_MOTOR, CONFIG_FLAGS,  FORCE_HALT, MAX_SPEED, MAX_STEPS, MAX_AMP, CURRENT, INFO_LED, SAVE }; // enumerate of intem in section array for clear human read code
#define MOVE_RELATIVE 0
#define MOVE_ABSOLUTE 1
#define SPEED_ 2
#define ABSOLUTE_COORDINATE 3
#define TIME_TO_DISABLE_MOTOR 4
#define CONFIG_FLAGS 5
#define FORCE_HALT 6
#define INFO_LED 7
#define SAVE 8

// steps move to one key push on this example
#define STEPS 500

byte addr[10];


OneWire  ds(22);  // on pin 10 (a 4.7K resistor is necessary)
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
  Serial.println(F("o - Set origin for absolute coordination."));
  Serial.println(F("c - Switch Enable/Disable sowtware trace boundary."));
  Serial.println(F("e - Switch Disable(halt)/Enable motor."));
  Serial.println(F("f - Store actual absolute coordination."));
  Serial.println(F("g - Go to stored absolute coordination."));
  Serial.println(F("n - Relative left move steps."));
  Serial.println(F("m - Relative right move steps."));
  Serial.println(F("N - Never ending left move."));
  Serial.println(F("M - Never ending right move."));
  Serial.println(F("z - Decrease speed -100."));
  Serial.println(F("x - Decrease speed -10."));
  Serial.println(F("v - Incerase speed +10."));
  Serial.println(F("b - Incerase speed +100."));
  Serial.println(F("p - Change step table."));
  Serial.println(F("t - Switch ON/OFF hlat motor status."));
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
  //du.set_actual_value(DEFAULT_SPEED,SPEED_); // set speed
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
        Serial.println( (value&STEPS_TABLE)>>1 );
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
        du.get_actual_value((val*)&u_value,TIME_TO_DISABLE_MOTOR);
        Serial.print(F("Time to disable motor after n activity:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,INFO_LED);
        Serial.print(F("Status of info led:"));
        Serial.println(u_value);
        du.get_actual_value((val*)&u_value,FORCE_HALT);
        Serial.print(F("Status of halt motor status:"));
        Serial.println(u_value);
        break;
      case 'o':
        Serial.print(F("Set origin for absolute coordination:"));
        du.get_actual_value((val*)&value,ABSOLUTE_COORDINATE); // for coretly write new value must first read actual value (because device use shadow register who actualice by reading, otherway new value will be compared with no actual value)
        du.set_actual_value(0,ABSOLUTE_COORDINATE); // set zero to absolute coordinate
        Serial.println(F("OK"));
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
      case 't':
        Serial.println(F("Switch ON/OFF hlat status motor:"));
        du.get_actual_value((val*)&u_value,FORCE_HALT);
        if (u_value==true) {
          Serial.println("switch OFF");
          du.set_actual_value(false,FORCE_HALT);
        }
        else {
          Serial.println("switch ON");
          du.set_actual_value(true,FORCE_HALT);
        }
        break;
      case 'r':
        Serial.println(F("Reverse motor direction ON/OFF:"));
        du.get_actual_value((val*)&u_value,CONFIG_FLAGS);
        if ( (u_value & 0x01) == 1) {
          Serial.println("Reverse motor direction OFF");
          du.set_actual_value((u_value & ~0x01),CONFIG_FLAGS);
        }
        else {
          Serial.println("Reverse motor direction ON");
          du.set_actual_value((u_value | 0x01),CONFIG_FLAGS);
        }
        break;
      case 'p':
        Serial.print(F("Change step table to:"));
        du.get_actual_value((val*)&u_value,CONFIG_FLAGS);
        n_value=(u_value >> 1) & 0x07;
        n_value++;
        if (n_value >= 3 ) n_value=0; // this device has only 3 types of step tables (0,1,2)
        Serial.println(n_value);
        u_value = u_value & ~0x0E;
        u_value = u_value | (n_value << 1);
        du.set_actual_value((u_value),CONFIG_FLAGS);
        break;
      case 'z':
        Serial.print(F("SPEED: speed -100="));
        du.get_actual_value((val*)&value,SPEED_);
        value=value-100;
        if ( value < 0 ) value=0;
        Serial.println(value);
        du.set_actual_value(value, SPEED_);
        break;
      case 'x':
        Serial.print(F("SPEED: speed -10="));
        du.get_actual_value((val*)&value,SPEED_);
        value=value-10;
        if ( value < 0 ) value=0;
        Serial.println(value);
        du.set_actual_value(value, SPEED_);
        break;
      case 'v':
        Serial.print(F("SPEED: speed +10="));
        du.get_actual_value((val*)&value,SPEED_);
        value=value+10;
        Serial.println(value);
        du.set_actual_value(value, SPEED_);
        break;
      case 'b':
        Serial.print(F("SPEED: speed +100="));
        du.get_actual_value((val*)&value,SPEED_);
        value=value+100;
        Serial.println(value);
        du.set_actual_value(value, SPEED_);
        break;
    }
  }
}
