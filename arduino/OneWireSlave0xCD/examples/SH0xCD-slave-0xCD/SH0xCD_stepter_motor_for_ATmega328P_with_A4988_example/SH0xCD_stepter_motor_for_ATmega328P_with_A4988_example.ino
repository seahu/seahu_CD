/*
   OneWireSlave0xCD - A skeleton 1-Wire multisensor emulator for AVR Microcontroller

   Copyright (C) 2020 Ondrej Lycka info (at) seahu.cz

   The skeleton requres an incllude file with the device-specific dependent command handling.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   1-Wire to septer motor dirver with H bridge interface (using MCU AVR ATmega328)
   ------------------------------------------------------------------------------------
   This device enable control stepter motor using H bridge circuite L298N or 2x L9110S or similar
   
   features:
          - enable set speed
          - enable set acceleration
          - enable set deceleration
          - control by relative or absolute coordination,
          - enable begin and end detection by ending sactual_curwitchs or current motor consume or by software oundary or combination
          - enable select from fort types steps tables
          - enable save energy by turns off the engine after inactivity
          - enable software or hardware force halt motor
          
   Device contain next sections:
      section0 :  MoveRelative       - movving relative distance in steps
                                        -2147483646 steps => never ending move to left
                                        +2147483646 steps => never ending move to right
      section1 :  MoveAbsolute       - movving absolute distance in steps
      section2 :  Speed              - moving speed in steps/sec
      section3 :  Accel              - moving acceleration in steps/(sec*sec)
      section4 :  Decel              - moving deceleration in steps/(sec*sec)
      section5 :  AbsoluteCoordinate - actual absolute coordinate
      section6 :  TimeToDisableMotor - delay in us to turns off the engine after inactivity
      section7 :  ConfigFlags        - configure bits falgs
                                          1.bit (+1) - 0= normal motor direction, 1= inverse motor direction
                                          2.-4.bit   - select step table used for stepter motor 0=8 steps, 1=8 inverse steps, 2=4 steps, 3=4 inverse steps
                                              432 1      432 1     432 1     432 1  bit.
                                             =000 0     =001 0    =010 0    =011 0
                                             (+0)       (+2)      (+4)      (+5)
                                             A  B       A  B      A  B      A  B
                                             11 10      00 01     11 10     00 01
                                             11 00      00 11     11 01     00 10
                                             11 01      00 10     10 11     01 00
                                             10 01      01 10     01 11     10 00
                                             10 11      01 00
                                             00 11      11 00
                                             01 11      10 00
                                             01 10      10 01
                                             
                                          5.bit (+16) - 0= disable switch detect route begin, 1= enable switch detect route begin
                                          6.bit (+32) - 0= disable switch detect route end, 1= enable switch detect route end
                                          7.bit (+64) - 0= disable detect route begin by motor current, 1= enable detect route begin by motor current
                                          8.bit (+128)- 0= disable detect route end by motor current, 1= enable detect route end by motor current
                                          9.bit (+256)- 0= disable software control begin of route, 1=enable software control begin of route with value in min_alarm_value from section AbsoluteCoordinate
                                          10.bit (+512)- 0= disable software control end of route, 1=enable software control end of route with value in max_alarm_value from section AbsoluteCoordinate
                                          11.bit (+1024)- 0= do nothing, 1=start proces to autodetec boundary (autodec route begin and end, begin will be 0 in absolute coordinate)
                                          12.bit (+2048)- 0=do nothing, 1=start proces to autodetec speed (optimal will be stred into section2 and maximal will be stored into section9)
                                          13.bit (+4096)- 0=do nothing, 1=start proces to autodetec acceleration (optimal will be stred into section3 and section4)
                                          etc. inverse motor direction and enable detec begin and end by motor current = 1+8+16
      section8  :  ForceHalt          - 0=do nothing, 1=halt moving

      section9  :  Current            - actual current
      section10 :  MaxSteps           - max. number of step in one direction 0=no limit
      section11 :  MaxAmp             - max. ampher enabled to motor consume, more stop motor
      section12 :  MaxSpeed           - max. speed of motor detected by AutoCalibrateSpeedAndAccel (only for read)

      section13 :  led idetification  - identify of 1-Wire device 1=on 0-off
      section14 :  Save               - 0=do nothing, 1=save actual section values
      

   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )
   Motor driver inspiration on: https://github.com/rob-smallshire/stepper-motor-controller/blob/master/%20stepper-motor-controller%20--username%20plastiv%40gmail.com/code/speed_cntr.c



  SCHEME:
  -------
  Suported MCU AVR ATmega8,16,328

                                    +-------\/-------+
                      (RESET)      -|1  PC6    PC5 28|- A5   (SCL)   
                      (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)   
  BEGIN_STOP_PIN <--- (TXD)     D1 -|3  PD1    PC3 26|- A3      
    END_STOP_PIN <--- (INT0)    D2 -|4  PD2    PC2 25|- A2
    !prefered for OW! (INT1) +#D3  -|5  PD3    PC1 24|- A1 
                               D4  -|6  PD4    PC0 23|- A0 ------> INFO_LED_PIN
                                   -|7  VCC    GND 22|-  
                                   -|8  GND   AREF 21|-  
                      (XTAL1) *D20 -|9  PB6   AVCC 20|-  
                      (XTAL2) *D21 -|10 PB7    PB5 19|- D13  (SCK)
  DIRECTION_PIN  <--- (T1)   +#D5  -|11 PD5    PB4 18|- D12  (MISO)
       STEP_PIN  <--- (AIN0) +#D6  -|12 PD6    PB3 17|- #D11 (MOSI,OC2)
        MS3_PIN  <--- (AIN1)   D7  -|13 PD7    PB2 16|- #D10 (OC1B) --> ENABLE_MOTOR_PIN
        MS2_PIN  <--- (IPC1)   D8  -|14 PB0    PB1 15|- #D9  (OC1A) --> MS1_PIN
                                    +----------------+
    
    (#  indicates the PWM pins)
    (+# indicates the additional PWM pins on the ATmega168.)
    (*  indicates additional two free pins instead external crystal if is used core: https://github.com/MCUdude/MiniCore this core have more options atc, change clk of MCU)
    (LED** first prototype use arduino nano led on pin 13, pin 8 use next versions)


 
*/

/*
 * !!!! BEFORE COMPILE chek suportedMCU.h !!!!
 * !-----------------------------------------!
 * 1-wire librrary must use another intterupt then servo rutine that use timer1.
 * Please check into inside librrary 'OneWireSlave0xCD' in file suportedMCU.h must be select for corect MCU timer2
 * as show below:
 *   //Timer setting and interrupt
 *  // on select:
 *           //#define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() ) !!!Compile problem inside arduino DO NOT USE (INT is used for time function, allready)
 *           //#define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D6 PWM)
 *           //#define Timer0_OCR0B // select timer0 with comparator B interrupt (colision with D5 PWM)
 *           //#define Timer1_OCR1A // select timer1 with comparator A interrupt (colision with D9 PWM)
 *           //#define Timer1_OCR1B // select timer1 with comparator B interrupt (colision with D10 PWM) note: it is not appropriate to use this 16bit timmer for this purpose
 *           //#define Timer2_OCR2A // select timer2 with comparator A interrupt (colision with D11 PWM) note: it is not appropriate to use this 16bit timmer for this purpose
 *           #define Timer2_OCR2B // select timer2 with comparator B interrupt (colision with D3 PWM) <-- best choice because when is uset D3 with INT1 for One-Wire pin, then it cannot be used for PWM anyway.
 */

// 1-Wire SH0xCD
#define CONT_OF_SECTIONS 11 // number of section (sensors or actors)

// step motor pins (one motor use two H bridge drivers A,B)
#define ENABLE_MOTOR_PIN 10
#define MS1_PIN 9
#define MS2_PIN 8
#define MS3_PIN 7
#define STEP_PIN 6
#define DIRECTION_PIN 5
// begin, end stop pin
#define BEGIN_STOP_PIN 1
#define END_STOP_PIN 2
// led PIN
#define INFO_LED_PIN A0

// enumerate moving state
#define START 0
#define ACCEL 1
#define RUN 2
#define DECEL 3
#define STOP 4

// enumerate bits in configure byte
#define DIRECTION 1
#define C_STEP_TABLE 2 // 4 8
#define C_BEGIN_STOP_PIN 16
#define C_END_STOP_PIN 32
#define C_BEGIN_STOP_SOFTW 256
#define C_END_STOP_SOFTW 512

//default value
#define LIMIT_SPEED 4000
#define MIN_ACCEL 800
#define DEFAULT_MAX_STEPS 40000
#define DEFAULT_SPEED 1000
#define DEFAULT_ACCEL 4000
#define DEFAULT_DECEL 4000
#define DEFAULT_MAX_AMP 20
#define DEFAULT_MAX_SPEED DEFAULT_SPEED
#define DEFAULT_TTIME_TO_DISABLE_MOTOR  5000000
#define DEFAULT_CONFIG_FLAGS C_BEGIN_STOP_PIN+C_END_STOP_PIN

//procesies
#define PROCES_NOTHING 0
#define PROCES_MOVE 1
// proces auto calibrate boundary
#define PROCES_GET_BOUNDARY 2
#define GET_BOUNDARY_STOP 0
#define GET_BOUNDARY_START 1
#define GET_BOUNDARY_LITTLE_MOVE 2
#define GET_BOUNDARY_FIND_BEGIN 3
#define GET_BOUNDARY_FIND_END 4
#define GET_BOUNDARY_GO_BACK 5
// proces auto calibrate speed
#define PROCES_SPEED_CALIBRATION 3
#define SPEED_CALIBRATION_STOP 0
#define SPEED_CALIBRATION_START 1
#define SPEED_CALIBRATION_DO 2
#define SPEED_CALIBRATION_END 3
#define SPEED_CALIBRATION_GO_BACK 4
// proces auto calibrate accel
#define PROCES_ACCEL_CALIBRATION 4
#define ACCEL_CALIBRATION_STOP 0
#define ACCEL_CALIBRATION_START 1
#define ACCEL_CALIBRATION_START_TEST 2
#define ACCEL_CALIBRATION_END_TEST 3
#define ACCEL_CALIBRATION_END 4
#define ACCEL_CALIBRATION_GO_BACK 5

// vales for never ending values (for relative moving)
#define NEVER_ENDING_LEFT_MOVING -2147483646
#define NEVER_ENDING_RIGHT_MOVING 2147483646


#include "OneWireSlave0xCD.cpp.h"
OneWireSlave0xCD ow; //define One Wire object

// global varibales for stepter driver
const  byte steps[] PROGMEM = { 
//const byte steps[]= { 
            0b10001110, // step_type 0
            0b10001100,
            0b10001101,
            0b10001001,
            0b10001011,
            0b10000011,
            0b10000111,
            0b10000110, 
            0b10001110, // step_type 1
            0b10001101,
            0b10001011,
            0b10000111,
            0b10001110,
            0b10001101,
            0b10001011,
            0b10000111,
            0b00000001, // step_type 2
            0b00000011,
            0b00000010,
            0b00000110,
            0b00000100,
            0b00001100,
            0b00001000,
            0b00001001, 
            0b00000001, // step_type 3
            0b00000010,
            0b00000100,
            0b00001000,
            0b00000001,
            0b00000010,
            0b00000100,
            0b00001000             
            };
bool direction=true; // false=inverse direction
unsigned long time_next_step=0;
unsigned long time_last_step=0;
byte state=STOP; // status of moving
byte cur=0; // cursor for steps table
long absolute_step=0; // actual absolute coordination in steps
long absolute_step_shadow=0; // for detec new value form 1-Wire
long cur_step=0; // actal step in move
bool ever_end_moving=false; // flag for never ending moving
unsigned int accel_steps; // number of accel steps
long decel_step=0; // step distance where start decelation
long stop_step=0; // step count where end moving
long step_delay=0; // dealy to next step
long run_delay=0; // delay for required speed
long max_delay=0; // delay for mim speed (during deceleration)
long rest=0; // rest from integer dividion on acceleration/deceleration calculate
byte step_type=2; // type of step table
// 1-Wire variable
byte configure=0; // configure byte
byte max_amp=20; // max electric current, for detec stop moving
unsigned long time_to_disable_motor=5000000; // time [us] when automaticaly disbale motor after stop moving (save energy) 0=never disable motor
unsigned int accel=DEFAULT_ACCEL;
unsigned int decel=DEFAULT_DECEL;
unsigned int speed=DEFAULT_SPEED;
unsigned int max_speed=LIMIT_SPEED; // for 1-Wire onlu for read (can be set by auto_speed_calibration)
byte actual_cur;
long begin_trace;     // absolute coordination for software stop moving behind begin boundary and for test alarm (value from EEPROM copy to ram for faster run)
long end_trace;       // absolute coordination for sowfware end moving after end boundary and for test alarm (value from EEPROM copy to ram for faster run)
long max_steps=DEFAULT_MAX_STEPS; // boundary for max moving in one direction
bool halt=false;

// procesies
byte proces=0; // indication of proces etc. get_boundary, auto_speed_calibration, .
byte proces_state=0; // proces state (state inside proces etc.get_boundary, auto_speed_calibration, ..)
byte next_proces=0; // indication of return proces etc. get_boundary, auto_speed_calibration, .
byte next_proces_state=0; // return proces state (state inside proces etc.get_boundary, auto_speed_calibration, ..)
// proces auto calibrate speed
byte max_cur, min_cur,last_cur;
int  last_speed,actual_speed, optimal_speed_first, optimal_speed_last;
// proces auto calibrate accel
unsigned int first_speed;

// factory description is good to store in PROGMEM to save RAM
                                       // "1234567890123456789012345678901234567890123456789012345678901234
const char device_description[] PROGMEM  = "SEAHU 1-Wire stepter motor driver C2020 by Ing. Ondrej Lycka    ";
const char MoveRelative[] PROGMEM        = "Distance [steps] relative distance in steps                     ";
const char MoveAbsolute[] PROGMEM        = "Distance [steps] absolute distance in steps                     ";
const char Speed[] PROGMEM               = "Speed [steps/s]{0-65535}                                        ";
const char Accel[] PROGMEM               = "Accel [steps/(s*s)]{0-65535} motor accleration                  ";
const char Decel[] PROGMEM               = "Decel [steps/(s*s)]{0-65535} motor deceleration                 ";
const char AbsoluteCoordinate[] PROGMEM  = "Value [steps] actual absolute coordinate                        ";
const char TimeToDisableMotor[] PROGMEM  = "Delay [s*.000001] delay to turns off the engine after inactivity";
const char ConfigFlags[] PROGMEM         = "Value set direction, step table, check boundary (more in web)   ";
const char ForceHalt[] PROGMEM           = "Switch {0,1} 0=nothing 1=stop moving                            ";


const char led[] PROGMEM                = "SWITCH identification 1=on 0-off";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections                 "; // mainly for devolepment

//declaration motor function
void setupMove(long step_distance, unsigned int accel, unsigned int decel, unsigned int speed);
  
//declaration measured functions (this exxample do not use measured functions)
void mMoveAbsolute(bool force_now);
void mSpeed(bool force_now);
void mAccel(bool force_now);
void mDecel(bool force_now);
void mCurrent(bool force_now);
void mForceHalt(bool force_now);
void mAbsoluteCoordinate(bool force_now);


/* 
 * define sections array (this array must be definet as global varialbe (because must be aviable throucht interruot). 
 *        For easy set control register is prepared next constants:
 *        C_READ      - actual value can be read
 *        C_WRITE     - actual value is writable for output devices
 *        C_ALARM     - section have alarm function
 *        C_MIN_ALARM - on min. value alarm  (can be change by One Wire)
 *        C_MAX_ALARM - on max. value alarm  (can be change by One Wire)
 *        C_MEMORY    - section not have value (value is always 0x00), instead it of value is stored into 32B RAM like user description of section
 *        C_BOOL      - section contain bool value
 *        C_U32BIT    - section contain unsigned 32 bit binary value
 *        C_32BIT     - section contain signed 32 bit binary value
*        
 *        (more about control byte (register) refer file OneWireSlave0xCD.h in OneWireSlave0xCD librarry)
 */
section sections[CONT_OF_SECTIONS]={
  // { contril byte os section           , actual value  , descrition      , user description buffer, measure function, lock }
  // servo1
  { C_READ + C_WRITE + C_ALARM + C_32BIT , {.i32=0}                             , MoveRelative              , NULL                   , NULL               , false }, // after read new value immediately set to zero = ready for new value
  { C_READ + C_WRITE + C_ALARM + C_32BIT , {.i32=0}                             , MoveAbsolute              , NULL                   , NULL               , false }, // after read new value immediately set to -2147483648 = redy for new value
  { C_READ + C_WRITE           + C_U32BIT, {.u32=DEFAULT_SPEED}                 , Speed                     , NULL                   , mSpeed             , false }, // direct use
  { C_READ + C_WRITE           + C_U32BIT, {.u32=DEFAULT_ACCEL}                 , Accel                     , NULL                   , mAccel             , false }, // direct use
  { C_READ + C_WRITE           + C_U32BIT, {.u32=DEFAULT_DECEL}                 , Decel                     , NULL                   , mDecel             , false }, // direct use
  { C_READ + C_WRITE           + C_32BIT , {.i32=0}                             , AbsoluteCoordinate        , NULL                   , mAbsoluteCoordinate, false }, // for serve read, write volatile values use shadow buffer
  { C_READ + C_WRITE           + C_U32BIT, {.u32=DEFAULT_TTIME_TO_DISABLE_MOTOR}, TimeToDisableMotor        , NULL                   , NULL               , false }, //direct use
  { C_READ + C_WRITE           + C_U32BIT, {.u32=DEFAULT_CONFIG_FLAGS}          , ConfigFlags               , NULL                   , NULL               , false }, // partitualy direct
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}                             , ForceHalt                 , NULL                   , mForceHalt         , false }, // direct use, for read use measure function with digitalread halt pin
  // identify
  { C_READ + C_WRITE           + C_BOOL  , {.u32=0}                            , led                      , NULL                   , NULL               , false }, // direct use
  // save actual value to eeprom
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}                            , Save                     , NULL                   , NULL               , false}, // after read new value immediately set to zero
};
typedef enum { MOVE_RELATIVE, MOVE_ABSOLUTE, SPEED_, ACCEL_, DECEL_, ABSOLUTE_COORDINATE, TIME_TO_DISABLE_MOTOR, CONFIG_FLAGS,  FORCE_HALT, INFO_LED, SAVE }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------
void mSpeed(bool force_now){ // do is fast do not difference between fast and slow call of this function
  union value0xCD newValue;
  newValue.u32=speed;
  ow.write_new_value(SPEED_, newValue);
}

void mAccel(bool force_now){ // do is fast do not difference between fast and slow call of this function
  union value0xCD newValue;
  newValue.u32=accel;
  ow.write_new_value(ACCEL_, newValue);
}

void mDecel(bool force_now){ // do is fast do not difference between fast and slow call of this function
  union value0xCD newValue;
  newValue.u32=decel;
  ow.write_new_value(DECEL_, newValue);
}

void mForceHalt(bool force_now){ // do is fast do not difference between fast and slow call of this function
  union value0xCD newValue;
}

void mAbsoluteCoordinate(bool force_now){ // do is fast do not difference between fast and slow call of this function
  union value0xCD newValue;
  newValue.i32=absolute_step;
  absolute_step_shadow=absolute_step; // actual value writte into shadow value and 1=Wrire value
  ow.write_new_value(ABSOLUTE_COORDINATE, newValue);
}

//------------------------ END MEASURE FUNCTIONS --------------------------

//------------------------ AUXLIARY SH0xCD FUNCTION ------------------------------
/**
 * This is modified oreginal function write_new_value form OneWireSlave0xCD.cpp.h , but withou any affect to alarm (only write new vale do section)
 * Write new value into global array of sections.
 * Wait to end lock, to prevent fail on One wire comunication (may be lock during reading or writing actual valu by One Wire).
 * And clear mearumnet flag.
 * 
 * @param section Is number section to which one will be write new value
 * @param mewValue Is union type varibales whre is stored new value
 * @return void.
 */ 
void write_new_value_withou_alarm_check(uint8_t section, union value0xCD mewValue){
  while ( (p_sections_+section)->lock && _OW_lock )  {}; // wait to unlock (section is lock only if is lock both section lock and  global _OW_lock)
  uint8_t control=(p_sections_+section)->control;
  switch( control & 0b00011000 ) { // filtr from contol byte of section pnly value types bit 3-5
    case C_BOOL:   // 0b00001000
      (p_sections_+section)->actualValue.u32=mewValue.u32; // save new value
      break;
    case C_U32BIT:  // 0b00010000
      (p_sections_+section)->actualValue.u32=mewValue.u32;
      break;
    case C_32BIT:   // 0b00011000
      (p_sections_+section)->actualValue.i32=mewValue.i32;
      break;
  }
  (p_sections_+section)->control|=C_READ; // clear flag for new measurment
}

//------------------------ END AUXLIARY SH0xCD FUNCTION ------------------------------

//------------------------ CHECK NEW VALUES FUNCTIONS --------------------------------------------

void check_save_values(){
  union value0xCD newValue;
  newValue.b=0; // prepare new value = zero
  
  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    ow.write_new_value(SAVE, newValue); // section Save off
    ow.save_values();
  }
}

void check_new_value_for_move_relative(){
  union value0xCD newValue;
  newValue.i32=0; // prepare new value = zero

  if (sections[MOVE_RELATIVE].actualValue.i32!=0){
    // move relative 
    //Serial.print("relative move:");
    //Serial.println(sections[MOVE_RELATIVE].actualValue.i32);
    setupMove(sections[MOVE_RELATIVE].actualValue.i32, sections[ACCEL_].actualValue.u32, sections[DECEL_].actualValue.u32, sections[SPEED_].actualValue.u32);
    //setupMove(500, 400, 400, 1000);
    if (sections[MOVE_RELATIVE].actualValue.i32==NEVER_ENDING_LEFT_MOVING || sections[MOVE_RELATIVE].actualValue.i32==NEVER_ENDING_RIGHT_MOVING ) ever_end_moving=true;
    else ever_end_moving=false;
    write_new_value_withou_alarm_check(MOVE_RELATIVE, newValue); // clear relative move value;
    proces=PROCES_MOVE;
  }
}

void check_new_value_for_move_absolute(){
  union value0xCD newValue;
  newValue.i32=-2147483648; // prepare new value to flag clear new value (this value is reseved for means is not new value)
  if (sections[MOVE_ABSOLUTE].actualValue.i32 != newValue.i32){
    // move relative 
    setupMove(sections[MOVE_ABSOLUTE].actualValue.i32-absolute_step, sections[ACCEL_].actualValue.u32, sections[DECEL_].actualValue.u32, sections[SPEED_].actualValue.u32);
    write_new_value_withou_alarm_check(MOVE_ABSOLUTE, newValue); // note to clear absolute move value;
    proces=PROCES_MOVE;
  }
}

void check_new_value_for_config_flags(){
  union value0xCD newValue;
  byte new_step_type;
  new_step_type=(sections[CONFIG_FLAGS].actualValue.u32 >> 1) & 0x0003;
  if (step_type!=new_step_type) { // change motor step table
    step_type=new_step_type;
    if ( (new_step_type & 0x01) == 0 )  digitalWrite(MS1_PIN, LOW);
    else                                digitalWrite(MS1_PIN, HIGH);
    new_step_type=new_step_type >> 1;
    if ( (new_step_type & 0x01) == 0 )  digitalWrite(MS2_PIN, LOW);
    else                                digitalWrite(MS2_PIN, HIGH);
    new_step_type=new_step_type >> 1;
    if ( (new_step_type & 0x01) == 0 )  digitalWrite(MS3_PIN, LOW);
    else                                digitalWrite(MS3_PIN, HIGH);
  }
}

void check_new_value_for_AbsoluteCoordinate(){
  if (sections[ABSOLUTE_COORDINATE].actualValue.i32!=absolute_step_shadow){ // change value from 1-Wire
    absolute_step=sections[ABSOLUTE_COORDINATE].actualValue.i32; // set actual absolute coordination by 1-Wire value
    absolute_step_shadow=sections[ABSOLUTE_COORDINATE].actualValue.i32; // aactual shadow register
    //Serial.print("Change valie of absolute coordination:");Serial.println(absolute_step_shadow);
  }
}

void check_new_value_for_info_led(){
  digitalWrite(INFO_LED_PIN,sections[INFO_LED].actualValue.b);
}
//------------------------ END CHECK NEW VALUES FUNCTIONS --------------------------------------------


//------------------------ STEP MOTOR FUNCTIONS --------------------------------------------

/* 
 *  enabel/disable motor
 */
void enable_motor(bool enable){
  if (enable==true) {
    digitalWrite(ENABLE_MOTOR_PIN, LOW);
  }
  else {
    digitalWrite(ENABLE_MOTOR_PIN, HIGH);
  }
}

/* 
 *  return actual direction affected by reverse direction configuration 
 */
bool actual_dirrection(){
  if ( (configure & DIRECTION)==0 ) return direction;
  else return !direction;
}

/* 
 *  do one motor step
 */
void step(){
  byte step;
  bool dir;
  if (direction==true) {
    absolute_step++;
    digitalWrite(DIRECTION_PIN, HIGH);
  }
  else {
    absolute_step--;
    digitalWrite(DIRECTION_PIN, LOW);
  }
  //step=steps[cur+step_type*8];
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(STEP_PIN, HIGH);
  time_last_step=micros();
}

/*
 * prepare relative move
 * setup a move, units are in steps, no motion occurs until processMove() is called
 * Note: this can only be called when the motor is stopped
 */
void setupMove(long step_distance, unsigned int accel, unsigned int decel, unsigned int speed)
{
  
  unsigned int decel_distance;
  unsigned long accel_lim_steps;
  
  
  if (step_distance==0) return;
  state=START;
  cur_step=0;
  rest=0;
  enable_motor(true);
  // direction
  if (step_distance<0){
    stop_step=-1*step_distance;
    direction=false;
  }
  else {
    stop_step=step_distance;
    direction=true;
  }
  // dealy for constand speed
  run_delay=1000000/speed; // [us]
  run_delay=(run_delay*1000)/676; // 1/0.676 correction to polynom compute result
  // accel
  if (accel>0) {
    // initial delay T0=sqrt(2/accel)
    step_delay=sqrt(2*1000000 / accel)*1000; // accel [step/s^2]; cur_delay [us]
    // count of accelerations steps: accel_steps= speed^2/(2*a)
    accel_steps=((long)speed*speed)/(2*accel);
  }
  else {
    step_delay=run_delay; // if don't set acceleration, first step_delay=run_delay
    accel_steps=0;
    accel_lim_steps=0;
    state=RUN;
  }
  // decel
  if (decel>0){
    max_delay=sqrt(2*1000000 / decel)*1000; // cecel [step/s^2]; cur_delay [us] (theretical max_delay will be not used, is here for sure due to possible inaccuracies during calculation of deceleration ) 
    //max_delay=(max_delay*1000)/676; // 1/0.676 correction to polynom compute result 
    // count of decelerations steps: decel_distance=(accel_steps*a)/d  or decel_distance= speed^2/(2*d)
    //decel_distance=((long)accel_steps*accel)/decel; // faster compute method, but depends on accel
    decel_distance=((long)speed*speed)/(2*decel); // secound compute method indepeded on accel
  }
  else { // if don't set deceleration
    decel_distance=0;
    accel_lim_steps=stop_step;
  }
  // theoretical acceleration limit (with no speed limit), if is set accel and decel
  if (accel>0 && decel>0 ){
    // count of steps to theroetical end acceleration and start decelarion (with no speed limit): accel_lim_steps=stop_step*d/(a+d)
    accel_lim_steps=((long)stop_step*decel)/(accel+decel);
  }
  // count of steps from start to starting deceleartion
  if (accel_lim_steps>=accel_steps) decel_step=stop_step-decel_distance; //  in case to acceleration reach to required speed _/--\_
  //else decel_step=stop_step-accel_lim_steps; //  in case to short step distance and acceleration don't reach to required speed _/\_
  else decel_step=accel_lim_steps; //  in case to short step distance and acceleration don't reach to required speed _/\_
  // actualise begin and end boundary value
  EEcmp(begin_trace, EE_START_POINT+ABSOLUTE_COORDINATE*EE_SIZE_OF_SECTION+EE_MIN_ALARM_VALUE,4); // get minValue from EEPROM
  EEcmp(end_trace, EE_START_POINT+ABSOLUTE_COORDINATE*EE_SIZE_OF_SECTION+EE_MAX_ALARM_VALUE,4); // get maxValue from EEPROM
  //Serial.println("Setup move done");
}

/*
 * if it is time, move one step 
 * Exit:  true returned if movement complete, false returned not a final target 
 */
bool processMovement(void)
{ 

  if (micros()<time_next_step) return false;
  // check if already at the target position
  if (cur_step == stop_step) state=STOP;

  switch(state){
    case START:
      step();
      cur_step++;
      time_next_step=time_last_step+(step_delay*676)/1000;
      state=ACCEL;
      break;
    case ACCEL:
      step();
      step_delay=step_delay-(2*step_delay+rest)/(4*cur_step+1); // calcutate new step_delay = T'n'=T'n-1'-(2*T'n-1'+rest)/(4*n +1)
      rest=(2*step_delay+rest)%(4*cur_step+1);
      if (step_delay<=run_delay){
        rest=0;
        state=RUN;
      }
      time_next_step=time_last_step+(step_delay*676)/1000;
      cur_step++;
      if (cur_step == decel_step) {
        state=DECEL;
      }
      break;
    case RUN:
      step();
      if (ever_end_moving==false) cur_step++;
      //cur_step++;
      time_next_step=time_last_step+(run_delay*676)/1000;
      if (cur_step >= decel_step) {
        state=DECEL;
      }
      break;
    case DECEL:
      step();
      step_delay=step_delay-(2*step_delay+rest)/(4*(cur_step-stop_step)+1); // calcutate new step_delay = T'n'=T'n-1'-(2*T'n-1'+rest)/(4*n +1) note: n start from decel_step
      rest=(2*step_delay+rest)%(4*(cur_step-stop_step)+1);
      if (step_delay>max_delay) step_delay=max_delay; // if calculation of decel is accurate, will be never used
      time_next_step=time_last_step+(step_delay*676)/1000;
      cur_step++;
      break;
    case STOP:
      return(true);
  }
  return false;
}


/*
 * do process Movement of one step with all control (begin, end position, stop switch and max current)
 */
bool processControlMovement(void){
  union value0xCD EEvalue;
  int configure=sections[CONFIG_FLAGS].actualValue.u32;
  
  
  //Serial.print("configure"); Serial.println(configure);
  //Serial.print("absolute_step:"); Serial.println(absolute_step);
  
  if (sections[FORCE_HALT].actualValue.b==true ) {
    //Serial.println("halt");
    //halt=true;
    return false;
  }
  if (processMovement()==true) return true;
  if (actual_dirrection()==true){
    if ( absolute_step>=end_trace ) {
      if ( (configure & C_END_STOP_SOFTW)!=0 ) {
        //Serial.print("soft stop: "); Serial.println(configure & C_BEGIN_STOP_SOFTW); state=STOP;
      }
      if ( sections[ABSOLUTE_COORDINATE].control & 0b10000000 ) {
        //Serial.print("Set allarm end boundary:");
        _OW_alarm=true; // if set low value alarm for section ABSOLUTE_COORDINATE
      }
    }
    if ( (configure & C_END_STOP_PIN)!=0) {
      if (digitalRead(END_STOP_PIN)==0) {
        //Serial.println("pin stop");
        state=STOP;
      }
    }
  }
  else {
    if ( absolute_step<=begin_trace ) {
      if ( (configure & C_BEGIN_STOP_SOFTW)!=0 ) {
        //Serial.print("configure: "); Serial.println(configure);
        //Serial.print("C_BEGIN_STOP_SOFTW: "); Serial.println(C_BEGIN_STOP_SOFTW);
        //Serial.print("soft stop: "); Serial.println((configure & 11));
        //Serial.print("soft stop: "); Serial.println((0 & (C_BEGIN_STOP_SOFTW)));  state=STOP;
      }
      if ( sections[ABSOLUTE_COORDINATE].control & 0b01000000 ) {
        //Serial.print("Set allarm begenin boundary:");
        _OW_alarm=true; // if set max value alarm for section ABSOLUTE_COORDINATE
      }
    }
    if ( (configure & C_BEGIN_STOP_PIN)!=0) {
      if (digitalRead(BEGIN_STOP_PIN)==0) {
        //Serial.println("pin stop"); 
        state=STOP;
      }
    }
  }
  return false;
} 

 

//------------------------ END STEP MOTOR FUNCTIONS --------------------------------------------

// setup
void setup() {


  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
  //defult values may by changed by values into eeprom by ow.ini()
  //sections[SPEED_].actualValue.u32=DEFAULT_SPEED;
  //sections[ACCEL_].actualValue.u32=DEFAULT_ACCEL;
  //sections[DECEL_].actualValue.u32=DEFAULT_DECEL;
  //sections[MAX_STEPS].actualValue.u32=DEFAULT_MAX_STEPS;
  //sections[CONFIG_FLAGS].actualValue.u32=DEFAULT_CONFIG_FLAGS;
  //sections[MOVE_RELATIVE].actualValue.i32=0;
  //sections[MOVE_ABSOLUTE].actualValue.i32=-2147483648;
  //sections[CONFIG_FLAGS].actualValue.u32=C_BEGIN_STOP_AMP+C_END_STOP_AMP;
  //sections[MAX_AMP].actualValue.u32=DEFAULT_MAX_AMP;
  
  pinMode(ENABLE_MOTOR_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  pinMode(INFO_LED_PIN, OUTPUT);
  pinMode(BEGIN_STOP_PIN, INPUT_PULLUP);
  pinMode(END_STOP_PIN, INPUT_PULLUP);
  //Serial.begin(115200);
  //Serial.println("Start:");
  //proces=PROCES_GET_BOUNDARY;
  //proces_state=GET_BOUNDARY_START;
  //proces=PROCES_NOTHING;
  proces=PROCES_NOTHING;
  next_proces=PROCES_NOTHING;
  //configure=0+C_BEGIN_STOP_AMP+C_END_STOP_AMP;
  //sections[MOVE_RELATIVE].actualValue.i32=5000;
  
}

void loop() {
    switch (proces){
    case PROCES_NOTHING:
      if (time_to_disable_motor!=0){
        if ( (time_last_step+sections[TIME_TO_DISABLE_MOTOR].actualValue.u32)<micros() ) enable_motor(false);
      }
      break;
    case PROCES_MOVE:
      if (processControlMovement()==true) {
        proces=PROCES_NOTHING;
        // pripadne aktualizuj alarm (upozorneni konce pohybu)
        //Serial.print("end PROCES_MOVE:");
        if (sections[MOVE_RELATIVE].control & 0b11000000 ) _OW_alarm=true;  //if enabled alarm for relatime moving
        if (sections[MOVE_ABSOLUTE].control & 0b11000000 ) _OW_alarm=true; // if enabled alarm for moving right
      }
      break;
  } 
  check_save_values();
  check_new_value_for_move_relative();
  check_new_value_for_move_absolute();
  check_new_value_for_config_flags();
  check_new_value_for_AbsoluteCoordinate();
  check_new_value_for_info_led();
  
  //Serial.println(sections[MOVE_RELATIVE].actualValue.i32);
  //delay(100);

}
