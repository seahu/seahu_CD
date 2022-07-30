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
          - control by relative or absolute coordination,
          - enable save energy by turns off the engine after inactivity
          - enable software or hardware force halt motor
          - enable change motor rotate direction
          - option 3 types of step tables (one phase, two phase and two oro one pahse)
          - indication led (share with one phase of motor)
          - enable save config

   Device contain next sections:
      section0 :  MoveRelative       - movving relative distance in steps
                                        -2147483646 steps => never ending move to left
                                        +2147483646 steps => never ending move to right
      section1 :  MoveAbsolute       - movving absolute distance in steps
      section2 :  Speed              - moving speed in steps/sec
      section3 :  AbsoluteCoordinate - actual absolute coordinate
      section4 :  TimeToDisableMotor - delay in us to turns off the engine after inactivity
      section5 :  ConfigFlags        - configure bits falgs
                                          1.bit (+1) - 0= normal motor direction, 1= inverse motor direction
                                          2.-4.bit   - select step table used for stepter motor:
                                            (+0) = one phase at a time, siples less torque less electric consume (32 steps per ratio and after gearbox 32x64=2048 steps per ratio)
                                            (+2) = two hases at a time, more torque butmore eletric consume (32 steps per ratio and after gearbox 32x64=2048 steps per ratio)
                                            (+4) = two or one phase, two times more  precise (smaller step) (64 steps per ratio and after gearbox 64x64=4096 steps per ratio)
                                              4321       4321      4321 bit.
                                             =000X      =001X     =010X
                                             (+0)       (+2)      (+4)
                                             A  B       A  B      A  B
                                             00 01      00 11     00 01
                                             00 10      01 10     00 11
                                             01 00      11 00     00 10
                                             10 00      10 01     01 10
                                                                  01 00
                                                                  11 00
                                                                  10 00
                                                                  10 01                                
      section6  :  ForceHalt          - 0=do nothing, 1=halt moving


      section7  :  led idetification  - identify of 1-Wire device 1=on 0-off (on my board indication led share same pin as motr pin B1, therefore must be off to enable corect funtion of motor)
      section8  :  Save               - 0=do nothing, 1=save actual section values



   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )
   Motor driver inspiration on: https://github.com/rob-smallshire/stepper-motor-controller/blob/master/%20stepper-motor-controller%20--username%20plastiv%40gmail.com/code/speed_cntr.c



  SCHEME:
  -------
    arduino-AVR_ATtiny85/84:
   ------------------------
   arduino not have native support for AVR_ATtiny85. This supoort have to ADCd :
   the procedure applies for arduino IDE 1.6.4 and hight
   In menu File/Preferences. In the dialg you fill item ADCditional Board Manager with: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json a stiskněte OK. Toto jádro je sice uváděny jako první, ale nedoporučuji ho používat. Bylo první z historických důvodů, ale jeho autor nemá na jeho vývoj tolik času, jako je jádro, zmíněné v odstavci o alternativních jádrech.
   And then use function Tools/Board/Boards Manager. In the list find attiny and install it.
   Beafore complile select:
    Board: ATtiny25/45/85
    Procesor: ATtiny85
    Clock: Internal16MHz
    Select Programmer: (I use secundary arduino as ISP "Arduino as ISP"
    Burn BootloADCer (this is necessary for set correct MCU clock)
    Burn by Sketch/UploADC Using Programmer
   
   More usefull information about ATtiny and arduino:
    https://www.arduinoslovakia.eu/page/attiny85

             / \ +5V                                          
              |                                               
             +-+                                              
             | | 470R                                         
             +-+                                              
              |                                               
          <- +-+                                         
          <- \ / led                                     
             ---                                         
              |                                          
              |                     AVR_ATtiny85          / \ +5V
              |                    +------\/-----+         | 
              |     (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC--+ 
         +----+--------- A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire
         |  +---- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----------+
         |  |            GND D5 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)--+   |
         |  |                      +-------------+                                     |   |                                                                                         +-+
         |  |                                                                          |   |
         |  |                                                                          |   |
         |  |                                                                          |   |            STEP MOTOR  28BYJ-48
         |  |                                                                          |   |                 5VDC
         |  +--------------------------------------------------------------------------------Pink---- A0 ---+
         |                                                                             |   |                (
         |                                                                             |   |                (
         |                                                                             |   |          A     +----+
         |                                                                             |   |                (    |
         |                                                                             |   |                (    |    / \ +5V
         |                                                                             +------Orange- A1 ---+    |     |
         |                                                                                 |  Red                +-----+ 
         |                                                                                 +--Yelow-- B0 ---+    |
         |                                                                                                  (    |
         |                                                                                                  (    |
         |                                                                                            B     +----+ 
         |                                                                                                  (
         |                                                                                                  (
         +------------------------------------------------------------------------------------Blue--- B1 ---+
                                                                                                          

5V red
PB0 orange
PB1 yellow
PB4 pink
PB3 blue                                                                                                       

*/



// 1-Wire SH0xCD
#define CONT_OF_SECTIONS 9 // number of section (sensors or actors)

// step motor pins
#define A0_ 0
#define A1_ 1
#define B0_ 4
#define B1_ 3

// other PINS
#define PIN_LED B1_ // in my board indication led share same pin as motr pin B1 (must be off to enable corect funtion of motor)

// enumerate bits in configure byte
#define DIRECTION 1


//default value
#define DEFAULT_SPEED 1000
#define DEFAULT_TTIME_TO_DISABLE_MOTOR  5000000
#define DEFAULT_CONFIG_FLAGS 0

//procesies
#define PROCES_NOTHING 0
#define PROCES_MOVE 1

// vales for never ending values (for relative moving)
#define NEVER_ENDING_LEFT_MOVING -2147483646
#define NEVER_ENDING_RIGHT_MOVING 2147483646


#include "OneWireSlave0xCD.cpp.h"
OneWireSlave0xCD ow; //define One Wire object

// global varibales for stepter driver
const  byte steps[] PROGMEM = {
  //const byte steps[]= {
  0b00000001, // step_type 0 (one phase at a time)
  0b00000010,
  0b00000100,
  0b00001000,
  0b00000001,
  0b00000010,
  0b00000100,
  0b00001000,
  0b00000011, // step_type 2 (two phase at a time)
  0b00000110,
  0b00001100,
  0b00001001,
  0b00000011,
  0b00000110,
  0b00001100,
  0b00001001,
  0b00000001, // step_type 3 (one or two phase at time)
  0b00000011,
  0b00000010,
  0b00000110,
  0b00000100,
  0b00001100,
  0b00001000,
  0b00001001,
};

bool led_status = 0; // for detect change led status by one wire (led pin is shared with motor pin and effect pin only if chane his value by one wire master)
bool direction = true; // false=inverse direction
unsigned long time_next_step = 0;
unsigned long time_last_step = 0;
byte cur = 0; // cursor for steps table
long absolute_step = 0; // actual absolute coordination in steps
long absolute_step_shadow = 0; // for detec new value form 1-Wire
long cur_step = 0; // actal step in move
bool ever_end_moving = false; // flag for never ending moving
unsigned int accel_steps; // number of accel steps
long stop_step = 0; // step count where end moving
long step_delay = 0; // dealy to next step
byte step_type = 0; // type of step table
// 1-Wire variable
byte configure = 0; // configure byte
unsigned long time_to_disable_motor = 5000000; // time [us] when automaticaly disbale motor after stop moving (save energy) 0=never disable motor
unsigned int speed = DEFAULT_SPEED;
bool halt = false;

// procesies
byte proces = 0; // indication of proces etc. get_boundary, auto_speed_calibration, .
byte proces_state = 0; // proces state (state inside proces etc.get_boundary, auto_speed_calibration, ..)
byte next_proces = 0; // indication of return proces etc. get_boundary, auto_speed_calibration, .
byte next_proces_state = 0; // return proces state (state inside proces etc.get_boundary, auto_speed_calibration, ..)

// factory description is good to store in PROGMEM to save RAM
// "1234567890123456789012345678901234567890123456789012345678901234
const char device_description[] PROGMEM  = "SEAHU SH025b 1-Wire unipolar stepter motor driver               ";
const char MoveRelative[] PROGMEM        = "Distance [steps] relative distance in steps                     ";
const char MoveAbsolute[] PROGMEM        = "Distance [steps] absolute distance in steps                     ";
const char Speed[] PROGMEM               = "Speed [steps/s]{0-65535}                                        ";
const char AbsoluteCoordinate[] PROGMEM  = "Value [steps] actual absolute coordinate                        ";
const char TimeToDisableMotor[] PROGMEM  = "Delay [s*.000001] delay to turns off the engine after inactivity";
const char ConfigFlags[] PROGMEM         = "Value set direction, step table (more in web)                   ";
const char ForceHalt[] PROGMEM           = "Switch {0,1} 0=nothing 1=stop moving                            ";

const char led[] PROGMEM                = "SWITCH identification 1=on 0-off";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections                 "; // mainly for devolepment

//declaration motor function
void setupMove(long step_distance, unsigned int speed);

//declaration measured functions (this exxample do not use measured functions)
void mMoveAbsolute(bool force_now);
void mAbsoluteCoordinate(bool force_now);


/*
   define sections array (this array must be definet as global varialbe (because must be aviable throucht interruot).
          For easy set control register is prepared next constants:
          C_READ      - actual value can be read
          C_WRITE     - actual value is writable for output devices
          C_ALARM     - section have alarm function
          C_MIN_ALARM - on min. value alarm  (can be change by One Wire)
          C_MAX_ALARM - on max. value alarm  (can be change by One Wire)
          C_MEMORY    - section not have value (value is always 0x00), instead it of value is stored into 32B RAM like user description of section
          C_BOOL      - section contain bool value
          C_U32BIT    - section contain unsigned 32 bit binary value
          C_32BIT     - section contain signed 32 bit binary value

          (more about control byte (register) refer file OneWireSlave0xCD.h in OneWireSlave0xCD librarry)
*/
section sections[CONT_OF_SECTIONS] = {
  // { contril byte os section           , actual value  , descrition      , user description buffer, measure function, lock }
  // servo1
  { C_READ + C_WRITE + C_ALARM + C_32BIT , {.i32 = 0}                             , MoveRelative              , NULL                   , NULL               , false }, // after read new value immediately set to zero = ready for new value
  { C_READ + C_WRITE + C_ALARM + C_32BIT , {.i32 = 0}                             , MoveAbsolute              , NULL                   , NULL               , false }, // after read new value immediately set to -2147483648 = redy for new value
  { C_READ + C_WRITE           + C_U32BIT, {.u32 = DEFAULT_SPEED}                 , Speed                     , NULL                   , NULL               , false }, // direct use
  { C_READ + C_WRITE           + C_32BIT , {.i32 = 0}                             , AbsoluteCoordinate        , NULL                   , mAbsoluteCoordinate, false }, // for serve read, write volatile values use shadow buffer
  { C_READ + C_WRITE           + C_U32BIT, {.u32 = DEFAULT_TTIME_TO_DISABLE_MOTOR}, TimeToDisableMotor        , NULL                   , NULL               , false }, //direct use
  { C_READ + C_WRITE           + C_U32BIT, {.u32 = DEFAULT_CONFIG_FLAGS}          , ConfigFlags               , NULL                   , NULL               , false }, // partitualy direct
  { C_READ + C_WRITE           + C_U32BIT, {.u32 = 0}                             , ForceHalt                 , NULL                   , NULL               , false }, // direct use, for read use measure function with digitalread halt pin
  // indication led
  { C_READ + C_WRITE           + C_BOOL,   {.b = 0}                               , led                       , NULL                   , NULL               , false },
  // save actual value to eeprom
  { C_READ + C_WRITE +         + C_BOOL  , {.u32 = 0}                             , Save                      , NULL                   , NULL               , false}, // after read new value immediately set to zero
};
typedef enum { MOVE_RELATIVE, MOVE_ABSOLUTE, SPEED_, ABSOLUTE_COORDINATE, TIME_TO_DISABLE_MOTOR, CONFIG_FLAGS, FORCE_HALT, LED, SAVE }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------

void mAbsoluteCoordinate(bool force_now) { // do is fast do not difference between fast and slow call of this function
  union value0xCD newValue;
  newValue.i32 = absolute_step;
  absolute_step_shadow = absolute_step; // actual value writte into shadow value and 1=Wrire value
  ow.write_new_value(ABSOLUTE_COORDINATE, newValue);
}

//------------------------ END MEASURE FUNCTIONS --------------------------

//------------------------ AUXLIARY SH0xCD FUNCTION ------------------------------
/**
   This is modified oreginal function write_new_value form OneWireSlave0xCD.cpp.h , but withou any affect to alarm (only write new vale do section)
   Write new value into global array of sections.
   Wait to end lock, to prevent fail on One wire comunication (may be lock during reading or writing actual valu by One Wire).
   And clear mearumnet flag.

   @param section Is number section to which one will be write new value
   @param mewValue Is union type varibales whre is stored new value
   @return void.
*/
void write_new_value_withou_alarm_check(uint8_t section, union value0xCD mewValue) {
  while ( (p_sections_ + section)->lock && _OW_lock )  {}; // wait to unlock (section is lock only if is lock both section lock and  global _OW_lock)
  uint8_t control = (p_sections_ + section)->control;
  switch ( control & 0b00011000 ) { // filtr from contol byte of section pnly value types bit 3-5
    case C_BOOL:   // 0b00001000
      (p_sections_ + section)->actualValue.u32 = mewValue.u32; // save new value
      break;
    case C_U32BIT:  // 0b00010000
      (p_sections_ + section)->actualValue.u32 = mewValue.u32;
      break;
    case C_32BIT:   // 0b00011000
      (p_sections_ + section)->actualValue.i32 = mewValue.i32;
      break;
  }
  (p_sections_ + section)->control |= C_READ; // clear flag for new measurment
}

//------------------------ END AUXLIARY SH0xCD FUNCTION ------------------------------

//------------------------ CHECK NEW VALUES FUNCTIONS --------------------------------------------

// check led status
void check_led(){
  if (sections[LED].actualValue.b==1 ) {
    if (led_status!=1) {
      led_status=1;
      digitalWrite(PIN_LED, HIGH); // if setion led is on 
    }
  }
  else {
    if (led_status!=0) {
      led_status=0;
      digitalWrite(PIN_LED, LOW);
    }
  }
}

void check_save_values() {
  union value0xCD newValue;
  newValue.b = 0; // prepare new value = zero

  if (sections[SAVE].actualValue.b == 1) { // if setion save is on
    ow.write_new_value(SAVE, newValue); // section Save off
    ow.save_values();
  }
}

void check_new_value_for_move_relative() {
  union value0xCD newValue;
  newValue.i32 = 0; // prepare new value = zero

  if (sections[MOVE_RELATIVE].actualValue.i32 != 0) {
    // move relative
    //Serial.print("relative move:");
    //Serial.println(sections[MOVE_RELATIVE].actualValue.i32);
    setupMove(sections[MOVE_RELATIVE].actualValue.i32, sections[SPEED_].actualValue.u32);
    //setupMove(500, 400, 400, 1000);
    if (sections[MOVE_RELATIVE].actualValue.i32 == NEVER_ENDING_LEFT_MOVING || sections[MOVE_RELATIVE].actualValue.i32 == NEVER_ENDING_RIGHT_MOVING ) ever_end_moving = true;
    else ever_end_moving = false;
    write_new_value_withou_alarm_check(MOVE_RELATIVE, newValue); // clear relative move value;
    proces = PROCES_MOVE;
  }
}

void check_new_value_for_move_absolute() {
  union value0xCD newValue;
  newValue.i32 = -2147483648; // prepare new value to flag clear new value (this value is reseved for means is not new value)
  if (sections[MOVE_ABSOLUTE].actualValue.i32 != newValue.i32) {
    // move relative
    setupMove(sections[MOVE_ABSOLUTE].actualValue.i32 - absolute_step, sections[SPEED_].actualValue.u32);
    write_new_value_withou_alarm_check(MOVE_ABSOLUTE, newValue); // note to clear absolute move value;
    proces = PROCES_MOVE;
  }
}

void check_new_value_for_config_flags(){
  union value0xCD newValue;
  step_type=(sections[CONFIG_FLAGS].actualValue.u32 >> 1) & 0x0003;
  configure=sections[CONFIG_FLAGS].actualValue.u32;

}

void check_new_value_for_AbsoluteCoordinate() {
  if (sections[ABSOLUTE_COORDINATE].actualValue.i32 != absolute_step_shadow) { // change value from 1-Wire
    absolute_step = sections[ABSOLUTE_COORDINATE].actualValue.i32; // set actual absolute coordination by 1-Wire value
    absolute_step_shadow = sections[ABSOLUTE_COORDINATE].actualValue.i32; // aactual shadow register
    //Serial.print("Change valie of absolute coordination:");Serial.println(absolute_step_shadow);
  }
}

//------------------------ END CHECK NEW VALUES FUNCTIONS --------------------------------------------


//------------------------ STEP MOTOR FUNCTIONS --------------------------------------------

/*
    enabel/disable motor
*/
void enable_motor(bool enable) {
  if (enable == false) {
    digitalWrite(A0_, LOW);
    digitalWrite(A1_, LOW);
    digitalWrite(B0_, LOW);
    if (led_status!=1) digitalWrite(B1_, LOW); // do not off signalization led if is set
  }
}

/*
    return actual direction affected by reverse direction configuration
*/
bool actual_dirrection() {
  if ( (configure & DIRECTION) == 0 ) return direction;
  else return !direction;
}

/*
    do one motor step
*/
void step() {
  byte step;
  bool dir;
  if (actual_dirrection() == true) {
    cur = (cur + 1) & 0x07;
    absolute_step++;
  }
  else {
    cur = (cur - 1) & 0x07;
    absolute_step--;
  }
  //step=steps[cur+step_type*8];
  //step = pgm_read_word_near(steps + cur);
  step = pgm_read_word_near(steps + cur+step_type*8);
  //Serial.print("cur:");
  //Serial.println(cur);
  //Serial.print("step:");
  //Serial.println(step, BIN);
  //Serial.print("step_type:");
  //Serial.println(step_type);
  if ( (0x01 & step) == 0 ) digitalWrite(A0_, LOW); else digitalWrite(A0_, HIGH);
  if ( (0x02 & step) == 0 ) digitalWrite(A1_, LOW); else digitalWrite(A1_, HIGH);
  if ( (0x04 & step) == 0 ) digitalWrite(B0_, LOW); else digitalWrite(B0_, HIGH);
  if ( (0x08 & step) == 0 ) digitalWrite(B1_, LOW); else digitalWrite(B1_, HIGH);
  time_last_step = micros();
}


/*
   prepare relative move
   setup a move, units are in steps, no motion occurs until processMove() is called
   Note: this can only be called when the motor is stopped
*/
void setupMove(long step_distance, unsigned int speed)
{
  if (step_distance == 0) return;

  cur_step = 0;
  enable_motor(true);
  // direction
  if (step_distance < 0) {
    stop_step = -1 * step_distance;
    direction = false;
  }
  else {
    stop_step = step_distance;
    direction = true;
  }
  // delay for constand speed
  step_delay = 1000000 / speed; // [us]
}

/*
   if it is time, move one step
   Exit:  true returned if movement complete, false returned not a final target
*/
bool processMovement(void)
{
  if (micros() < time_next_step) return false;
  // check if already at the target position
  if (cur_step == stop_step) return true; // end moving
  // do step
  step();
  if (ever_end_moving == false) cur_step++;
  time_next_step = time_last_step + step_delay;
  return false;
}

/*
   do process Movement of one step with all control (begin, end position, stop switch and max current)
*/
bool processControlMovement(void) {
  union value0xCD EEvalue;
  
  if (sections[FORCE_HALT].actualValue.b == true ) {
    //Serial.println("halt");
    //halt=true;
    return false;
  }
  if (processMovement() == true) return true;
  return false;
}



//------------------------ END STEP MOTOR FUNCTIONS --------------------------------------------

// setup
void setup() {


  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
  //defult values may by changed by values into eeprom by ow.ini()
  //sections[SPEED_].actualValue.u32=DEFAULT_SPEED;
  //sections[CONFIG_FLAGS].actualValue.u32=DEFAULT_CONFIG_FLAGS;
  //sections[MOVE_RELATIVE].actualValue.i32=0;
  //sections[MOVE_ABSOLUTE].actualValue.i32=-2147483648;
  //sections[CONFIG_FLAGS].actualValue.u32=C_BEGIN_STOP_AMP+C_END_STOP_AMP;

  pinMode(A0_, OUTPUT);
  pinMode(A1_, OUTPUT);
  pinMode(B0_, OUTPUT);
  pinMode(B1_, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  //Serial.begin(115200);
  //Serial.println("Start:");
  //proces=PROCES_GET_BOUNDARY;
  //proces_state=GET_BOUNDARY_START;mSpeed
  //proces=PROCES_NOTHING;
  proces = PROCES_NOTHING;
  next_proces = PROCES_NOTHING;
  //configure=0+C_BEGIN_STOP_AMP+C_END_STOP_AMP;
  //sections[MOVE_RELATIVE].actualValue.i32=5000;

}

void loop() {
  switch (proces) {
    case PROCES_NOTHING:
      if (time_to_disable_motor != 0) {
        if ( (time_last_step + sections[TIME_TO_DISABLE_MOTOR].actualValue.u32) < micros() ) enable_motor(false);
      }
      break;
    case PROCES_MOVE:
      if (processControlMovement() == true) {
        proces = PROCES_NOTHING;
        // pripadne aktualizuj alarm (upozorneni konce pohybu)
        //Serial.print("end PROCES_MOVE:");
        if (sections[MOVE_RELATIVE].control & 0b11000000 ) _OW_alarm = true; //if enabled alarm for relatime moving
        if (sections[MOVE_ABSOLUTE].control & 0b11000000 ) _OW_alarm = true; // if enabled alarm for moving right
      }
      break;
  }
  check_led();
  check_save_values();
  check_new_value_for_move_relative();
  check_new_value_for_move_absolute();
  check_new_value_for_AbsoluteCoordinate();
  check_new_value_for_config_flags();

  //Serial.println(sections[MOVE_RELATIVE].actualValue.i32);
  //delay(100);

}
