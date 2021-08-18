/*
   OneWireSlave0xCD - A skeleton 1-Wire multisensor emulator for AVR Microcontroller

   Copyright (C) 2020 Ondrej Lycka info (at) seahu.cz

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

   1-Wire interface for control 2xRGB LED STRIP + 1xWITESTRIP + 1xMOTION SENSOR (using MCU AVR ATmega8 or AVR ATmega328)
   ---------------------------------------------------------------------------------------------------------------------
   This device enable control 2xRGB leds strip + 1xPWM for white led strip + 1x serve motion sensor (this device I use in my work for entry system).
   Features:
    - enable fluet change color led strip or PWM value white led strip 
    - enable flash color led strip or PWM value white led strip
    - can set number of reperation of fleuents chabges or flesh
    - movie sensor can light on white led strip
    - set time of  hold on light after movie sensor end activity
    - inclue indication led
    - enable save actual value as default falues
    
   Because MCU not have enough hardware PWM output, the device use software PWM.
   Because software PWM is CPU intensive, than serve 1-Wire communication may make led strip slightly flashing.
   Device contain next sections:
      section0 :  for frist  RGB led strip - actually color (as RGB number in binary 32bit format in range 0-16777215 (value=Blue+256*Green+65536*Red) 
      section1 :  for frist  RGB led strip - duration in [ms] of change, if >0 duration is time of fluent change form actual to new RGB value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
      section2 :  for frist  RGB led strip - number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
      section3 :  for second RGB led strip - actually color (as RGB number in binary 32bit format in range 0-16777215 (value=Blue+256*Green+65536*Red) 
      section4 :  for second RGB led strip - duration in [ms] of change, if >0 duration is time of fluent change form actual to new RGB value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
      section5 :  for second RGB led strip - number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
      section6 :  for white  PWM led strip - actually PWM value (as number in binary 32bit format in range 0-255) 
      section7 :  for white  PWM led strip - duration in [ms] of change, if >0 duration is time of fluent change form actual to new PWM value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
      section8 :  for white  PWM led strip - number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
      section9 :  for motion sensor         - sensor status (only for read) 1=motion sensor detect activity, 0=no detec activity
      section10:  for motion sensor         - 0=no action, 1=if motion sensor detec activity set PWM white led strip to max value (ligh ON)
      section11:  for motion sensor         - Time in ms in range 0-3600. The time to hold on light after end activity.
      section12:  for led indicator         - Led indication of device led 1=on 0=off
      section13:  for all                   - 1=save actual values as default values (into EEPROM.) (not for MCU ATmega8-no have enough ROM memory)
    
   OneWire family code: 0xCD (more about devices with this family code on https://github.com/seahu/seahu_CD   )

   *  ---------------------  ELETRONIC SCHEME ---------------------------
   *                        
   *                           ATmega8 or ATmega168 or ATmega328
   *                                 +-------\/-------+
   *                   (RESET)      -|1  PC6    PC5 28|- A5   (SCL) ------- RED2  
   *                   (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA) ------- SENSOR  
   *                   (TXD)     D1 -|3  PD1    PC3 26|- A3 --------------- BLUE1     
   *                   (INT0)    D2 -|4  PD2    PC2 25|- A2 --------------- GREEN1
   * !prefered for OW! (INT1) +#D3  -|5  PD3    PC1 24|- A1 --------------- RED1           \\
   *                            D4  -|6  PD4    PC0 23|- A0 --------------+ WHITE    +-----|<|------------o + 12V  (and RED1, REEN1, BLUE1, RED2, GREEN2, BLUE2 have some circuit)
   *                                -|7  VCC    GND 22|-                  |          |   White - LED strip
   *                                -|8  GND   AREF 21|-                  +--------|<   (transitor N-kanal FU120N)
   *                   (XTAL1) *D20 -|9  PB6   AVCC 20|-                            _|_
   *                   (XTAL2) *D21 -|10 PB7    PB5 19|- D13  (SCK)
   *                   (T1)   +#D5  -|11 PD5    PB4 18|- D12  (MISO) ------ GREEN2
   *                   (AIN0) +#D6  -|12 PD6    PB3 17|- #D11 (MOSI,OC2) -- BLUE2
   *                   (AIN1)   D7  -|13 PD7    PB2 16|- #D10 (OC1B)
   *                   (IPC1)   D8  -|14 PB0    PB1 15|- #D9  (OC1A)
   *                                 +----------------+
   *                                 

   Compile for MCU ATMEL AVR ATmega8 & 168 use Arduino NG or later
   Compile for MCU ATMEL AVR ATmega328 use Arduino nano
*/


//  generate Device ID based on compile date and time

// flag MCU with small ROM memory (for MCU with small memory as 8KB must be reduced functionality)
#ifdef __AVR_ATmega8__
  #define SMALL_MEMORY
#endif

 
//#define __AVR_ATtiny85__
#include "OneWireSlave0xCD.cpp.h"
 
#define MAIN_DELAY 2 

//#define PIN_W 13
#define PIN_W A0
#define PIN_R1 A1
#define PIN_G1 A2
#define PIN_B1 A3
#define PIN_SENSOR A4
#define PIN_R2 A5
#define PIN_G2 12
#define PIN_B2 11
#define PIN_LED 13
int timer=0;

// actual values
byte w=0;
byte r1=0;
byte g1=0;
byte b1=0;
byte r2=0;
byte g2=0;
byte b2=0;

// start values
byte start_w=0;
byte start_r1=0;
byte start_g1=0;
byte start_b1=0;
byte start_r2=0;
byte start_g2=0;
byte start_b2=0;

// end values
byte end_w=0;
byte end_r1=0;
byte end_g1=0;
byte end_b1=0;
byte end_r2=0;
byte end_g2=0;
byte end_b2=0;

//duartion (stored in section table)
//repeat   (stored in section table)

//times
unsigned long last_time_w=0;
unsigned long last_time_rgb1=0;
unsigned long last_time_rgb2=0;
unsigned long time_stop_senzor=0;

// for detec new value from 1-wire must store all value
unsigned long last_value_w=0;
unsigned long last_value_rgb1=0;
unsigned long last_value_rgb2=0;
bool last_sensor=0;

#ifdef SMALL_MEMORY
  #define CONT_OF_SECTIONS 13 // number of section -> AT mega8 have small memory (I must disable SAVE section to save memory)
#endif
#ifndef SMALL_MEMORY
  #define CONT_OF_SECTIONS 14 // number of section (sensors or actors)
#endif

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU 1-Wire to 2xRGB + 1xPWM + 1xMotion sensor C2021 by Ing. Ondrej Lycka";
// RGB1
const char rgb1[] PROGMEM               = "RGB {0-16777215} RGB1 value=Blue+256*Green+65536*Red";
const char duration1[] PROGMEM          = "Time [x.001 s] The time of change RGB1 <0=flah >0=fluent         ";
const char repeat1[] PROGMEM            = "Counter {-1 - 2147483647} Repeat RGB1. O=no repeat -1=to infinity";
// RGB2
const char rgb2[] PROGMEM               = "RGB {0-16777215} RGB2 value=Blue+256*Green+65536*Red";
const char duration2[] PROGMEM          = "Time [x.001 s] The time of change RGB1 <0=flah >0=fluent         ";
const char repeat2[] PROGMEM            = "Counter {-1 - 2147483647} Repeat RGB2. O=no repeat -1=to infinity";
// PWM white
const char white[] PROGMEM              = "PWM {0-255} For white led strip.";
const char duration_white[] PROGMEM     = "Time [x.001 s] The time of change RGB1 <0=flah >0=fluent         ";
const char repeat_white[] PROGMEM       = "Counter {-1 - 2147483647}White repeat O=no repeat -1=to infinity";
// Motion sensor
const char sensor[] PROGMEM             = "Botton 1=motion sensor activity 0=no activity                    ";
const char sensor_light[] PROGMEM       = "Switch button 1=sensor activity=>light up 0=do nothing";
const char sensor_hold[] PROGMEM        = "Time [x1 s,] {0-3600}The time to hold on light after end activity";
// standart system switch
const char led[] PROGMEM                = "Led indication led 1=on 0=off"; // indication led
#ifndef SMALL_MEMORY
  const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment
#endif
  
//declaration measured functions 
/* this example have not measured functions */


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
  // RGB1
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , rgb1            , NULL                   , false        , false },
  { C_READ + C_WRITE           + C_32BIT , {.u32=0}      , duration1       , NULL                   , false        , false },
  {          C_WRITE           + C_32BIT , {.u32=0}      , repeat1         , NULL                   , false        , false },
  // RGB2
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , rgb2            , NULL                   , false        , false },
  { C_READ + C_WRITE           + C_32BIT , {.u32=0}      , duration2       , NULL                   , false        , false },
  {          C_WRITE           + C_32BIT , {.u32=0}      , repeat2         , NULL                   , false        , false },
  // PWM white
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , white           , NULL                   , false        , false },
  { C_READ + C_WRITE           + C_32BIT , {.u32=0}      , duration_white  , NULL                   , false        , false },
  {          C_WRITE           + C_32BIT , {.u32=0}      , repeat_white    , NULL                   , false        , false },
  // Motion sensor
  { C_READ + C_WRITE + C_ALARM + C_U32BIT, {.u32=0}      , sensor         , NULL                   , false        , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , sensor_light    , NULL                   , false        , false},
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , sensor_hold     , NULL                   , false        , false },
  // system
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , led             , NULL                   , false        , false},
  #ifndef SMALL_MEMORY
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL                   , false        , false}, //( disable this because ATmega8 has not enought memory )
  #endif
};
typedef enum { RGB1, DURATION_RGB1, REPEAT_RGB1, RGB2, DURATION_RGB2, REPEAT_RGB2, WHITE, DURATION_W, REPEAT_W, SENSOR,SENSOR_LIGHT, SENSOR_HOLD, LED,SAVE }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------

/* this example have not measured functions */

//------------------------ END MEASURE FUNCTIONS --------------------------

//------------------------ 1-WIRE SYSTEM FUNCTIONS --------------------------------------------
#ifndef SMALL_MEMORY
void check_save_values(){
  union value0xCD newValue;
  newValue.b=0; // prepare new value = zero
  
  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    ow.write_new_value(SAVE, newValue); // section Save off
    ow.save_values();
  }
}
#endif

void check_indication_led(){
  if (sections[LED].actualValue.b==1) digitalWrite(PIN_LED, HIGH);
  else digitalWrite(PIN_LED, LOW);
}

//------------------------ END 1-WIRE SYSTEM FUNCTIONS --------------------------------------------

//------------------------ PWM FUNCTIONS ---------------------------------------------------

/*
 * Function calculate new PWM walue of one of r,g,b or w component.
 * Take into account time of duration (time of smooth change between actual value and new value or flashs time) and repeat option.
 */
byte caculate_new_value( unsigned long *p_now, byte *p_start_value, byte *p_end_value, long *p_duartion, long *p_repeat, unsigned long *p_last_time, bool reset_time){
  byte actual_value;
  if ((int)*p_duartion == 0 ) return *p_end_value; // if no set durating immediately retun end value
  // flash
  if ((int)*p_duartion < 0) {
    if ( *p_now > (*p_last_time)-(*p_duartion) ) {
      if (*p_repeat == 0) return *p_end_value;
      actual_value=*p_end_value; // chnage actual value to end value
      *p_end_value=*p_start_value; // chagne start and end values
      *p_start_value=actual_value;
      if ( reset_time == true ) {
        if ( *p_repeat > 0 ) (*p_repeat)--;
        *p_last_time=*p_now;
      }
    }
    else {
      actual_value=*p_start_value;
    }
    return actual_value;
  }
  // fluent change
  if ( *p_now >= (*p_last_time)+(*p_duartion) ) {
    actual_value=*p_end_value;
    if ( *p_repeat!=0 ){
      *p_end_value=*p_start_value;
      *p_start_value=actual_value;
      if ( reset_time == true ) {
        if ( *p_repeat > 0 ) {
          (*p_repeat)--; // attention *p_repeat-- frist decrease pointer address and then return value fronnew address. Therefore (*p_repeat)--
        }
        *p_last_time=*p_now;
      }
    }
  }
  else {
    actual_value= (*p_start_value) + (((long)( ((long)(*p_end_value)-(long)(*p_start_value))*(*p_now-*p_last_time))) / ((long)(*p_duartion))); // !!! consmue more time do flashing (no siutable for software PWM)
  }
  return actual_value;
}

/*
 * Function calculate actual values for all r,g,b,w comoponents at actual time
 */
void caculate_new_values(){
  unsigned long now;
  now=millis();
  // caculate_new_value (   &now, &start_value, &end_value, &delay, &repeat, &last_time, reset_last_time)
  w = caculate_new_value (  &now, &start_w,  &end_w,  &sections[DURATION_W].actualValue.i32,    &sections[REPEAT_W].actualValue.i32   , &last_time_w   , true  );
  r1= caculate_new_value (  &now, &start_r1, &end_r1, &sections[DURATION_RGB1].actualValue.i32, &sections[REPEAT_RGB1].actualValue.i32, &last_time_rgb1, false );
  g1= caculate_new_value (  &now, &start_g1, &end_g1, &sections[DURATION_RGB1].actualValue.i32, &sections[REPEAT_RGB1].actualValue.i32, &last_time_rgb1, false );
  b1= caculate_new_value (  &now, &start_b1, &end_b1, &sections[DURATION_RGB1].actualValue.i32, &sections[REPEAT_RGB1].actualValue.i32, &last_time_rgb1, true  );
  r2= caculate_new_value (  &now, &start_r2, &end_r2, &sections[DURATION_RGB2].actualValue.i32, &sections[REPEAT_RGB2].actualValue.i32, &last_time_rgb2, false );
  g2= caculate_new_value (  &now, &start_g2, &end_g2, &sections[DURATION_RGB2].actualValue.i32, &sections[REPEAT_RGB2].actualValue.i32, &last_time_rgb2, false );
  b2= caculate_new_value (  &now, &start_b2, &end_b2, &sections[DURATION_RGB2].actualValue.i32, &sections[REPEAT_RGB2].actualValue.i32, &last_time_rgb2, true  );
}


/*
 * Split 32bit RGB value into separate 8bit R, G, B values
 */
void split_rgb(uint32_t Rgb, byte *r, byte *g, byte *b){
  //Rgb=sections[RGB1].actualValue.u32;
  *b=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  *g=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  *r=(uint8_t)(Rgb & 0xFF); // first 8 bits
  //start_time_rgb1=millis();
}

/*
 * Compose 32bit RGB value from 8bit R, G, B values
 */
unsigned long rgb(byte r, byte g, byte b){
  return r<<16 + g<<8 + b;
}

/*
 * Detac new values from 1Wire bus
 */
void detect_new_vales_from_1wire(){
  // RGB1
  if (sections[RGB1].actualValue.u32!= last_value_rgb1 ) { // if change
    start_r1=r1; start_g1=g1; start_b1=b1; // start values = actual values
    split_rgb(sections[RGB1].actualValue.u32, &end_r1, &end_g1, &end_b1); // end values = new values
    last_value_rgb1=sections[RGB1].actualValue.u32;
    //if (end_r1==255) digitalWrite(PIN_R2, HIGH);
    last_time_rgb1=millis();
    
  }
  // RGB2
  if (sections[RGB2].actualValue.u32!= last_value_rgb2 ) {
    start_r2=r2; start_g2=g2; start_b2=b2; // start values = actual values
    split_rgb(sections[RGB2].actualValue.u32, &end_r2, &end_g2, &end_b2); // end values = new values
    last_value_rgb2=sections[RGB2].actualValue.u32;
    last_time_rgb2=millis();
  }
  // WHITE
  if (sections[WHITE].actualValue.u32!= last_value_w) {
    start_w=w;                                // start value = actual value
    end_w  =(byte)sections[WHITE].actualValue.u32; // end value = new value
    last_value_w=sections[WHITE].actualValue.u32;
    last_time_w=millis();
  }
}


/*
 * Check movie sensor
 */
void check_sensor(){
  if (sections[SENSOR_LIGHT].actualValue.b==1){
    if (digitalRead(PIN_SENSOR)==HIGH) { // sensor motion activity
      sections[WHITE].actualValue.u32=255; // set full white light
      last_sensor=1;
    }
    else {
      if (last_sensor==1) {
        time_stop_senzor=millis();
        last_sensor=0;
      }
      if ( millis() > time_stop_senzor + sections[SENSOR_HOLD].actualValue.u32 ) sections[WHITE].actualValue.u32=0; // set off white light
    }
  }
}

/*
 * Function generate one PWM step
 */
void pwm_step(byte w, byte r1, byte g1, byte b1, byte r2, byte g2, byte b2){
  if (timer==255) {
    timer=0;
    return;
  }
  if (timer==0) {
    // aplicate new value only in start of new timer cycle (prevention of possible flash effect)
    detect_new_vales_from_1wire();
    // some check normaly inserted into mailn loop (here is placed for reduce CPU overload)
    caculate_new_values();
    check_sensor();
    check_indication_led();
    #ifndef SMALL_MEMORY
      check_save_values(); // at ATmega8 must disable to save memory
    #endif
    
    // set output to HIGH on start of new cycle except zero value  
    if (w==0) digitalWrite(PIN_W, LOW);
    else digitalWrite(PIN_W, HIGH);
    if (r1==0) digitalWrite(PIN_R1, LOW);
    else digitalWrite(PIN_R1, HIGH);
    if (g1==0) digitalWrite(PIN_G1, LOW);
    else digitalWrite(PIN_G1, HIGH);
    if (b1==0) digitalWrite(PIN_B1, LOW);
    else digitalWrite(PIN_B1, HIGH);
    if (r2==0) digitalWrite(PIN_R2, LOW);
    else digitalWrite(PIN_R2, HIGH);
    if (g2==0) digitalWrite(PIN_G2, LOW);
    else digitalWrite(PIN_G2, HIGH);
    if (b2==0) digitalWrite(PIN_B2, LOW);
    else digitalWrite(PIN_B2, HIGH);
  }
  else {
    // set LOW value if raise to his boundary
    if (w==timer) digitalWrite(PIN_W, LOW);
    if (r1==timer) digitalWrite(PIN_R1, LOW);
    if (g1==timer) digitalWrite(PIN_G1, LOW);
    if (b1==timer) digitalWrite(PIN_B1, LOW);
    if (r2==timer) digitalWrite(PIN_R2, LOW);
    if (g2==timer) digitalWrite(PIN_G2, LOW);
    if (b2==timer) digitalWrite(PIN_B2, LOW);
  }
  timer++;
}

//------------------------ END PWM FUNCTIONS ------------------------------------------------


//------------------------SETUP -----------------------------------------------------
void setup() { // the setup function runs once when you press reset or power the board
  //Serial.begin(9600); // open the serial port at 9600 bps:
  pinMode(PIN_W, OUTPUT);
  pinMode(PIN_R1, OUTPUT);
  pinMode(PIN_G1, OUTPUT);
  pinMode(PIN_B1, OUTPUT);
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_R2, OUTPUT);
  pinMode(PIN_G2, OUTPUT);
  pinMode(PIN_B2, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
}


//----------------------- MAIN LOOP ------------------------------------------------
void loop() { // the loop function runs over and over again forever
   pwm_step(w,r1,g1,b1,r2,g2,b2);
  delayMicroseconds(0);
}
