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

   1-Wire interface for control RGBW LED (using MCU AVR ATtiny85)
   ---------------------------------------------------------------
   This device enable control RGBW led or RGBW leds strip. Red, green and blue leds are directly diren by hardware timers, but white led is driver by software PWM.
   Sowtware PWM has less precision especially during 1-Wire communication.
   Features:
    - enable fluet change color led strip
    - enable flash color led strip
    - can set number of reperation of fleuents chabges or flesh
    - enable save actual value as default falues
   
   Device contain next sections:
      section0 :  actually color (as RGBW number in binary format)
      section1 :  duration in [ms] of change, if >0 duration is time of fluent change form actual to new RGB value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
      section2 :  number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 

      section3 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)
    
   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )

   arduino-AVR_ATtiny85/84:
   ------------------------
   arduino not have native support for AVR_ATtiny85. This supoort have to add :
   the procedure applies for arduino IDE 1.6.4 and hight
   In menu File/Preferences. In the dialg you fill item Additional Board Manager with: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json a stiskněte OK. Toto jádro je sice uváděny jako první, ale nedoporučuji ho používat. Bylo první z historických důvodů, ale jeho autor nemá na jeho vývoj tolik času, jako je jádro, zmíněné v odstavci o alternativních jádrech.
   And then use function Tools/Board/Boards Manager. In the list find attiny and install it.
   Beafore complile select:
    Board: ATtiny25/45/85
    Procesor: ATtiny85
    Clock: Internal16MHz
    Select Programmer: (I use secundary arduino as ISP "Arduino as ISP"
    Burn Bootloader (this is necessary for set correct MCU clock)
    Burn by Sketch/Upload Using Programmer
   
   More usefull information about ATtiny and arduino:
    https://www.arduinoslovakia.eu/page/attiny85


                                     AVR_ATtiny85   
                                   +------\/-----+ 
                    (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC
                         A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire
   Blue led  <--- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----> Red led
                            GND 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)----> Green led
                                   +-------------+               
*/

//  generate Device ID based on compile date and time
//#define __AVR_ATtiny85__
#include "OneWireSlave0xCD.cpp.h"

#define CONT_OF_SECTIONS 4 // number of section (sensors or actors)
#define PIN_R 1
#define PIN_G 0
#define PIN_B 4
#define PIN_W 3 //(not use in this example)

byte timer=0; // couter for software timer

// actual values
byte W=0;
byte R=0;
byte G=0;
byte B=0;

// start values
byte start_r=0;
byte start_g=0;
byte start_b=0;
byte start_w=0;

// end values
byte end_r=0;
byte end_g=0;
byte end_b=0;
byte end_w=0;

//times
unsigned long last_time_rgbw=0;

// for detec new value from 1-wire must store all value
unsigned long last_value_rgbw=0;

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU 1-Wire to RGB C2020 by Ing. Ondrej Lycka";
const char rgbw[] PROGMEM               = "RGBW {0-4294967295} value=Blue+256*Green+65536*Red+16777216*white";
const char duration[] PROGMEM           = "Time [x.001 s] The time of change RGB1 <0=flah >0=fluent         ";
const char repeat[] PROGMEM             = "Counter {-1 - 2147483647} Repeat RGB1. O=no repeat -1=to infinity";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment
  
//declaration measured functions 
/* this example have not measured functions */

void pokus(){
  return;
}

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
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , rgbw            , NULL                   , false           , false },
  { C_READ + C_WRITE           + C_32BIT , {.u32=0}      , duration        , NULL                   , false           , false },
  {          C_WRITE           + C_32BIT , {.u32=0}      , repeat          , NULL                   , false           , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL                   , false           , false },
};
typedef enum { RGBW, DURATION_RGBW, REPEAT_RGBW, SAVE }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------
/* this example have not measured functions */


//------------------------ END MEASURE FUNCTIONS --------------------------

//------------------------ OTHER FUNCTIONS --------------------------------------------

void check_save_values(){
  union value0xCD newValue;
  newValue.b=0; // prepare new value = zero
  
  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    ow.write_new_value(SAVE, newValue); // section Save off
    ow.save_values();
  }
}
//------------------------ END OTHER FUNCTIONS --------------------------------------------


//------------------------ PWM FUNCTIONS --------------------------------------------

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
  W = caculate_new_value (  &now, &start_w, &end_w, &sections[DURATION_RGBW].actualValue.i32, &sections[REPEAT_RGBW].actualValue.i32, &last_time_rgbw   , false  );
  R = caculate_new_value (  &now, &start_r, &end_r, &sections[DURATION_RGBW].actualValue.i32, &sections[REPEAT_RGBW].actualValue.i32, &last_time_rgbw, false );
  G = caculate_new_value (  &now, &start_g, &end_g, &sections[DURATION_RGBW].actualValue.i32, &sections[REPEAT_RGBW].actualValue.i32, &last_time_rgbw, false );
  B = caculate_new_value (  &now, &start_b, &end_b, &sections[DURATION_RGBW].actualValue.i32, &sections[REPEAT_RGBW].actualValue.i32, &last_time_rgbw, true  );
}


/*
 * Split 32bit RGB value into separate 8bit R, G, B values
 */
void split_rgb(uint32_t Rgb, byte *r, byte *g, byte *b, byte *w){
  //Rgb=sections[RGB1].actualValue.u32;
  *b=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  *g=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  *r=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate
  *w=(uint8_t)(Rgb & 0xFF); // first 8 bits
}

/*
 * Compose 32bit RGBW value from 8bit R, G, B values
 */
unsigned long rgb(byte r, byte g, byte b, byte w){
  return w<<24 + r<<16 + g<<8 + b;
}


/*
 * detect new RGBW values
 */
void detect_new_vales_from_1wire(){
  if (sections[RGBW].actualValue.u32!= last_value_rgbw ) { // if change
    start_r=R; start_g=G; start_b=B, start_w=W; // start values = actual values
    split_rgb(sections[RGBW].actualValue.u32, &end_r, &end_g, &end_b, &end_w); // end values = new values
    last_value_rgbw=sections[RGBW].actualValue.u32;
    last_time_rgbw=millis();
  }
}


/*
 * Function generate one step of software PWM
 */
void pwm_step(){
  if (timer==255) {
    timer=0;
    return;
  }
  if (timer==0) {
    // aplicate new value only in start of new timer cycle (prevention of possible flash effect)
    detect_new_vales_from_1wire();
    caculate_new_values();
    //write new values into hardware PWM (using global variables R,G,B)
    analogWrite(PIN_R, R);
    analogWrite(PIN_G, G);
    analogWrite(PIN_B, B);
    //W - value for software PWM is used in this function
    
    // some check normaly inserted into mailn loop (here is placed for reduce CPU overload)
    check_save_values();
    
    // set output to HIGH on start of new cycle except zero value  
    if (W==0) digitalWrite(PIN_W, LOW);
    else digitalWrite(PIN_W, HIGH);
  }
  else {
    // set LOW value if raise to his boundary
    if (W==timer) digitalWrite(PIN_W, LOW);
  }
  timer++;
}

// ----------- SETUP ---------------------------------
void setup() {
  pinMode(PIN_R, OUTPUT);
  analogWrite(PIN_R, 0);
  pinMode(PIN_G, OUTPUT);
  analogWrite(PIN_G, 0);
  pinMode(PIN_B, OUTPUT);
  analogWrite(PIN_B, 0);
  pinMode(PIN_W, OUTPUT); 
  digitalWrite(PIN_W, LOW); // White pin is only digital (no analogWriteable)
  
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt

  // Example how to use direct control PWM at AT tiny85 by write into MCU registers
  // ------------------------------------------------------------------------------
  // SETUP
  //timer 0
  //TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  //TCCR0B = 0<<WGM02 | 3<<CS00; // Optional; already set
  //tmier 1
  // Configure counter/timer1 for fast PWM on PB4
  //GTCCR = 1<<PWM1B | 2<<COM1B0;
  //TCCR1 = 2<<COM1A0 | 7<<CS10;
  //OCR1C=255;
  // SET PWM VALUES
  //OCR0A=255; //PWM pin #D0 blue
  //OCR0B=254; //PWM pin #D1 red
  //OCR1B=0; //PWM pin #D4 green
}

//----------------- MAIN LOOP ---------------------
void loop() {
  pwm_step(); // one step of 
}

