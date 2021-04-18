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
   
   Device contain next sections:
      section0 :  actually color (as RGBW number in binary format)
      section1 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)
    
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
#include "OneWireSlave0xCD.h"

#define CONT_OF_SECTIONS 2 // number of section (sensors or actors)
#define PIN_R 1
#define PIN_G 0
#define PIN_B 4
#define PIN_W 3 //(not use in this example)

byte W=0; //actual white pwm value
byte timer=0;

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU 1-Wire to RGBW C2021 by Ing. Ondrej Lycka";
const char rgbw[] PROGMEM                = "RGBW {0-4294967295} value=Blue+256*Green+65536*Red+16777216*white";
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
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , rgbw             , NULL                   , false           , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL                   , false           , false},
};
typedef enum { RGBW, SAVE }; // enumerate of intem in section array for clear human read code


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
 * detect new RGBW values
 */
void detect_new_vales_from_1wire(){
  uint32_t Rgb;
  uint8_t R,G,B; // W is global
  
  Rgb=sections[RGBW].actualValue.u32;
  B=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  G=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  R=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate
  W=(uint8_t)(Rgb & 0xFF); // first 8 bits
  // write new value into hardware PWM timers
  analogWrite(PIN_R, R);;
  analogWrite(PIN_G, G);
  analogWrite(PIN_B, B);
  // software pwm for white use global variable W
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

