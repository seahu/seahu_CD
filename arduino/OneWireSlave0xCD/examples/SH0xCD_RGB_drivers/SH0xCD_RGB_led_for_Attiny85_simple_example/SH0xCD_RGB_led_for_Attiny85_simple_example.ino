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

   1-Wire interface for control RGB LED (using MCU AVR ATtiny85)
   ---------------------------------------------------------------
   This device enable control RGB led or RGB leds strip.
   Device contain next sections:
      section0 :  actually color (as RGB number in binary format)
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
#include "OneWireSlave0xCD.cpp.h"

#define CONT_OF_SECTIONS 2 // number of section (sensors or actors)
#define PIN_R 1
#define PIN_G 0
#define PIN_B 4
#define PIN_W 3 //(not use in this example)

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU 1-Wire to RGB C2020 by Ing. Ondrej Lycka";
const char rgb[] PROGMEM                = "RGB {0-16777215} value=Blue+256*Green+65536*Red";
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
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , rgb             , NULL                   , false           , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL                   , false           , false},
};
typedef enum { RGB, SAVE }; // enumerate of intem in section array for clear human read code


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


// setup
void setup() {
  pinMode(PIN_R, OUTPUT);
  analogWrite(PIN_R, 0);
  pinMode(PIN_G, OUTPUT);
  analogWrite(PIN_G, 0);
  pinMode(PIN_B, OUTPUT);
  analogWrite(PIN_B, 0);
  pinMode(PIN_W, OUTPUT); //(not use in this example)
  digitalWrite(PIN_W, LOW); //(not use in this example)
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
  //OCR0A=2; //PWM pin #D0
  //OCR0B=2; //PWM pin #D1
  //OCR1B=2; //PWM pin #D4
}

// main loop 
void loop() {
  uint32_t Rgb;
  uint8_t R,G,B,W;
  
  Rgb=sections[RGB].actualValue.u32;
  B=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  G=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate 
  R=(uint8_t)(Rgb & 0xFF); // first 8 bits
  Rgb=Rgb>>8 ; // 8 bits rotate (not use in this example)
  W=(uint8_t)(Rgb & 0xFF); // first 8 bits (not use in this example)
  
  analogWrite(PIN_R, R);;
  analogWrite(PIN_G, G);
  analogWrite(PIN_B, B);
  
  
  //save values if need
  check_save_values();
  delay(50);
}
