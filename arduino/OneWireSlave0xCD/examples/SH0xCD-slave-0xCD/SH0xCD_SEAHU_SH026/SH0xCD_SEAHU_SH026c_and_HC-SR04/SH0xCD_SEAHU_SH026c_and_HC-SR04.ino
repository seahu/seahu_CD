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

   1-Wire interface for measure distance with utrasonic sensor HC-SR04 by use MCU AVR ATtiny85
   -------------------------------------------------------------------------------------------
   This is exaple how-to use universal 1-wire device with one didigtal and one analog/digital PIO for
   measure distance witch utrasonic sensor HC-SR04.
   In this example use one digital pin to triger pin of HC-SR04 sensor and analog pin configured as digial input coonect to echo pin.
   Features:
      - distance measure:  0ocm to 400cm +/-0.5cm
      - sampling:               5Hz
      - enable save measure corection value
      - enable set alarm condition for measured distance
      
   Device contain next sections:
      section0 :  distamce in cm
      section1 :  distance corection
      section4 :  identification led 
      section5 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device loADC default values from another project and switch on beeper)

   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )

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

              / \ +5V                                                                           / \ +5V
               |                                                                                 |
              +-+                                                                +---------------+
              | | 470R                                                           |               |
              +-+                                                               +-+  BAT         |
               |                                                                \ /  41         +-+ BAT
           <- +-+                                                               ---             \ / 41
           <- \ / led                                      +------+--------------|              ---
              ---                                          |      | 100nF        | 450uF         |
               |                                           |    -----          -----             |
               |                    AVR_ATtiny85           |    -----          -----             |
               |                   +------\/-----+         |      |              |              +-+
               |    (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC--+     ---            ---             | |20K
               +-------- A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire             +-+
 +- ANALOG_PIN <- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----------------|
 |                          GND 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)--+        +-+
 |                                 +-------------+                                     |        | |47K
 |                                                                               DIGITAL_PIN    +-+
 |                                                                                     |         |
 |                                                                                     |        ---       / \ +5V  sensor HC-SR04
 |                                                                                     |                   |       +---------------------------+
 |                                                                                     |                   +-------| vcc        --      --     |
 |                                                                                     +---------------------------| triger   /    \  /    \   |
 +-----------------------------------------------------------------------------------------------------------------| echo     \    /  \    /   |
                                                                                                           +-------| gnd        --      --     |
                                                                                                           |       +---------------------------+
                                                                                                          ---
*/

//  generate Device ID based on compile date and time
//#define __AVR_ATtiny85__ // more about suported platform in file suportedMCU.h of OneWireSlave0xCD library
                         // some platform as arduino NG or later, leonardo, micro, nano are auto detected by arduino IDE
//#define Enable_Analog_comapartor // enable Analog Comaparto for check loss power (check or update suportedMCU.h in OneWireSalve0x55 library) not use here
#include "DHT.h"
#include "OneWireSlave0xCD.cpp.h"

#define CONT_OF_SECTIONS 4 // number of section (sensors or actors)
// pins define for test into arduino nano
#ifdef __AVR_ATmega328P__
  #define __DEBUG__ // on arduino nano enable debug
  #define PIN_ECHO 7 // pin for echo
  #define PIN_TRIG 9 // pin for triger
  #define PIN_LED 13
  #define PIN_AIN 1 // for detect power lost
#endif
// pins define for ATtiny85
#ifdef __AVR_ATtiny85__
  #define PIN_ECHO 4 // pin for echo
  #define PIN_TRIG 0 // pin for triger
  #define PIN_LED 3
  #define PIN_AIN 1 // for detect power lost
#endif

#define REPEATE_INTERVAL 200 // interval beatwen two measured in [ms]

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU SH026c 1-Wire to ultrasonic distance sensor                ";
const char distance[] PROGMEM           = "Distance [cm]                                                    ";
const char corection_dist[] PROGMEM     = "Distance [cm] distance corection                                 ";
const char led[] PROGMEM                = "Led indication led 1=on 0=off                                    "; // indication led
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections                  "; // mainly for devolepment

//global variables
unsigned long time_last_measure=0;

  
//declaration measured functions 
void mMeasure(bool force_now); // temperature and humidity are measured together therefore share same function



/* 
 * define sections array (this array must be definet as global varialbe (because must be aviable throucht interruot). 
 *        For easy set control register is prepared next constants:
 *        C_READ      - actual value can be reADC
 *        C_WRITE     - actual value is writable for output devices
 *        C_ALARM     - section have alarm function
 *        C_MIN_ALARM - on min. value alarm  (can be change by One Wire)
 *        C_MAX_ALARM - on max. value alarm  (can be change by One Wire)
 *        C_MEMORY    - section not have value (value is always 0x00), insteADC it of value is stored into 32B RAM like user description of section
 *        C_BOOL      - section contain bool value
 *        C_U32BIT    - section contain unsigned 32 bit binary value
 *        C_32BIT     - section contain signed 32 bit binary value
*        
 *        (more about control byte (register) refer file OneWireSlave0xCD.h in OneWireSlave0xCD librarry)
 */
section sections[CONT_OF_SECTIONS]={
  // { contril byte os section           , actual value  , descrition      , user description buffer, measure function, lock }
  { C_READ +           C_ALARM + C_32BIT , {.i32=0}      , distance        , NULL                   , mMeasure        , false },
  { C_READ + C_WRITE +           C_32BIT,  {.i32=0}      , corection_dist  , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_BOOL,   {.b=0}        , led             , NULL                   , NULL            , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.b=0}        , Save            , NULL                   , NULL            , false },
};

typedef enum { DISTANCE, CORECTION_DIST, LED, SAVE }; // enumerate of intem in section array for clear human reADC code

//-------------- TO SERVE POWEROFF (thank analog comparator interrupt) ----

// this device do not use poweroff feature

//-------------- END TO SERVE POWEROFF ------------------------------------

//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------
// temperature and humidity are measured together therefore share same function
void mMeasure(bool force_now){
  union value0xCD newValue;
  long duration, distance; // Duration used to calculate distance
  unsigned int dist;

  
  if (force_now==false) return;
  // continue to measure
  if (sections[DISTANCE].control & C_READ != 0) { // is not set new measure flag then measure only by interval
    if (millis()-time_last_measure < REPEATE_INTERVAL ) return; // duration beatwen two measured must be min. REPEATE_INTERVAL.
  }
  digitalWrite(PIN_TRIG, LOW); 
  delayMicroseconds(2); 

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10); 
 
  digitalWrite(PIN_TRIG, LOW);
  duration = pulseIn(PIN_ECHO, HIGH, 40000); // timeout 0.04s
  if (duration==0) return 0;
 
  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration/58.2;

  // save new values
  newValue.i32 = distance + sections[CORECTION_DIST].actualValue.i32;
  ow.write_new_value(DISTANCE, newValue);
  #ifdef __DEBUG__
    Serial.print("Distance: "); 
    Serial.println(sections[DISTANCE].actualValue.i32);
  #endif
  time_last_measure=millis();
}


//------------------------ END MEASURE FUNCTIONS --------------------------


//------------------------ OTHER FUNCTIONS --------------------------------------------

// check led status
void check_led(){
  if (sections[LED].actualValue.b==1) digitalWrite(PIN_LED, LOW); // if setion led is on 
  else digitalWrite(PIN_LED, HIGH);
}

// check save
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
  #ifdef __DEBUG__
    Serial.begin(9600); 
    Serial.println("DHTxx test!");
  #endif
  pinMode(PIN_TRIG, OUTPUT);           // set triger pin to output
  digitalWrite(PIN_TRIG, LOW);       // default value for triger
  pinMode(PIN_ECHO, INPUT);           // set echo pin
  pinMode(PIN_LED, OUTPUT);
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
}

// main loop 
void loop() {
   ow.foreground_measure(false); // if argunet is true, then will be call every meseruent functions(force_now=true), otherwise
                                // wil be call only mesurent function witch section has set alarm or before was called as part
                                // of interrupt service, bat return whithou measurment.
  check_led(); // check led status
  check_save_values(); //save values if need
  delay(1);
}

