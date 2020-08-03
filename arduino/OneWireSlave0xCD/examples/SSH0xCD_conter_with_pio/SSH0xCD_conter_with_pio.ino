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

   This example implement conter with one input and one output pin, all into one OneWire device.
   ----------------------------------------------------------------------------------------------
   Example is for AVR_ATtiny can be also used on arduino leonardo or micro and nano (default One Wire pin for other platform refer to suportedMCU.h).
   Hardware platform and some options must be set into header (SH0xCD_platform.h) file.
   Counter use internal analog comparator to detect poweroff for save counter data on EEPROM (EEPROM have only ~100 000 revretes).
   OneWire famyly code: 0xCD (more about devices with tgis family code on https:\\   )


                                                                                                / \ +5V
                                                                                                 |
                                                                                 +---------------+
                                                                                 |               |
                                                                                +-+  BAT         |
                                                                                \ /  41         +-+ BAT
                                                                                ---             \ / 41
                                                           +------+--------------|              ---
                                                           |      | 100nF        | 450uF         |
                                                           |    -----          -----             |
                                    AVR_ATtiny85           |    -----          -----             |
                                   +------\/-----+         |      |              |              +-+
                    (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC--+     ---            ---             | |20K
       PIN_OUT <-------- A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire             +-+
       PIN_IN  <- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----------------|
                         GND D5 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)--+        +-+
                                   +-------------+                                     |        | |47K
                                                                                   PIN_COUNT    +-+
                                                                                                 |
                                                                                                ---
   
*/

//  generate Device ID based on compile date and time

#include "SH0xCD_platform.h"

#define CONT_OF_SECTIONS 4 // number of section (sensors or actors)
#define PIN_OUT 3
#define PIN_IN 6
#define PIN_COUNT 5



OneWireSlave0xCD ow; //define One Wire object


// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU CONTERS WITH PIO EXAMPLE VERSION 0.01 C2020 by Ing. Ondrej Lycka";
const char pio_in[] PROGMEM         = "PIO-IN intput_of_arduino_pin_6";
const char pio_out[] PROGMEM        = "PIO-OUT output_of_arduino_pin_7";
const char counter[] PROGMEM        = "Counter";
const char counter_off[] PROGMEM    = "Counter poweroff";

bool last_conter_status=HIGH;

//declaration measured functions 
void mPio_in(bool force_now);
void mPio_out(bool force_now);
void mCounter(bool force_now);

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
 *
 *        Default user descriptions is stored into EEPROM (NULL value in sections table). 
 *        If you need can be stored into RAM, but must be declared buffer and said in sections table.
 */
section sections[CONT_OF_SECTIONS]={
  // { contril byte os section           , actual value  , descrition      , user description buffer, measure function, lock }
  { C_READ           + C_ALARM + C_BOOL  , {.b=0}        , pio_in          , NULL                   , mPio_in         , false },
  { C_READ + C_WRITE + C_ALARM + C_BOOL  , {.b=0}        , pio_out         , NULL                   , mPio_out        , false },
  { C_READ + C_WRITE + C_ALARM + C_U32BIT, {.u32=0}      , counter         , NULL                   , mCounter        , false },
  { C_READ +                   + C_U32BIT, {.u32=0}      , counter         , NULL                   , NULL            , false }
};
typedef enum { PIO_IN, PIO_OUT, COUNTER, COUNTER_OFF }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------

// Pio_in  measure function
void mPio_in(bool force_now){
  union value0xCD mewValue; // space to temporary store new value
  // read value of pin is short operation that may be done immediately during serve interrupt
  if ( digitalRead(PIN_IN)==HIGH) mewValue.b=true;
  else                            mewValue.b=false;
  ow.write_new_value(PIO_IN, mewValue);
  return;
}

// Pio_out  measure function
void mPio_out(bool force_now){
  union value0xCD mewValue; // space to temporary store new value
  // write value of pin is short operation that may be done immediately during serve interrupt
  if ( (sections[PIO_OUT].actualValue.b)==true ) digitalWrite(PIN_IN, HIGH);
  else                                           digitalWrite(PIN_IN, LOW);
  ow.write_new_value(PIO_OUT, (sections[PIO_OUT].actualValue)); // write the same value as actual value (write_new_value do clear measurent flag)
  return;
}

// Count measure function
void mCounter(bool force_now){
  union value0xCD newValue; // space to temporary store new value
  // check counter is short operation that may be done even immediately during serve interrupt
  switch(last_conter_status){
    case HIGH:
      if (digitalRead(PIN_COUNT)==LOW) {
        last_conter_status==LOW;
        newValue.u32=(sections[COUNTER].actualValue.u32)+1;
        ow.write_new_value(COUNTER, newValue); // write new value with all test (rtc. min, max, lock...)
       }
       break;
    case LOW:
      last_conter_status==digitalRead(PIN_COUNT);
      break;
  }
}
//------------------------ END MEASURE FUNCTIONS --------------------------


//-------------- TO SERVE POWEROFF (thank analog comparator interrupt) ----
AC_INT {
  sections[COUNTER_OFF].actualValue.u32=(sections[COUNTER_OFF].actualValue.u32)+1; // directly increment COUNTER_OFF
  ow.save_values();
}
//-------------- END TO SERVE POWEROFF ------------------------------------

// setup
void setup() {
  // setup pins
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  pinMode(PIN_COUNT, INPUT);
  digitalWrite(PIN_COUNT, HIGH); // turn on pullup resistors
   
  // setup One Wire
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface for run bacground throught interrupt and load saved seting from EEPROM or on frist run set defaut values

  //setup analog comare for detect poweroff 
  CONF_AC // (defined in suportedMCU.h)
}


// main loop 
void loop() {
   union value0xCD newValue; // space to temporary store new value
   // serve measrment functions
   ow.foreground_measure(true); // if argunet is true, then will be call every meseruent functions(force_now=true), otherwise
                                // wil be call only mesurent function witch section has set alarm or before was called as part
                                // of interrupt service, bat return whithou measurment.
   // .
   // ..
   
   // do what you want (but do not disable interrupt, if do known what you do)
   // ..
   // .   
}