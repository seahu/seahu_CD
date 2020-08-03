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

   Example is for arduino nano or leonardo (for other platform refer or update file suportedMCU.h in OneWireSlave0xCD librarry)
*/

//  generate Device ID based on compile date and time

#include "OneWireSlave0xCD.h"

#define CONT_OF_SECTIONS 8 // number of section (sensors or actors)
#define PIN_OUT 7
#define PIN_IN 6
#define PIN_PWM 5
#define PIN_ADC A0

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU multi sensor/actor DEVICE EXAMPLE VERSION 0.01 C2020 by Ing. Ondrej Lycka";
const char humidity[] PROGMEM       = "Humidity [%]";
const char temperature[] PROGMEM    = "Temperature [x.1 C]";
const char temp_calibrate[] PROGMEM = "Setter temp_calibrate [x.1 C]";
const char pio_in[] PROGMEM         = "PIO-IN intput_of_arduino_pin_6";
const char pio_out[] PROGMEM        = "PIO-OUT output_of_arduino_pin_7";
const char pwm[] PROGMEM            = "PWM 8-bit_arduion_pin_5";
const char adc[] PROGMEM            = "ADC 16-bit_arduino_pin_A0";
const char lcd[] PROGMEM            = "String LCD_nonochrome {1x32}";
// default user descriptions is stored into EEPROM. If you need can be stored into RAM, but must be declared buffer and said in sections table.
char lcd_buf[32]                    ="-PRINT TO LCD 32B BUFFER IN RAM-"; 

  
//declaration measured functions 
void mHumidity(bool force_now);
void mTemperature(bool force_now);
void mPio_in(bool force_now);
void mPio_out(bool force_now);
void mPwm(bool force_now);
void mAdc(bool force_now);

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
  { C_READ           + C_ALARM + C_U32BIT, {.u32=0}      , humidity        , NULL                   , mHumidity       , false },
  { C_READ           + C_ALARM + C_32BIT , {.i32=0}      , temperature     , NULL                   , mTemperature    , false },
  { C_READ + C_WRITE           + C_32BIT , {.i32=0}      , temp_calibrate  , NULL                   , NULL            , false },
  { C_READ           + C_ALARM + C_BOOL  , {.b=0}        , pio_in          , NULL                   , mPio_in         , false },
  { C_READ + C_WRITE + C_ALARM + C_BOOL  , {.b=0}        , pio_out         , NULL                   , mPio_out        , false },
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , pwm             , NULL                   , mPwm            , false },
  { C_READ + C_WRITE + C_ALARM + C_U32BIT, {.u32=0}      , adc             , NULL                   , mAdc            , false },
  { C_READ + C_WRITE           + C_MEMORY, {.u32=0}      , lcd             , lcd_buf                , mAdc            , false },
};
typedef enum { HUMIDITY, TEMPERATURE, TEMP_CALIBRATE, PIO_IN, PIO_OUT, PWM, AD_CONVERT, LCD }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------

// Humidity measure function
void mHumidity(bool force_now){
  if (force_now==false) return; // force_now=true means that this function is called as part interrup service and must be a short as is posible. If measrment take more time, than leave for later from main loop.
  union value0xCD newValue; // space to temporary store new value
  // .
  // ..
  // your own code for measurin humidity
  // ..
  // .
  newValue.u32=250; // result of measurment
  ow.write_new_value(HUMIDITY, newValue); // write new value into setion HUMIDITY with all tests (alarm min,alarm max,change,lock one wire)
}

// Temperature measure function
void mTemperature(bool force_now){
  if (force_now==false) return; // force_now=true means that this function is called as part interrup service and must be a short as is posible. If measrment take more time, than leave for later from main loop.
  union value0xCD newValue; // space to temporary store new value
  // .
  // ..
  // your own code for measurin temperature
  // ..
  // .
  newValue.u32=250; // result of measurment
  newValue.u32=250 - (sections[TEMP_CALIBRATE].actualValue.u32); // apply calibrate value from TEMP_CALIBRATE section
  ow.write_new_value(TEMPERATURE, newValue);
}

// Pio_in  measure function
void mPio_in(bool force_now){
  union value0xCD newValue; // space to temporary store new value
  // read value of pin is short operation that may be done immediately during servre interrupt
  if ( digitalRead(PIN_IN)==HIGH) newValue.b=true;
  else                            newValue.b=false;
  ow.write_new_value(PIO_IN, newValue);
  return;
}

// Pio_out  measure function
void mPio_out(bool force_now){
  union value0xCD newValue; // space to temporary store new value
  // write value of pin is short operation that may be done immediately during servre interrupt
  if ( (sections[PIO_OUT].actualValue.b)==true ) digitalWrite(PIN_IN, HIGH);
  else                                           digitalWrite(PIN_IN, LOW);
  ow.write_new_value(PIO_OUT, (sections[PIO_OUT].actualValue)); // write the same value as actual value (write_new_value do clear measurent flag)
  return;
}

// PWM  measure function
void mPwm(bool force_now){
  union value0xCD newValue; // space to temporary store new value
  // write pwm value TO pin is short operation that may be done immediately during servre interrupt
  analogWrite(PIN_PWM, (sections[PWM].actualValue.u32));
  if ( (sections[PIO_OUT].actualValue.b)==true ) digitalWrite(PIN_IN, HIGH);
  ow.write_new_value(PWM, (sections[PWM].actualValue)); // write the same value as actual value (write_new_value do clear measurent flag)
  return;
}

// Adc measure function
void mAdc(bool force_now){
  if (force_now==false) return; // force_now=true means that this function is called as part interrup service and must be a short as is posible. If measrment take more time, than leave for later from main loop.
  union value0xCD newValue; // space to temporary store new value
  newValue.u32=analogRead(PIN_ADC); // result of measurment (may take some time)
  ow.write_new_value(AD_CONVERT, newValue);
  return;
}
//------------------------ END MEASURE FUNCTIONS --------------------------



// setup
void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_ADC, INPUT);
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
}



// main loop 
void loop() {
  ow.foreground_measure(false); // if argunet is true, then will be call every meseruent functions(force_now=true), otherwise
                                // wil be call only mesurent function witch section has set alarm or before was called as part
                                // of interrupt service, bat return whithou measurment.
   // refresh PIO_OUT as far is possible (if change value by One Wire)
   if ( (sections[PIO_OUT].actualValue.b)==true ) digitalWrite(PIN_IN, HIGH);
   else                                           digitalWrite(PIN_IN, LOW);
   // print_lcd(lcd_buf); // refresh lcd 
   
   // .
   // ..
   // do what you want (but do not disable interrupt, if do known what you do)
   // ..
   // .
   delay(50);
}
