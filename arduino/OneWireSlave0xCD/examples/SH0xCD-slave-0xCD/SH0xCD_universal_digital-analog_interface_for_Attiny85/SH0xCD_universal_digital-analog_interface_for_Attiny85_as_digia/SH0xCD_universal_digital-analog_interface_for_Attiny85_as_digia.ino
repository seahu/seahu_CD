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

   1-Wire interface for universal digital-analog with MCU AVR ATtiny85 as universal digital-anlog device
   -----------------------------------------------------------------------------------------------------
   This is universal 1-wire device with one didigtal and one analog/digital PIO. This PIO can be use meny diferentaly way.
   In this example will be use as one digital input/output port with counter and one analog input.
   This is usefull with join with many arduino sensors.
   This device save actual values on poweroff events.
   Device contain next sections:
      section0 :  read from digital pin
      section1 :  write into digital pin (digital pin is open drain write to 1 enale pin use also as input)
      section2 :  digital pin counter (increase with decrease signal edge)
      section3 :  anlog intup 10-bit resolution 0-5V (0=0V 1023=5V)
      section4 :  anlog intup correction value 
      section5 :  identification led 
      section6 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device loADC default values from another project and switch on beeper)
      section7 :  reset conter

      PS: I wont to ADCd gain for ADC, but gain is aviable only for diferential inpusts beatben pins PB3 and PB4 or PB5 and PB2. It is suitable for this circuit.
          Eventualy I can would be use diferent reference voltage for ADC. My be later
    
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
    ANALOG_PIN <- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----------------|
                         GND D5 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)--+        +-+
                                   +-------------+                                     |        | |47K
                                                                                 DIGITAL_PIN    +-+
                                                                                                 |
                                                                                                ---
*/

//  generate Device ID based on compile date and time
#define __AVR_ATtiny85__ // more about suported platform in file suportedMCU.h of OneWireSlave0xCD library
                         // some platform as arduino NG or later, leonardo, micro, nano are auto detected by arduino IDE
//#define Enable_Analog_comapartor // enable Analog Comaparto for check loss power (check or update suportedMCU.h in OneWireSalve0x55 library)
#include "OneWireSlave0xCD.cpp.h"

#define CONF_AC      {ACSR &= ~(1<<ACD); ACSR |= (1<<ACBG)| (1<<ACI) | (1<<ACIE) | (1<<ACIS1)| (1<<ACIS0); ADCSRB&=~(1<<ACME);} // prepare analog comparator to check loss power
#define AC_INT       ISR(ANA_COMP_vect) // the analog comparator interrupt service routine


#define CONT_OF_SECTIONS 8 // number of section (sensors or actors)
#define PIN_IN 0 // IN, OUT and COUNT is same pin
#define PIN_OUT PIN_IN // IN, OUT and COUNT is same pin
#define PIN_COUNT PIN_IN // IN, OUT and COUNT is same pin
#define PIN_ANALOG A2
#define PIN_LED 3
#define PIN_AIN 1 // for detect power lost

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU 1-Wire universal digital-analog board 1xPIO + 1xIN-ANLOG Ing. Ondrej Lycka";
const char pio_in[] PROGMEM          = "PIO-IN digital pin";
const char pio_out[] PROGMEM          = "PIO-OUT digital pin";
const char counter[] PROGMEM            = "Counter digital pin";
const char adc[] PROGMEM                = "ADC analog pin";
const char corection[] PROGMEM          = "Setter corection signed value for ADC";
const char led[] PROGMEM                = "Led indication led 1=on 0=off"; // indication led
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment
const char counter_off[] PROGMEM        = "Counter counter_poweroff";

//global variables
bool last_conter_status=HIGH;
  
//declaration measured functions 
void mPio_in(bool force_now);
void mADC(bool force_now);


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
  { C_READ +           C_ALARM + C_BOOL  , {.b=0}        , pio_in          , NULL                   , mPio_in         , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.b=0}        , pio_out         , NULL                   , NULL            , false },
  { C_READ + C_WRITE + C_ALARM + C_U32BIT, {.u32=0}      , counter         , NULL                   , NULL            , false },
  { C_READ +           C_ALARM + C_U32BIT, {.u32=0}      , adc             , NULL                   , mADC            , false },
  { C_READ + C_WRITE +           C_32BIT,  {.i32=0}      , corection       , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_BOOL,   {.b=0}        , led             , NULL                   , NULL            , false },
  { C_READ + C_WRITE +         + C_BOOL  , {.b=0}        , Save            , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , counter_off     , NULL                   , NULL            , false }
};

typedef enum { PIO_IN, PIO_OUT, COUNTER, A_TO_D, CORECTION, LED, SAVE, COUNTER_OFF }; // enumerate of intem in section array for clear human reADC code

//-------------- TO SERVE POWEROFF (thank analog comparator interrupt) ----

// ATTENCTION - arduino function analogRead change settion for analog comparator, who is used here (therefore after analogread I must repeatly config analog comparator)
AC_INT {
  sections[COUNTER_OFF].actualValue.u32=(sections[COUNTER_OFF].actualValue.u32)+1; // directly increment COUNTER_OFF
  ow.save_values();
  sections[LED].actualValue.b=1;
  digitalWrite(PIN_LED, LOW);   // turn the LED on 
  
}
//-------------- END TO SERVE POWEROFF ------------------------------------

//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------
// Pio_in  measure function
void mPio_in(bool force_now){
  union value0xCD mewValue; // space to temporary store new value
  // reADC value of pin is short operation that may be done immediately during serve interrupt
  if ( digitalRead(PIN_IN)==HIGH) mewValue.b=true;
  else                            mewValue.b=false;
  ow.write_new_value(PIO_IN, mewValue);
  return;
}

// ADC  measure function
void mADC(bool force_now){
  union value0xCD mewValue; // space to temporary store new value
  // reADC value of pin is short operation that may be done immediately during serve interrupt
  mewValue.u32 = analogRead(PIN_ANALOG) + sections[CORECTION].actualValue.i32;  // reADC the analog input + corection
  CONF_AC; // arduino analogRead() change analog comparator config, thereforemust be again set
  ow.write_new_value(A_TO_D, mewValue); // write new value with all test (rtc. min, max, lock, clear measurent flag...)
  return;
}

//------------------------ END MEASURE FUNCTIONS --------------------------


//------------------------ OTHER FUNCTIONS --------------------------------------------
//chcek PIO_OUT
void check_pio_out(){
  if (sections[PIO_OUT].actualValue.b==1) digitalWrite(PIN_OUT, HIGH); // if setion led is on 
  else digitalWrite(PIN_OUT, LOW);
}

//chcek counter
void check_counter(){
  union value0xCD newValue; // space to temporary store new value
  switch(last_conter_status){
    case HIGH:
      if (digitalRead(PIN_COUNT)==LOW) {
        last_conter_status=LOW;
        newValue.u32=(sections[COUNTER].actualValue.u32)+1;
        ow.write_new_value(COUNTER, newValue); // write new value with all test (rtc. min, max, lock...)
       }
       break;
    case LOW:
      last_conter_status=digitalRead(PIN_COUNT);
      break;
  }
}

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

//pokus comparator
//void pokuss(){
//  bool comp_out=(ACSR & (1 << ACO));       //Reading ACO comparator output bit
//  if(comp_out==1) sections[LED].actualValue.b=1;
//}
//------------------------ END OTHER FUNCTIONS --------------------------------------------


// setup
void setup() {
  pinMode(PIN_IN, INPUT);           // set pin to input
  digitalWrite(PIN_IN, HIGH);       // turn on pullup resistors (PIN_IN is same as PIN_OUT)
  pinMode(PIN_ANALOG, INPUT);
  pinMode(PIN_LED, OUTPUT);
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
  CONF_AC;
}

// main loop 
void loop() {
   ow.foreground_measure(true); // if argunet is true, then will be call every meseruent functions(force_now=true), otherwise
                                // wil be call only mesurent function witch section has set alarm or before was called as part
                                // of interrupt service, bat return whithou measurment.
  check_pio_out();
  check_counter();
  check_led(); // check led status
  check_save_values(); //save values if need
  delay(10);
}

