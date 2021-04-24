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

   This I use in my flat as multi sensor and actor
   -------------------------------------------------------------
   For measure hot and cold water, water and gas sensor, control vlave for water (cold and hot), valve for gas and switch ventilator in my flat.
   For ths I use arduino nano.
   
   Device use internal analog comparator to detect poweroff for save counter data on EEPROM (EEPROM have only ~100 000 revretes).
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
      CounterA <-------- A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire             +-+
      CounterB <- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----------------|
                         GND D5 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)--+        +-+
                                   +-------------+                                     |        | |47K
                                                                                  CounterC      +-+
                                                                                                 |
                                                                                                ---




                                          ATMEL ATMEGA328P
                                         +-------\/-------+
                           (RESET)      -|1  PC6    PC5 28|- A5   (SCL)   
                           (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)   
                           (TXD)     D1 -|3  PD1    PC3 26|- A3      
                           (INT0)    D2 -|4  PD2    PC2 25|- A2
    !prefered for OW! (OC2B,INT1)   #D3 -|5  PD3    PC1 24|- A1
                                     D4 -|6  PD4    PC0 23|- A0
                                        -|7  VCC    GND 22|-  
                                        -|8  GND   AREF 21|-  
                                        -|9  PB6   AVCC 20|-  
                                        -|10 PB7    PB5 19|- D13  (SCK)
                        (OC0B,T1)   #D5 -|11 PD5    PB4 18|- D12  (MISO)
                      (OC0A,AIN0)   #D6 -|12 PD6    PB3 17|- #D11 (MOSI,OC2A)
                           (AIN1)    D7 -|13 PD7    PB2 16|- #D10 (OC1B)
                                     D8 -|14 PB0    PB1 15|- #D9  (OC1A)
                                         +----------------+





   
*/

//  generate Device ID based on compile date and time

#include "OneWireSlave0xCD.cpp.h"

#define CONT_OF_SECTIONS 15 // number of section (sensors or actors)

#define PIN_ALARM_GAS A1
#define PIN_ALARM_WATER A0

#define PIN_RELAY1 12
#define PIN_RELAY2 11
#define PIN_RELAY3 10
#define PIN_RELAY4 9
#define PIN_RELAY5 8
#define PIN_RELAY6 4
#define PIN_RELAY7 6
#define PIN_RELAY8 5

#define PIN_COUNT_A A3
#define PIN_COUNT_B A2

OneWireSlave0xCD ow; //define One Wire object


// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU multi sensor/actor for my flat in Prostejov Czech Republic C2020 by Ing. Ondrej Lycka";
const char Alarm_gas[] PROGMEM         = "Alarm gas_alarm intput_of_arduino_pin_4";
const char Alarm_water[] PROGMEM         = "Alarm water_alarm intput_of_arduino_pin_5";
const char rele1[] PROGMEM         = "Switch rele1 Napajeni_ventilu_pro_vodu";
const char rele2[] PROGMEM         = "Switch rele2 ON_OFF_ventil_voda";
const char rele3[] PROGMEM         = "Switch rele3 ON_OFF_ventil_TUV";
const char rele4[] PROGMEM         = "Switch rele4 Napajeni_ventilu_pro_plyn";
const char rele5[] PROGMEM         = "Switch rele5 +_kontakt_ventilu_plynu";
const char rele6[] PROGMEM         = "Switch rele6 -_kontakt_ventilu_plynu";
const char rele7[] PROGMEM         = "Switch rele7 ventilator_WC";
const char rele8[] PROGMEM         = "Switch rele8 ventilator_centralni";
const char automat_gas[] PROGMEM   = "Switch ON_OFF atomatic_serve_gas_alarm";
const char automat_water[] PROGMEM  = "Switch ON_OFF atomatic_serve_water_alarm ";
const char counter_a[] PROGMEM      = "Counter counter_A intput_of_arduino_pin_11";
const char counter_b[] PROGMEM      = "Counter counter_B intput_of_arduino_pin_12";
const char counter_off[] PROGMEM    = "Counter counter_poweroff";


bool last_conter_status_A=HIGH;
bool last_conter_status_B=HIGH;

//declaration measured functions 
// void mCounter_a(bool force_now); // not necessary, counter is changing its value continuously alone (in main loop)



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
  // { contril byte os section           , actual value  , descrition      , memory          , measure function , lock }
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele1           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele2           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele3           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele4           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele5           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele6           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele7           , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , rele8           , NULL            , false            , false},
  { C_READ +           C_ALARM + C_BOOL  , {.u32=0}      , Alarm_gas       , NULL            , false            , false},
  { C_READ +           C_ALARM + C_BOOL  , {.u32=0}      , Alarm_water     , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , automat_gas     , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , automat_water   , NULL            , false            , false},
  { C_READ + C_WRITE + C_ALARM + C_U32BIT, {.u32=0}      , counter_a       , NULL            , false            , false},
  { C_READ + C_WRITE + C_ALARM + C_U32BIT, {.u32=0}      , counter_b       , NULL            , false            , false},
  { C_READ + C_WRITE           + C_U32BIT, {.u32=0}      , counter_off     , NULL            , false            , false}
};
typedef enum { RELAY1, RELAY2, RELAY3, RELAY4, RELAY5, RELAY6, RELAY7, RELAY8,ALARM_GAS, ALARM_WATER, AUTOMAT_GAS, AUTOMAT_WATER, COUNTER_A, COUNTER_B, COUNTER_OFF }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------

// not necessary

//------------------------ END MEASURE FUNCTIONS --------------------------


//-------------- TO SERVE POWEROFF (thank analog comparator interrupt) ----
AC_INT {
  sections[COUNTER_OFF].actualValue.u32=(sections[COUNTER_OFF].actualValue.u32)+1; // directly increment COUNTER_OFF
  ow.save_values();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
}
//-------------- END TO SERVE POWEROFF ------------------------------------

// setup
void setup() {
  // setup pins
  pinMode(13, OUTPUT);
  // PIN_ALARM_GAS
  digitalWrite(PIN_ALARM_GAS, HIGH); // turn on pullup resistors
  pinMode(PIN_ALARM_GAS, INPUT);
  // PIN_ALARM_WATER
  digitalWrite(PIN_ALARM_WATER, HIGH); // turn on pullup resistors
  pinMode(PIN_ALARM_WATER, INPUT);
  
  // PIN_RELAY1 (LOW-OFF, HIGH-ON relay)
  digitalWrite(PIN_RELAY1, LOW);
  pinMode(PIN_RELAY1, OUTPUT);
  // PIN_RELAY2
  digitalWrite(PIN_RELAY2, LOW);
  pinMode(PIN_RELAY2, OUTPUT);
  // PIN_RELAY3
  digitalWrite(PIN_RELAY3, LOW);
  pinMode(PIN_RELAY3, OUTPUT);
  // PIN_RELAY4
  digitalWrite(PIN_RELAY4, LOW);
  pinMode(PIN_RELAY4, OUTPUT);
  // PIN_RELAY5
  digitalWrite(PIN_RELAY5, LOW);
  pinMode(PIN_RELAY5, OUTPUT);
  // PIN_RELAY6
  digitalWrite(PIN_RELAY6, LOW);
  pinMode(PIN_RELAY6, OUTPUT);
  // PIN_RELAY7
  digitalWrite(PIN_RELAY7, LOW);
  pinMode(PIN_RELAY7, OUTPUT);
  // PIN_RELAY8
  digitalWrite(PIN_RELAY8, LOW);
  pinMode(PIN_RELAY8, OUTPUT);

  // PIN_COUNT_A
  digitalWrite(PIN_COUNT_A, HIGH); // turn on pullup resistors
  pinMode(PIN_COUNT_A, INPUT);
  // PIN_COUNT_B
  digitalWrite(PIN_COUNT_B, HIGH); // turn on pullup resistors
  pinMode(PIN_COUNT_B, INPUT);
  
  // setup One Wire
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface for run bacground throught interrupt and load saved seting from EEPROM or on frist run set defaut values

  //setup analog comare for detect poweroff 
  CONF_AC // (defined in suportedMCU.h)

  Serial.begin(9600);
}

void check_input(uint8_t pin, uint8_t Section){
  union value0xCD newValue; // space to temporary store new value
  if (digitalRead(pin)==LOW) newValue.b=0;
  else newValue.b=1;
  if (sections[Section].actualValue.b!=newValue.b) ow.write_new_value(Section, newValue); // write change if requirement
}

void check_output(uint8_t pin, uint8_t Section){ // LOW-OFF relay , HIGH-ON relay
  if (sections[Section].actualValue.b==1) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);  
}

void chcek_counter(uint8_t pin, uint8_t Section, bool *last_conter_status){
  union value0xCD newValue; // space to temporary store new value
  bool actual_pin_status;

  actual_pin_status=digitalRead(pin); // actual pin status
  switch(*last_conter_status){
    case HIGH:
      if (actual_pin_status==LOW) {
        *last_conter_status=LOW;
        newValue.u32=(sections[Section].actualValue.u32)+1;
        ow.write_new_value(Section, newValue); // write new value with all test (rtc. min, max, lock...)
        Serial.println(newValue.u32);
       }
       break;
    case LOW:
      *last_conter_status=actual_pin_status;
      break;
   } 
}

union value0xCD setBollValue(bool value) {
  union value0xCD newValue;
  newValue.b=value;
  return newValue;
}

// main loop 
void loop() {
  // CHECK PIN_ALARM_GAS
  check_input(PIN_ALARM_GAS, ALARM_GAS);
  // CHECK PIN_ALARM_WATER
  check_input(PIN_ALARM_WATER, ALARM_WATER);
  // CHECK PIN_RELAY1-5
  check_output(PIN_RELAY1, RELAY1);
  check_output(PIN_RELAY2, RELAY2);
  check_output(PIN_RELAY3, RELAY3);
  check_output(PIN_RELAY4, RELAY4);
  check_output(PIN_RELAY5, RELAY5);
  check_output(PIN_RELAY6, RELAY6);
  check_output(PIN_RELAY7, RELAY7);
  check_output(PIN_RELAY8, RELAY8);
  // CHECK COUNTER_A
  chcek_counter(PIN_COUNT_A, COUNTER_A, &last_conter_status_A);
  // CHECK COUNTER_B
  chcek_counter(PIN_COUNT_B, COUNTER_B, &last_conter_status_B);

  // SERVE GAS ALARM IF IS ALLOW
  if (sections[AUTOMAT_GAS].actualValue.b==1){ // if allow gas server alarm
    if (digitalRead(PIN_ALARM_GAS)==LOW) { // if gas alarm => svitch on WC ventialtor, central ventila and switch off gas valve
      // switch on ventilators
      ow.write_new_value(RELAY7, setBollValue(1));
      ow.write_new_value(RELAY8, setBollValue(1));
      // switch off gas valve
      ow.write_new_value(RELAY4, setBollValue(1));
      ow.write_new_value(RELAY5, setBollValue(1));
      ow.write_new_value(RELAY6, setBollValue(1));
    }
    else {
      // switch off ventilators
      ow.write_new_value(RELAY7, setBollValue(0));
      ow.write_new_value(RELAY8, setBollValue(0));
      // switch on gas valve
      ow.write_new_value(RELAY4, setBollValue(0));
      ow.write_new_value(RELAY5, setBollValue(0));
      ow.write_new_value(RELAY6, setBollValue(0));    
    }
  }
  // SERVE WATER ALARM IF IS ALLOW
  if (sections[AUTOMAT_WATER].actualValue.b==1){ // if allow water server alarm
    if (digitalRead(PIN_ALARM_WATER)==HIGH) { // if gas alarm => svitch on WC ventialtor, central ventila and switch off gas valve
      // switch off water valve (could and hot water)
      ow.write_new_value(RELAY1, setBollValue(1)); // power 230V for valves
      ow.write_new_value(RELAY2, setBollValue(0)); // cloud valve
      ow.write_new_value(RELAY3, setBollValue(0)); // hot valve
    }
    else {
      // switch off water valve (could and hot water)
      ow.write_new_value(RELAY1, setBollValue(0)); // power 230V for valves
      ow.write_new_value(RELAY2, setBollValue(0)); // cloud valve
      ow.write_new_value(RELAY3, setBollValue(0)); // hot valve
    }
  }



}
