/*
   OneWireSlave0xCD - A skeleton 1-Wire multisensor emulator for AVR Microcontroller
   Copyright (C) 2020 Ondrej Lycka info (at) seahu.cz

   The skeleton requries an incllude file with the device-specific dependent command handling.
   
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

   1-Wire interface for RFID CARD READER with RDM6300 module
   ---------------------------------------------------------
   This device create interface between 1_Wire bus and RFID card reader (RDM6300 modelue)
   Except reader interface this device include control beeper and RGB led  + control door lock.
   This device come from SH0xCD_rfid_reader_Wiegand26 and try keep max. of compatibility. Therefore sections for control red and blue led is on end of list sections.
   Device contail next section:
      section0 :  card reader, card code is stored  in user note
      section1 :  always equal 0. Write to 1 to acutal_value start predefined access allow event (beep, on green led and on door lock for predefinet time, defined in check_start_allow_acceess() code bellow)
      section2 :  always equal 0. Write to 1 to acutal_value start predefined access deny event (beep for predefinet time, defined in check_start_denny_access() code bellow)
      section3 :  control green led 0=off, 1=on
      section4 :  control beep 0=off, other values mean time lenght of sound wave in [us]
      section5 :  control door lock led 0=close, 1=open
      section6 :  control red led 0=off, 1=on
      section7 :  control blue led 0=off, 1=on
      section8 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)

   Feaures:
    - card code encode to format used i my work. (if you need anther format card code edit recode() function)
    - after read new card activate alarm
    - for easy use have prepared sekvencies for allow and deny access
    
   OneWire famyly code: 0xCD (more about devices with tgis family code on https:\\   )

          RFID READER 
       +--------------+
  ant -|              |                                             ATMEL ATMEGA328P              
  ant -|  RDM6300     |-> +5V                                      +-------\/-------+             
       |              |-> GND                        (RESET)      -|1  PC6    PC5 28|- A5   (SCL) 
  GND -|              |-> NC                  +----> (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA) 
  VCC -|              |-> RX                  |      (TXD)     D1 -|3  PD1    PC3 26|- A3         
  LED -|              |-> TX  <---------------+      (INT0)    D2 -|4  PD2    PC2 25|- A2   
       +--------------+       !prefered for OW! (OC2B,INT1)   #D3 -|5  PD3    PC1 24|- A1   
                                                               D4 -|6  PD4    PC0 23|- A0   
                                                                  -|7  VCC    GND 22|-      
                                                                  -|8  GND   AREF 21|-  
                                                                  -|9  PB6   AVCC 20|-  
                                                                  -|10 PB7    PB5 19|- D13  (SCK)
                                      beep <----- (OC0B,T1)   #D5 -|11 PD5    PB4 18|- D12  (MISO)
                                green led  <--- (OC0A,AIN0)   #D6 -|12 PD6    PB3 17|- #D11 (MOSI,OC2A)
                                 blue led  <-------  (AIN1)    D7 -|13 PD7    PB2 16|- #D10 (OC1B)
                                  red led  <---------------    D8 -|14 PB0    PB1 15|- #D9  (OC1A) ------> door lock
                                                                   +----------------+





   
*/

//  generate Device ID based on compile date and time

#include "OneWireSlave0xCD.cpp.h"

#define WORK_CARD_BUFFER_SIZE 12 // size of card buffer for module RDM6300

#define CONT_OF_SECTIONS 9 // number of section (sensors or actors)

#define PIN_LED_RED  8
#define PIN_LED_GREEN  6
#define PIN_LED_BLUE  7
#define PIN_BEEP 5
#define PIN_LOCK 9

#define ACCESS_ALLOW_BEEP 100 // time in ms of beep acustic access signal
#define ACCESS_ALLOW_LED 2000 // time in ms of optical access signal
#define ACCESS_ALLOW_LOCK 3000 // time in ms of open lock
#define ACCESS_DENNY_BEEP 1000 // time in ms of beep acustic denny signal
#define NEW_CARD_LED 100 // time in ms of led on in after detec nee card
#define NEW_CARD_BEEP 100 // time in ms of beep in after detec nee card

byte buffer_index;
bool change;
unsigned long last_time=millis();
unsigned long last_beep=micros();
unsigned long stopLED=0;
unsigned long stopBEEP=0;
unsigned long stopLOCK=0;
const uint8_t c_orig[] PROGMEM  ={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
const uint8_t c_new[] PROGMEM   ={'0','8','4','C','2','A','6','E','1','9','5','D','3','B','7','F'};


OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
//                                         1234567890123456789012345678901234567890123456789012345678901234 (max 64B)
const char device_description[] PROGMEM = "SEAHU RFID READER INTERFACE between 1-Wire and RDM6300 module";
const char Card_reader[] PROGMEM        = "String Card_code";
const char Access_allow[] PROGMEM       = "Switch button on-start_acces_allow_signalization+open_door";
const char Access_denny[] PROGMEM       = "Switch button on-start_acces_deny_signalization";
const char Led_green[] PROGMEM          = "Switch Led Green_acces_led";
const char Beep[] PROGMEM               = "Value Beep_duration [us]";
const char Door_lock[] PROGMEM          = "Switch Lock door lock";
const char Led_red[] PROGMEM            = "Switch Led Red_acces_led";
const char Led_blue[] PROGMEM           = "Switch Led Blue_acces_led";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment

// card code buffer
char work_card_code_buf[WORK_CARD_BUFFER_SIZE];
char final_card_code_buf[32];


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
  { C_READ + C_WRITE + C_ALARM + C_MEMORY, {.u32=0}      , Card_reader     , final_card_code_buf, false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Access_allow    , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Access_denny    , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Led_green       , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_U32BIT, {.u32=0}      , Beep            , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Door_lock       , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Led_red         , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Led_blue        , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL            , false            , false},
};
typedef enum { CARD_READER, ACCESS_ALLOW, ACCESS_DENNY, LED_GREEN, BEEP, DOOR_LOCK, LED_RED, LED_BLUE, SAVE}; // enumerate of intem in section array for clear human read code

//------------------------ MEASURE FUNCTIONS --------------------------
// not necessary
//------------------------ END MEASURE FUNCTIONS --------------------------


//------------------------ SETUP --------------------------------------------
void setup() {
  // setup pins
  // make the pushbutton's pin an input:
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_BEEP, LOW);
  digitalWrite(PIN_LOCK, LOW);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BEEP, OUTPUT);
  pinMode(PIN_LOCK, OUTPUT);
  // on arduino nano I can't use interrupt becouse aviable is only two pins fro interrupt pin2 and pin3 - pin3 I use pro 1-wire communocation already
  //attachInterrupt(digitalPinToInterrupt(PIN_D0), read_D0, FALLING);     // hardware interrupt - high to low pulse
  //attachInterrupt(digitalPinToInterrupt(PIN_D1), read_D1, FALLING);     // hardware interrupt - high to low pulse
  
  // setup One Wire
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface for run bacground throught interrupt and load saved seting from EEPROM or on frist run set defaut values

  //setup analog comare for detect poweroff 
  //CONF_AC // (defined in suportedMCU.h)

  Serial.begin(9600);
  Serial.println("start");  
}

//------------------------ END SETUP --------------------------------------------

//------------------------ FUNCTIONS --------------------------------------------
union value0xCD setBollValue(bool value) {
  union value0xCD newValue;
  newValue.b=value;
  return newValue;
}

union value0xCD setU32Value(unsigned long value) {
  union value0xCD newValue;
  newValue.u32=value;
  return newValue;
}

uint8_t convert_char(uint8_t hex){
  uint8_t i;

  for (i=0; i<16; i++){
    if (hex==pgm_read_byte_near(c_orig + i)) return pgm_read_byte_near(c_new + i);
  }
  return 0;
}

void recode(){ // recode to format used fi. duha
    memset(final_card_code_buf,0,32); // null string buffer
    for (byte i=0; i<(WORK_CARD_BUFFER_SIZE-2); i++){ // two last 2 bytes is not part of card code
      final_card_code_buf[i]=convert_char(work_card_code_buf[i]);
    }
    sections[CARD_READER].control|=C_ALARM_STATUS; // set alarm
    _OW_alarm=true; // set global alarm
    Serial.println(final_card_code_buf);
    //led
    digitalWrite(PIN_LED_BLUE, HIGH); // switch led on
    ow.write_new_value(LED_BLUE, setBollValue(1)); // section led on
    //beep
    ow.write_new_value(BEEP, setU32Value(1000)); // section beep on
    // set led and beep time
    stopLED=millis()+NEW_CARD_LED;
    stopBEEP=millis()+NEW_CARD_BEEP;
}


void rfid_read(){ // read one byte from module RDM6300 and gradually process card code (card code start by byte=0x02 next 10bytes card code (as string representated hex values of card code) + 2 byte check control (I do not use) and stop by byte=0x03)
  char value;
  if (Serial.available()>0) {
    value=Serial.read();
    //Serial.print(buffer_index);
    //Serial.print(" ");
    //Serial.println((byte) value);
    if (value==2) {
      if ( abs((millis()-last_time))>1000 ) { // enable repead only one per second
        Serial.println("cas");
        memset(work_card_code_buf,0,WORK_CARD_BUFFER_SIZE); // null string buffer
      }
      buffer_index=0;
      change=false;
      return;
    }
    if (value==3) { // tag has been fully transmitted
      if (change==true) {
        recode();
        last_time=millis();
      }
      return;
    }
    if (buffer_index >= WORK_CARD_BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
      Serial.println("Error: Buffer overflow detected! ");
      memset(work_card_code_buf,0,WORK_CARD_BUFFER_SIZE); // null string buffer
      buffer_index=0;
    }
    if ( work_card_code_buf[buffer_index]!=value ) change=true;
    work_card_code_buf[buffer_index] = value; // everything is alright => copy current value to buffer
    buffer_index++;
  }
}


void check_start_allow_acceess(){
  // if section access_allow is on
  //Serial.println(sections[ACCESS_ALLOW].actualValue.b);
  if (sections[ACCESS_ALLOW].actualValue.b==1){ // if setion access allow is on 
    ow.write_new_value(ACCESS_ALLOW, setBollValue(0)); // section acces_Allow off
    Serial.println("ACCESS_ALLLOW");  
    //led
    digitalWrite(PIN_LED_GREEN, HIGH); // switch led on
    ow.write_new_value(LED_GREEN, setBollValue(1)); // section led on
    //beep
    ow.write_new_value(BEEP, setU32Value(1000)); // section beep on
    // door_lock
    digitalWrite(PIN_LOCK, HIGH);
    ow.write_new_value(DOOR_LOCK, setBollValue(1)); // section door_lock on
    // section lock on
    stopLED=millis()+ACCESS_ALLOW_LED;
    stopBEEP=millis()+ACCESS_ALLOW_BEEP;
    stopLOCK=millis()+ACCESS_ALLOW_LOCK;
    // section allow off
  }
}

void check_start_denny_access(){
  // if section access_deny is on
  if  (sections[ACCESS_DENNY].actualValue.b==1){ // if setion access deny is on 
    ow.write_new_value(ACCESS_DENNY, setBollValue(0)); // section acces_Allow off
    Serial.println("ACCESS_DENNY");  
    //led
    digitalWrite(PIN_LED_RED, HIGH); // switch led on
    ow.write_new_value(LED_RED, setBollValue(1)); // section led on
    //beep
    ow.write_new_value(BEEP, setU32Value(500)); // section beep on
    // set time out
    stopLED=millis()+ACCESS_DENNY_BEEP;
    stopBEEP=millis()+ACCESS_DENNY_BEEP;
  }
}

void check_stop_allow_deny_access(){
  if (stopLED!=0) {
    if (millis()>stopLED){
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_BLUE, LOW);
      ow.write_new_value(LED_RED, setBollValue(0)); // section led off
      ow.write_new_value(LED_GREEN, setBollValue(0)); // section led off
      ow.write_new_value(LED_BLUE, setBollValue(0)); // section led off
      stopLED=0;
    }
  }
  if (stopBEEP!=0){
    if (millis()>stopBEEP) {
      ow.write_new_value(BEEP, setU32Value(0)); // section beep off
      stopBEEP=0;
    }
  }
  if (stopLOCK!=0){
    if (millis()>stopLOCK) {
      digitalWrite(PIN_LOCK, HIGH);
      ow.write_new_value(DOOR_LOCK, setBollValue(0)); // section door_lock off
      stopLOCK=0;
    }
  }
}

void check_outputs(){
  if  (sections[LED_RED].actualValue.b==1) digitalWrite(PIN_LED_RED, HIGH);
  else digitalWrite(PIN_LED_RED, LOW);
  if  (sections[LED_GREEN].actualValue.b==1) digitalWrite(PIN_LED_GREEN, HIGH);
  else digitalWrite(PIN_LED_GREEN, LOW);
  if  (sections[LED_BLUE].actualValue.b==1) digitalWrite(PIN_LED_BLUE, HIGH);
  else digitalWrite(PIN_LED_BLUE, LOW);
  if  (sections[DOOR_LOCK].actualValue.b==1) digitalWrite(PIN_LOCK, HIGH);
  else digitalWrite(PIN_LOCK, LOW);
}

void check_beep(){
  if  (sections[BEEP].actualValue.u32==0) {
    digitalWrite(PIN_BEEP, LOW);
    return;
  }
  if ( abs(micros()-last_beep) > sections[BEEP].actualValue.u32 ){
    if (digitalRead(PIN_BEEP)==HIGH) digitalWrite(PIN_BEEP, LOW);
    else digitalWrite(PIN_BEEP, HIGH);
    last_beep=micros();
  }
}

void check_save_velus(){
  // if section access_allow is on
  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    ow.write_new_value(SAVE, setBollValue(0)); // section Save off
    ow.save_values();
    Serial.println("Values all sections was benn saved.");
  }
}


//------------------------ END FUNCTIONS --------------------------------------------

//------------------------ MAIN LOOP --------------------------------------------- 
void loop() {
  rfid_read();
  // check 1-wire values
  check_start_allow_acceess();
  check_start_denny_access();
  check_stop_allow_deny_access();
  check_outputs();
  check_beep();
  check_save_velus();
}
