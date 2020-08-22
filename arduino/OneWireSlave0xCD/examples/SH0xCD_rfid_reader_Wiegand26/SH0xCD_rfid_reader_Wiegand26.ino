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

   1-Wire interface for RFID CARD READER with Wiengand26 comunication
   -------------------------------------------------------------------
   This device create interface between 1_Wire bus and RFID card reader with Wiengand 26bit communication.
   Except Winegand intergace this device include control beeper and led includet into card reader + control door lock.
   Device contail next section:
      section0 :  card reader, card code is stored  in user note
      section1 :  always equal 0. Write to 1 to acutal_value start predefined access allow event (beep, on green led and on door lock for predefinet time, defined in check_start_allow_acceess() code bellow)
      section2 :  always equal 0. Write to 1 to acutal_value start predefined access deny event (beep for predefinet time, defined in check_start_denny_access() code bellow)
      section3 :  control green led 0=off, 1=on
      section4 :  control beep 0=off, 1=on
      section5 :  control door lock led 0=close, 1=open
      section6 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)

   Feaures:
    - card code encode to format used i my work. (if you need anther format card code edit recode() function)
    - after read new card activate alarm
    
   OneWire famyly code: 0xCD (more about devices with tgis family code on https:\\   )



                                          ATMEL ATMEGA328P                                                CARD READER (Wiengand26 compatible)
                                         +-------\/-------+                                             +---------------+
                           (RESET)      -|1  PC6    PC5 28|- A5   (SCL) ----> D1  <---------------- D1 -|               |
                           (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA) ----> D0  <---------------- D1 -|               |
                           (TXD)     D1 -|3  PD1    PC3 26|- A3                           <------ beep -|               |
                           (INT0)    D2 -|4  PD2    PC2 25|- A2                           <------- led -|               |
    !prefered for OW! (OC2B,INT1)   #D3 -|5  PD3    PC1 24|- A1                                VCC 12V -|               |
                                     D4 -|6  PD4    PC0 23|- A0                                    GND -|               |
                                        -|7  VCC    GND 22|-                                            +---------------+
                                        -|8  GND   AREF 21|-  
                                        -|9  PB6   AVCC 20|-  
                                        -|10 PB7    PB5 19|- D13  (SCK)
            beep <----- (OC0B,T1)   #D5 -|11 PD5    PB4 18|- D12  (MISO)
            led  <--- (OC0A,AIN0)   #D6 -|12 PD6    PB3 17|- #D11 (MOSI,OC2A)
                           (AIN1)    D7 -|13 PD7    PB2 16|- #D10 (OC1B)
                                     D8 -|14 PB0    PB1 15|- #D9  (OC1A)
                                         +----------------+





   
*/

//  generate Device ID based on compile date and time

#include "SH0xCD_platform.h"

#define CONT_OF_SECTIONS 7 // number of section (sensors or actors)

#define PIN_D0   A4
#define PIN_D1   A5
#define PIN_LED  6
#define PIN_BEEP 5
#define PIN_LOCK 13 // I have not door lock at least trun on/off arduino led

#define ACCESS_ALLOW_BEEP 100 // time in ms of beep acustic access signal
#define ACCESS_ALLOW_LED 2000 // time in ms of optical access signal
#define ACCESS_ALLOW_LOCK 3000 // time in ms of open lock
#define ACCESS_DENNY_BEEP 1000 // time in ms of beep acustic denny signal

byte bit_mask=1;
byte bit_count=0;
byte max_bit_count=26;
byte cur=0;
byte buf_len=6;
byte buff[6];
int last_D0=1;
int last_D1=1;
unsigned long last_time=millis();
unsigned long stopLED=0;
unsigned long stopBEEP=0;
unsigned long stopLOCK=0;


OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
//                                         1234567890123456789012345678901234567890123456789012345678901234 (max 64B)
const char device_description[] PROGMEM = "SEAHU RFID READER INTERFACE between 1-Wire and Wiegand26";
const char Card_reader[] PROGMEM        = "String Card_code";
const char Access_allow[] PROGMEM       = "Switch button on-start_acces_allow_signalization+open_door";
const char Access_denny[] PROGMEM       = "Switch button on-start_acces_deny_signalization";
const char Led[] PROGMEM                = "Switch Led Green_acces_led";
const char Beep[] PROGMEM               = "Switch Beep";
const char Door_lock[] PROGMEM          = "Switch Lock door lock";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment

// card code buffer
char card_code_buf[32];


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
  { C_READ + C_WRITE + C_ALARM + C_MEMORY, {.u32=0}      , Card_reader     , card_code_buf   , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Access_allow    , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Access_denny    , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Led             , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Beep            , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Door_lock       , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL            , false            , false},
};
typedef enum { CARD_READER, ACCESS_ALLOW, ACCESS_DENNY, LED, BEEP, DOOR_LOCK, SAVE}; // enumerate of intem in section array for clear human read code

//------------------------ MEASURE FUNCTIONS --------------------------
// not necessary
//------------------------ END MEASURE FUNCTIONS --------------------------


//------------------------ SETUP --------------------------------------------
void setup() {
  // setup pins
  // make the pushbutton's pin an input:
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  digitalWrite(PIN_LED, HIGH);
  digitalWrite(PIN_BEEP, HIGH);
  digitalWrite(PIN_LOCK, HIGH);
  pinMode(PIN_LED, OUTPUT);
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

void check_begin(){
  // check end of data or long pause
  if (bit_count==max_bit_count || (millis()-last_time)>10000 ){
    Serial.print("bit_count:");
    Serial.println(bit_count);
    bit_mask=1;
    cur=0;
    bit_count=0;
    memset (buff,0,buf_len);
    Serial.println("");
  }
  last_time=millis();
}

void nex_bit(){
  bit_mask=bit_mask<<1;
  if (bit_mask==0){
    bit_mask=1;
    cur++;
  }  
}

void read_D0(){
  check_begin();
  buff[cur]=buff[cur]&(~bit_mask);
  nex_bit();
  bit_count++;
}

void read_D1(){
  check_begin();
  buff[cur]=buff[cur]|bit_mask;
  nex_bit();
  bit_count++;
}

bool sum_parity(byte bit_mask_, byte cur_, byte len){
  bit_mask=bit_mask_;
  cur=cur_;
  byte sum=0;
  for (byte i=0; i<len; i++){
    if ((byte)(buff[cur]&bit_mask)!=0) sum++;
    nex_bit();
  }
  return (sum&0x01);
}

bool check_parity(){
    // packet contains 26 bit:
    // | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8 | 9  | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 | 24 | 25 | 26 | [bit]
    // | smb|                           24 bit data                                                                                |lsb |
    // | smb|           <---first 12 bit data ---->                    |           <---secount 12 bit data ---->                   |lsb |<-lsb is negate value of secount 12 bit data parity
    // |         <--byte 0-->                 |          <--byte 1-->                 |         <--byte 2-->                  |          <--byte 3-->                 | [byte]
    bool msb=buff[0]&0x01; // bit 0 versus parity frist 12 data bits
    bool lsb=buff[3]&0x02; // last negate bit versus parity last 12 data bits
    bool sum_msb=sum_parity(0x02,0,12); // sum parity frist 12 data bits
    bool sum_lsb=sum_parity(0x20,1,12)^0x01; // sum parity last 12 data bits (negate result)
    Serial.print ("parita msb:");
    Serial.println (msb);
    Serial.print ("sum_msb:");
    Serial.println (sum_msb);
    Serial.print ("parita lsb:");
    Serial.println (lsb);
    Serial.print ("sum_lsb:");
    Serial.println (sum_lsb);
    if (msb!=sum_msb || lsb!=sum_lsb) {
      Serial.println ("Parity Error.");
      return false;
    }
    Serial.println ("Parity OK.");
    return true;
}

void print_binary_code(){
    cur=0;
    bit_mask=1;
    for (byte i=0; i<26; i++){
      if ((byte)(buff[cur]&bit_mask)==0) Serial.print("0");
      else Serial.print("1");
      nex_bit();
    }
    Serial.println("");
    cur=0;
    bit_mask=1;
    bit_count=0;
}

void recode(){
    memset(card_code_buf,0,32); // null string buffer
    cur=0;
    bit_mask=0x02; //satrt with second bit
    for (byte i=0; i<3; i++){
      for (byte j=0; j<2; j++){
        byte L=0;
        for (byte k=0; k<4; k++){
          if ((byte)(buff[cur]&bit_mask)) L=L|0x10; // logic 1
          L=L>>1;
          nex_bit();
        }
        //Serial.print(L,HEX);
        sprintf(card_code_buf,"%s%X",card_code_buf,L);
      }
    }
    Serial.println(card_code_buf);
}

void check_start_allow_acceess(){
  // if section access_allow is on
  if (sections[ACCESS_ALLOW].actualValue.b==1){ // if setion access allow is on 
    ow.write_new_value(ACCESS_ALLOW, setBollValue(0)); // section acces_Allow off
    //led
    digitalWrite(PIN_LED, LOW); // switch led on
    ow.write_new_value(LED, setBollValue(1)); // section led on
    //beep
    digitalWrite(PIN_BEEP, LOW);
    ow.write_new_value(BEEP, setBollValue(1)); // section beep on
    // door_lock
    digitalWrite(PIN_LOCK, LOW);
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
    digitalWrite(PIN_BEEP, LOW);
    ow.write_new_value(BEEP, setBollValue(1)); // section beep on
    // section beep on
    stopBEEP=millis()+ACCESS_DENNY_BEEP;
  }
}

void check_stop_allow_deny_access(){
  if (stopLED!=0) {
    if (millis()>stopLED){
      digitalWrite(PIN_LED, HIGH);
      ow.write_new_value(LED, setBollValue(0)); // section led off
      stopLED=0;
    }
  }
  if (stopBEEP!=0){
    if (millis()>stopBEEP) {
      digitalWrite(PIN_BEEP, HIGH);
      ow.write_new_value(BEEP, setBollValue(0)); // section beep off
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
  if  (sections[LED].actualValue.b==1) digitalWrite(PIN_LED, LOW);
  else digitalWrite(PIN_LED, HIGH);
  if  (sections[BEEP].actualValue.b==1) digitalWrite(PIN_BEEP, LOW);
  else digitalWrite(PIN_BEEP, HIGH);
  if  (sections[DOOR_LOCK].actualValue.b==1) digitalWrite(PIN_LOCK, LOW);
  else digitalWrite(PIN_LOCK, HIGH);
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
  // read the input pin:
  int statusD0 = digitalRead(PIN_D0);
  int statusD1 = digitalRead(PIN_D1);
  if (last_D0!=statusD0) {
    if (statusD0==0) read_D0(); // falinf edge
    last_D0=statusD0;
  }
  if (last_D1!=statusD1) {
    if (statusD1==0) read_D1(); // falinf edge
    last_D1=statusD1;
  }
  if (bit_count==max_bit_count ){
    print_binary_code();
    if (check_parity()==false) {
      Serial.println ("Parity Error.");
      return;
    }
    else {
      Serial.println ("Parity OK.");
      recode(); //write card code to card_buffer
      sections[CARD_READER].control|=C_ALARM_STATUS; // set alarm
      _OW_alarm=true; // set global alarm
    }
    cur=0;
    bit_mask=1;
    bit_count=0;    
  }
  check_start_allow_acceess();
  check_start_denny_access();
  check_stop_allow_deny_access();
  check_outputs();
  check_save_velus();
}
