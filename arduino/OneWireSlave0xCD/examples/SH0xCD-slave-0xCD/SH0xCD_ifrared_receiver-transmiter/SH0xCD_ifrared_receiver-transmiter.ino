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

   This example implement inrared receiver-transmiter as OneWire device.
   --------------------------------------------------------------------
   This device create interface between 1-Wire bus and InfraRed receiver and transmiter. You can use this device for scaning signal from most IR remote control or
   control other devis by scaned IR signals. You can read the detected IR signal from sector 0 as a string of binary data in hexadecimal display.
   Conversely, to transmit an IR signal, you must enter its hexadecimal resenatation in section 0.
   Any further reading or writing from/to section0 must be separated by writing an empty string to this section.
   The other sections are only auxiliary.
   Device contail next section:
      section0 :  string buffer of binary data in hexadecimal display
      section1 :  number of retransmissions
      section2 :  transmition mode 0=38KHz 1=36KHz
      section3 :  led indication IR activity 1=enable 0-disable
      section4 :  led idetification of 1-Wire device 1=on 0-off
      section5 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)

   Features:
    - reivre IR from most types remote controls
    - transmite emulte most types remote controls
    - allows you to set the number of retransmissions for better transmission stability
    - transmite enabe select two modes 36KHz or 38KHz
    - enable led indication IR activity
    - enable led idetification of 1-Wire device
    - enable save actual setting value into EEPROM
    
   Example is for AVR_ATtiny85 can be also used on arduino leonardo or micro and nano (default One Wire pin for other platform refer to suportedMCU.h).
   Hardware platform and some options must be set into header (SH0xCD_platform.h) file.
   
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

                                                                                                / \ +5V
                                                                                                 |
   / \ +5V                                                                       +---------------+
    |                                                                            |               |
   +-+                                                                           |               |
   | |                                                                           |              +-+ -> IR diode
   +-+                                                                           |              \ / ->
    |                                                      +------+--------------|              ---
   +-+  -> LED                                             |      | 100nF        | 450uF         |
   \ /  ->                                                 |    -----          -----             |
   ---                              AVR_ATtiny85           |    -----          -----             |
    |                              +------\/-----+         |      |              |              +-+
    |               (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC--+     ---            ---             | |100R
  +---- PIN_OUT <------- A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire             +-+
  | +- PIN_IN  <- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----------------+      / \+5V
  |                      GND D5 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)                    |   +-+
  |                                +-------------+                                                       +---| |
  +------------------------------------------------------------------------------------+---------------------| | <- IR sensor
                                                                                                      GND ---| | <-
                                                                                                             +-+
                                                                                                         
*/

//  generate Device ID based on compile date and time
//#define __DEBUG__

// define constants
#define CONT_OF_SECTIONS 6 // number of section (sensors or actors)
#define LEN_IR_BUF 32
#define TOLERAN 2
#define MAX_SLEEP 25000

// platform settings
#ifdef __AVR_ATtiny85__
  // first prototype
  //#define PIN_RECV 4
  //#define PIN_IR_LED 1
  //#define PIN_LED 3
  //#define Timer0_OCR0A // force own (no default) MCU settings
  // secound prototype
  #define PIN_RECV 3
  #define PIN_IR_LED 1
  #define PIN_LED 4
  #define Timer0_OCR0A // force own (no default) MCU settings
#endif
#ifdef __AVR_ATmega328P__
  #define PIN_RECV 7
  #define PIN_IR_LED 9
  #define PIN_LED 13
#endif
#include "OneWireSlave0xCD.cpp.h"

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
const char device_description[] PROGMEM = "SEAHU Infrared receiver/transmiter v1 C2021 by Ing. Ondrej Lycka";
const char ir_hex_buf[] PROGMEM         = "STRING hex code of IR signal";
const char tx_repeate[] PROGMEM         = "Counter Number of retransmissions must be >=1";
const char tx_mode[] PROGMEM            = "SWITCH transmiter 0=38KHz 1=36KHz";
const char info_led[] PROGMEM           = "SWITCH info led 1=enable 0-disable";
const char led[] PROGMEM                = "SWITCH identification 1=on 0-off";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment

char ir_buf[LEN_IR_BUF];
bool hex_code_lock=false;
bool last_conter_status=HIGH;
bool tx=false; // transmitter status
byte time_start0;
byte time_start1;
byte time_data0;
byte time_data1;
byte len;
byte data[20];
byte recv_last=1;
byte mask=0x01;
byte stage=0; // position of decode code 
              // 0 - wait to new start signal
              // 1 - save lengh of first start signal
              // 2 - save lengh of secound start signal
              // 3 - save lengh of frist datat signal (saved as zero)
              // 4 - save leng of secound diferent leng of signal (saved as one)
unsigned long last_time=micros();
unsigned long new_time=micros();


//declaration measured functions 
// not use here

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
  // { contril byte os section           , actual value  , descrition , user description buffer, measure function, lock }
  { C_READ + C_WRITE + C_ALARM + C_MEMORY, {.b=0}        , ir_hex_buf , ir_buf                 , NULL            , false },
  { C_READ + C_WRITE           + C_U32BIT, {.u32=1}      , tx_repeate , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_BOOL  , {.b=0}        , tx_mode    , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_BOOL  , {.b=0}        , info_led   , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_BOOL  , {.b=0}        , led        , NULL                   , NULL            , false },
  { C_READ + C_WRITE           + C_BOOL  , {.b=0}        , Save       , NULL                   , NULL            , false }
};
typedef enum { IR_BUF, TX_REPEATE, TX_MODE, INFO_LED, LED, SAVE }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------

// this examle has not any measure function

//------------------------ END MEASURE FUNCTIONS --------------------------
//-------------- TO SERVE POWEROFF (thank analog comparator interrupt) ----
// not use here
//AC_INT {
//  sections[COUNTER_OFF].actualValue.u32=(sections[COUNTER_OFF].actualValue.u32)+1; // directly increment COUNTER_OFF
//  ow.save_values();
//}
//-------------- END TO SERVE POWEROFF ------------------------------------

// ------------- AUXLIARY INFRARED FUNCTIONS -------------------------------

/* 
 * Set on hardware 38KHz/36KHz for pin with transmitter infrared diode
 */
void on_3XKHz() {
  #ifdef __AVR_ATtiny85__
    //Attiny85 , running @ 16MHZ
    //using timer 1 and pin D1 (PB1)
    pinMode(PIN_IR_LED, OUTPUT);
    digitalWrite(PIN_IR_LED, HIGH);
    TCNT1 = 0;
    TCCR1 = 0;
    GTCCR |= (1 << PSR1); //section 13.3.2 reset the prescaler
    TCCR1 |= (1 << CTC1); // section 12.3.1 CTC mode
    TCCR1 |= (1 << COM1A0); //togle pin PB1 table 12-4
    TCCR1 |= (1 << CS11); //prescaler 2 table 12-5
    if  (sections[TX_MODE].actualValue.b==0){ // is set 38KHz mode
      OCR1C = 105; // (16 000 000 / (2*105) = 72072 *2 = 38 095 Hz (for 36KHz use value 111)
      OCR1A = 104;
    }
    else {  // is set 36KHz mode
      OCR1C = 111;
      OCR1A = 110;
    }
  #endif
  #ifdef __AVR_ATmega328P__
    // Tmega328P, running @ 16MHZ
    //using timer 1 (16bit) and pin D9 (PB1)
    pinMode(PIN_IR_LED, OUTPUT);
    digitalWrite(PIN_IR_LED, HIGH);
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12); // WGM10=0, WGM11=0, WGM12=1, WGM13=0 table 15-5. CTC mode
    TCCR1A |= (1 << COM1A0); // COM1A0=1, COM1A1=0 togle pin PB1 table 15-2.
    TCCR1B |= (1 << CS10); // CS10=1, CS11=0, CS12=0 prescaler 1 table 12-6.
    //if  (sections[TX_MODE].actualValue.b==0)  OCR1A = 210; // (16 000 000 / (210) = 72072 *2 = 38 095 Hz (for 36KHz use value 220)
    //else                                      OCR1A = 220;
    OCR1A = 210;
  #endif
  if  (sections[INFO_LED].actualValue.b==1)  { // server information led
    if (sections[LED].actualValue.b==0)  digitalWrite(PIN_LED,LOW);
    else digitalWrite(!PIN_LED,HIGH); // if led is constant on use inverse value
  }
  tx=true;
}

/* 
 * Set off hardware 38KHz/36KHz
 */
void off_3XKHz() {
  #ifdef __AVR_ATtiny85__
    TCCR1 &= ~( (1 << COM1A1) | (1 << COM1A0) ); //diconected OC1A from pin PB1 table 12-4
  #endif
  #ifdef __AVR_ATmega328P__
    TCCR1A &= ~( (1 << COM1A1) | (1 << COM1A0) ); //diconected OC1A from pin PB1 table 15-2
  #endif
  if  (sections[INFO_LED].actualValue.b==1)  { // server information led
    if (sections[LED].actualValue.b==0)  digitalWrite(PIN_LED,HIGH);
    else digitalWrite(!PIN_LED,LOW); // if led is constant on use inverse value
  }
  tx=false;
}

/*
 * print binary data - only for debug
 */
void print_bin_data(){
  #ifdef __DEBUG__      
    byte mask;
    byte i;
    while ( i < len) { // i*8 because every byte has 8 bits (len is in bits)
      mask=0x01;
      while (mask!=0 && i < len){
        
          if ( (data[i/8] & mask) == 0 ) Serial.print("0");
          else                         Serial.print("1");
        
      }
    }
  #endif
}

/*
 * Convert number value to char representtive this value in hex format
 */

byte val_to_hex(byte val){
  if( val < 10) return val + 48;
  else          return val + 55;
}

/*
 * Convert binary byte to two chars representtive byte in hex format
 * return buf pointer to next chars
 */
char *byte_to_hex(byte val, char*buf){
  buf[0]=val_to_hex(val >> 4);
  buf[1]=val_to_hex(val & 0x0F);
  return buf+2;
}

/*
 * Binary data to hex prepresenatation
 */
void data_to_hex(char *buf){
  byte i=0;
  memset(buf, 0, LEN_IR_BUF);
  buf = byte_to_hex(time_start0, buf);
  buf = byte_to_hex(time_start1, buf);
  buf = byte_to_hex(time_data0,  buf);
  buf = byte_to_hex(time_data1,  buf);
  buf = byte_to_hex(len,         buf);
  while ( (i*8) < len) { // i*8 because every byte has 8 bits (len is in bits)
    buf = byte_to_hex(data[i++], buf);
  }
}

/*
 * Convert char in hex format to binary value
 */
byte hex_to_val(byte hex){
  if ( hex >= 65 ) return (hex-55);
  else             return  (hex-48);
}

/*
 * Convert two chars representtive value in hex format to byte value and return it
 */
byte char_to_byte(char * buf){
  byte val;
  val = hex_to_val(buf[0]) << 4;
  val = val + hex_to_val(buf[1]);
  return val;
}

/*
 * Convert string in hex format to binary data
 */
void hex_to_data(char *buf){
  byte i=0;
  time_start0=char_to_byte(buf);
  time_start1=char_to_byte(buf+2);
  time_data0=char_to_byte(buf+4);
  time_data1=char_to_byte(buf+6);
  len       =char_to_byte(buf+8);
  buf=buf+10;
  while ( (i*8) < len) { // i*8 because every byte has 8 bits (len is in bits)
    data[i]=char_to_byte(buf +i*2 );
    i++;
  }
}

/*
 * Change transmite status
 */
void change_tx(){
  if(tx==true) off_3XKHz();
  else on_3XKHz();
}

/*
 * Transmite binary data
 */
void transmit_bin_data(){
  byte tmp;
  byte mask;
  byte i;
  byte n;
  unsigned int st0=time_start0*40;
  unsigned int st1=time_start1*40;
  unsigned int dt0=time_data0*10;
  unsigned int dt1=time_data1*10;
  stage=6;
  for (n=0; n < sections[TX_REPEATE].actualValue.u32; n++){ // repeate sending
    on_3XKHz(); // start transmiter status
    //delayMicroseconds(time_start0*40);
    delayMicroseconds(st0);
    change_tx();
    //delayMicroseconds(time_start1*40);
    delayMicroseconds(st1);
    change_tx();
    byte i=0;
    while ( i < len) { // i*8 because every byte has 8 bits (len is in bits)
      mask=0x01;
      while (mask!=0 && i < len){
        //if ( (data[i/8] & mask) == 0 ) delayMicroseconds(time_data0*10);
        //else                           delayMicroseconds(time_data1*10);
        if ( (data[i/8] & mask) == 0 ) delayMicroseconds(dt0);
        else                           delayMicroseconds(dt1);
        change_tx();
        mask = mask << 1;
        i++;
      }
    }
    off_3XKHz(); // on end must be off tramsmiter
    delayMicroseconds(15000); // radio calm
  }
  stage=0;
}


/*
 * Write binary data to 1-Wire buf into hex buf
 */
void write_data_to_ir_buf(){
  memset(ir_buf, 0, LEN_IR_BUF);
  //hex_to_data(ir_buf);
  data_to_hex(ir_buf);
  #ifdef __DEBUG__
    Serial.println(ir_buf);
     //print_bin_data(){; // in debug mode print binary data
    Serial.println("");
  #endif
  sections[IR_BUF].control|=C_ALARM_STATUS; // set alarm
  _OW_alarm=true; // set global alarm
  hex_code_lock=true;
}

/*
 * Save leng of data singnal into binary data array as 0 or 1 by leng of signal
 */
void save_as(bool val){
  byte i;
  i=len/8;
  if (val==0) data[i]= data[i] & ~mask; // save zero
  else        data[i]= data[i] | mask;  // save one
  mask=mask<<1;
  if (mask==0) {
    i++;
    mask=0x01;
  }
  len++;
}

/*
 * Recaivre IR signal
 */
void receiv() {
  byte recv_now;
  unsigned long delta_time;
  byte variance0,variance1;
  
  recv_now=digitalRead(PIN_RECV);
  new_time=micros();

  // if set use info led
  if  (sections[INFO_LED].actualValue.b==1)  {
    if (sections[LED].actualValue.b==0)  digitalWrite(PIN_LED,recv_now);
    else digitalWrite(PIN_LED,!recv_now); // if led is constant on use inverse value
  }

  // test max time of signal
  delta_time=(new_time-last_time)/10;
  if ( delta_time > 1200) {
      if (stage>=3 && len>0) write_data_to_ir_buf(); // send data
      if (stage!=0) stage=0;
      if (recv_now==recv_last) return;
   }
   if (recv_now!=recv_last) {
    switch (stage){
      case 0: // try detec new sekvence
        if (recv_now==0 && delta_time>1200) stage=1; // only if is pulldown signal and last logic level one was min duration beatwin two signals, prevent strat detection in midle of another signal
        break;
      case 1: // end first  pulse
        time_start0=delta_time/4; // start pulses can be up to 9000 us length therefore need decrease distinction to 8-bit (sumary delta_time/40 - 40us precision)
        stage=2;
        break;
      case 2: // end secound pulse
        time_start1=delta_time/4; // start pulses can be up to 9000 us length therefore need decrease distinction to 8-bit (sumary delta_time/40 - 40us precision)
        stage=3;
        break;
      case 3: // end first data pulse (save time size of pulse and code this size as bit=0 )
        time_data0=delta_time;
        //i=0;
        mask=0x01;
        len=0;
        save_as(0); //first save value is always zero
        stage=4;
        //Serial.print("stage4");
        break;
      case 4: // end next data pulse (if pulse have diferent time size save thos time and code this time size as bit=1 and go to next stage)
        variance0=time_data0 / 4;
        if ( ( delta_time >= ( time_data0 - variance0 )) && ( delta_time <= ( time_data0 + variance0) ) ) { // save zero
          time_data0 = (time_data0 + delta_time)/2; // specify the signal length using an artimetic average
          save_as(0);
        }
        else { // secound data signal lengh save him as one
          time_data1=delta_time;
          save_as(1);
          stage=5;
        }
        break;
      case 5: // code next pulses by their sizes as bit=0 or bit=1 (another size is error)
        //test signal first lengh (save as zero)
        variance0=time_data0 / 4;
        variance1=time_data1 / 4;
        if ( ( delta_time >= ( time_data0 - variance0 )) && ( delta_time <= ( time_data0 + variance0) ) ) { // save zero
          time_data0 = (time_data0 + delta_time)/2; // specify the signal length using an artimetic average
          save_as(0);
        }
        //test signal secound lengh (save as one)
        else if ( ( delta_time >= ( time_data1 - variance1)) && ( delta_time <= ( time_data1 + variance1) ) ) { // save zero
            time_data1 = (time_data1 + delta_time)/2; // specify the signal length using an artimetic average
            save_as(1);
          }
        // else wrong signal
        else {
          /*
          Serial.print(time_data0-variance0);
          Serial.print("<=");
          Serial.print(delta_time);
          Serial.print("<=");
          Serial.println(time_data0+variance0);
          Serial.print(time_data1-variance1);
          Serial.print("<=");
          Serial.print(delta_time);
          Serial.print("<=");
          Serial.println(time_data1+variance1);
          Serial.print("time_data0:");
          Serial.println(time_data0);
          Serial.print("time_data1:");
          Serial.println(time_data1);
          Serial.println("neplatna hodnota");
          */
          stage=0;
        }
        break;
    }
    recv_last=recv_now;
    last_time=new_time;
  }
}

/*
 * Check change number of transmition repeate
 */
void check_tx_repeate(){
  union value0xCD value; // space to temporary store new value

  value.u32=1;
  if (sections[TX_REPEATE].actualValue.u32 < 1){ // tx_repeate cannot be zero
    ow.write_new_value(TX_REPEATE, value); // section new value
  }

}
  
/*
 * Check status change of indication led
 */
void check_outputs(){
  // check identification led
   if (stage==0) {
    if  (sections[LED].actualValue.b==1)  {
      digitalWrite(PIN_LED, LOW);
      //on_3XKHz();
    }
    else {
      digitalWrite(PIN_LED, HIGH);
      //off_3XKHz();
    }
   }  
}

/*
 * Check save event
 */
void check_save_velues(){
  union value0xCD value; // space to temporary store new value

  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    value.b=0;
    ow.write_new_value(SAVE, value); // section Save off
    ow.save_values();
    #ifdef __DEBUG__
      Serial.println("Values all sections was benn saved.");
    #endif
  }
}


// setup
void setup() {
  // setup pins
  pinMode(PIN_RECV, INPUT);
  pinMode(PIN_LED, OUTPUT);
   
  // setup One Wire
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface for run bacground throught interrupt and load saved seting from EEPROM or on frist run set defaut values

  //setup analog comare for detect poweroff 
  //CONF_AC // (defined in suportedMCU.h)
  #ifdef __DEBUG__
    Serial.begin(115200);
  #endif
}

// main loop 
void loop() {
   // serve measrment functions
   //ow.foreground_measure(true); // if argunet is true, then will be call every meseruent functions(force_now=true), otherwise
                                // wil be call only mesurent function witch section has set alarm or before was called as part
                                // of interrupt service, bat return whithou measurment.
   // .
   // ..

   
   if (ir_buf[0]==0) { // bufer je vynulovan to umozni dalsi vysilani nebo prijem
    hex_code_lock=false; // odemkni buffer pro moznost prijmu
    receiv(); // zacni prijimat IR signaly
   }
   else { //v buferu jsou pripravene hodnoty, ted je potreba rozlisit zda-li jsou urceny pro vysilani
    if (hex_code_lock==false) { // kdby slo o hodnotu z prijmu tak by byl buffer zamkly, takze jde o vysilani
      #ifdef __DEBUG__ 
        Serial.print("transmite code:");
        Serial.println(ir_buf);
      #endif
      hex_to_data(ir_buf);
      transmit_bin_data();
      memset(ir_buf, 0, LEN_IR_BUF); // po odeslani se bufer vymaze
    }
   }
   check_tx_repeate();
   check_outputs();
   check_save_velues();
   
   // do what you want (but do not disable interrupt, if do known what you do)
   // ..
   // .   
 }
