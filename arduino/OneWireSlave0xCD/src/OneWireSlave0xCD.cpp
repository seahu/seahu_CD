#include "OneWireSlave0xCD.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <string.h>
#include <stdio.h> // na testovani pak smazat (pro fci sprintf)
#include "eeprom_lib.h"
#include "comptime.h"

// global variables
volatile uint8_t actual_section_=0; // number of actual selected section
section *p_sections_; // defacto define pointer into array of sections (in ini_ow_uni change this pointer to user defined array)
uint8_t number_of_sections_; // number of sections
//char *p_device_description_ PROGMEM; // pointer into device description
char *p_device_description_; // pointer into device description

    uint8_t *p_to_buf_;
    uint8_t len_;
    uint16_t crc_; // for save actual fluent calculated crc
    uint16_t crc_read_; // space for read crc from master
    bool confirm_;
    void (*complete_)();

// GENERATION OF UNIQUE ROM CODE
uint8_t rom_[8] = { FAM, SERIAL_NUMBER, 0x00 };
//   will be calculated in begin:-------^^^^

uint8_t buf_ow_uni_[64]; // buffer for temporarry vaules like as descriptin, note, etc, ... (buffer only for librarry in this file) (PS: 64B becouse section note has size 64B)

OneWireSlave0xCD ow__;  // global variable to class OneWireSlave0xCD (ow__ because ow_ already used in OneWireSlaveExtend)

// functions - private declaration
void ow_main_0xDD(uint8_t cmd); // main function, this funcion is call from service of interruption

// declaration shared OW command function for this universal measurment
void cmd_0x44(); // measure command (measure selected section)
void cmd_0xF1(); // read factory description device (64Bytes)
void cmd_0xF2(); // read number of sections (8bit)
void cmd_0xF3(); // read from CONTROL register (8bit) - control register selected section
void cmd_0xF5(); // read from ACTUAL_VALUE register (1-4Byte depends on setting in control register)
void cmd_0xF7(); // read from factory DESCRIPTION of section (64Bytes)
void cmd_0xF8(); // read from user NOTE of section (32Bytes)
void cmd_0xFA(); // read from MIN_ALARM_VALUE register (1-4Bytes depends on setting in control register)
void cmd_0xFB(); // read from MAX_ALARM_VALUE register (1-4Bytes depends on setting in control register)
void cmd_0x53(); // write into CONTROL register (8bit) - control register selected section
void cmd_0x55(); // write into ACTUAL_VALUE (1-4Bytes depends on setting in control register)
void cmd_0x58(); // write into NOTE register (32Bytes)
void cmd_0x5A(); // write into MIN_ALARM_VALUE (1-4 Byte depends on setting in control register)
void cmd_0x5B(); // write into MAX_ALARM_VALUE (1-4 Byte depends on setting in control register)

//void OW_send_bytes( uint8_t *buf, uint8_t len, void (*complete)() ); // send x bytes with crc16 on end to master (crc16 is fluently calculated started with cmd code and ending by last sended byte)
//void OW_get_bytes( uint8_t *buf, uint8_t len, void (*complete)(), bool confirm); // get x bytes with with checking crc16 from master, if all is OK and confirm=true will be sent confirm to master about successfully getting data 



void OneWireSlave0xCD::save_default_values(){
  uint8_t i;
  uint8_t j;
  EEPROM_write_byte(EE_FIRST_RUN, 0xAA); // 0xAA is symptom that default seting was done
  for (i=0; i<number_of_sections_; i++){
    for (j=0; j<(EE_SIZE_OF_SECTION); j++){  // zero all sectin
      EEPROM_write_byte((EE_START_POINT)+(i)*(EE_SIZE_OF_SECTION)+(EE_CONTROL)+(j), 0); // 0 is default value for control byte, actualValue, minAlarmValue, maxAlarmValue and user note
    }
  }
}

void OneWireSlave0xCD::save_values(){
  uint8_t i;
  uint8_t j;
  for (i=0; i<number_of_sections_; i++){
    EEPROM_write_byte((EE_START_POINT)+(i)*(EE_SIZE_OF_SECTION)+(EE_CONTROL), ((p_sections_+i)->control & 0b11000000) );       // control register save only writale flags
    for (j=0; j<EE_SIZE_VALUE; j++){
      EEPROM_write_byte((EE_START_POINT)+(i)*(EE_SIZE_OF_SECTION)+(EE_ACTUAL_VALUE)+(j), (p_sections_+i)->actualValue.u8t[j]);       // actual value
      // other values as min, max alarm value and user note not necessary save because are directly stored only in EEPROM
    }
  }
}

void OneWireSlave0xCD::load_values(){
  uint8_t i;
  uint8_t j;
  for (i=0; i<number_of_sections_; i++){
    // control register
    (p_sections_+i)->control=(uint8_t)((EEPROM.read((EE_START_POINT)+(i)*(EE_SIZE_OF_SECTION)+(EE_CONTROL)) & 0b11000000) | (p_sections_+i)->control); // load only writale flags
    for (j=0; j<EE_SIZE_VALUE; j++){
      (p_sections_+i)->actualValue.u8t[j]=EEPROM.read((EE_START_POINT)+(i)*(EE_SIZE_OF_SECTION)+(EE_ACTUAL_VALUE)+(j));            // actual value
      // other values as min, max alarm value and user note not necessary save because are directly stored only in EEPROM
    }
  }
}


void OneWireSlave0xCD::ini(uint8_t number_of_sections, section userSections[], const char* device_description PROGMEM) {
  // set global variables
  p_sections_=userSections;
  number_of_sections_=number_of_sections;
  p_device_description_=device_description;
  
  // load or save default values
  if (EEPROM.read(EE_FIRST_RUN) != 0xAA) {
    ow__.save_default_values();
  }
  ow__.load_values();

  // ini OW
  ow__.begin(&ow_main_0xDD, rom_); 

  // set calculating CRC16 in fluent
  _OW_crc_mode=CRC_16;
}


//------------------------ AUXILIARY ONE WIRE FUNCTION ----------------------------
//------------------------------------------------------------------------

/*
 * send X-bytes to master
 * 
 * PS: crc16 start calculate ealier witch commant byte (_OW_crc_mode=CRC_16 must be set beforehand)
 */
void OW_send_bytes_send_crc();
void OW_send_bytes( uint8_t *buf, uint8_t len, void (*complete)() ) {
  complete_=complete;
  len_=len;
  p_to_buf_=buf;
  ow__.write(p_to_buf_, len, &OW_send_bytes_send_crc);
} //    ||
  //    \/
void OW_send_bytes_send_crc() {
  crc_=~_OW_crc16;
  ow__.write((uint8_t *) &crc_, 2, complete_); // continue by complete function after send crc
}


/*
 * get X-bytes from master
 * 
 * PS: crc16 start calculate ealier witch commant byte (_OW_crc_mode=CRC_16 must be set beforehand)
 */
void OW_get_bytes_read_crc();
void OW_get_bytes_send_check();
void OW_get_bytes( uint8_t *buf, uint8_t len, void (*complete)(), bool confirm) {
  complete_=complete;
  len_=len;
  p_to_buf_=buf;
  confirm_=confirm;
   ow__.read(p_to_buf_, len, &OW_get_bytes_read_crc);
} //    ||
  //    \/
void OW_get_bytes_read_crc() {
  crc_=~_OW_crc16;
  ow__.read((uint8_t*)&crc_read_, 2, &OW_get_bytes_send_check);
} //    ||
  //    \/
void OW_get_bytes_send_check() {
  if ( crc_read_ != crc_ ){
    ow__.reset();
    return;
  }
  if (confirm_==false) complete_(); // continue by complete function withou confirm
  else {
    crc_read_=0xAA; // use crc_read_ as buffer for writing data
    ow__.write((uint8_t*)&crc_read_, 1, complete_); // continue by complete function after confirm
  }
} 



//------------------------ ONE WIRE FUNCTION ----------------------------
//------------------------------------------------------------------------

// auxiliary function for check end of _OW_alarm
void check_off_ow_alarm(){
  if ( _OW_alarm==false) return; 
  for ( uint8_t i=0; i<number_of_sections_; i++ ){ // for all sections
    if (  C_ALARM_STATUS & (p_sections_+i)->control ) return; // alarm exist
  }
  _OW_alarm=false;
}

//0x44 - measure command (measure selected section)
void cmd_0x44__finish();
void cmd_0x44(){
  OW_get_bytes( &actual_section_, 1, &cmd_0x44__finish, true);
} //    ||
  //    \/
void cmd_0x44__finish(){
  if (actual_section_==0xFF){
    for (uint8_t i; i<number_of_sections_; i++){
      if ( (p_sections_+i)->measreCmd!=NULL) { // if was set measure comnand, than do it
        uint8_t control= (p_sections_+i)->control;
        (p_sections_+i)->control=control & 0b11111110; // write 0 into read flag - mean not ready for read date ecause measure now
        (p_sections_+i)->measreCmd(false); // run custom measure function
        (p_sections_+i)->control=control;
      }
    }
  }
  else {
    if (actual_section_ >= number_of_sections_) actual_section_=number_of_sections_-1;
    if ( (p_sections_+actual_section_)->measreCmd!=NULL) { // if was set measure comnand, than do it
      uint8_t control= (p_sections_+actual_section_)->control;
      (p_sections_+actual_section_)->control=control & 0b11111110; // write 0 into read flag - mean not ready for read date ecause measure now
      (p_sections_+actual_section_)->measreCmd(false); // run custom measure function
      (p_sections_+actual_section_)->control=control;
    }
  }
  ow__.waitToCmd();
}

//0xF1 - read factory description device (64Bytes)
void cmd_0xF1(){
  for (uint8_t i=0; i<64; i++){
    buf_ow_uni_[i]=pgm_read_byte_near( p_device_description_ + i);
  }
  OW_send_bytes(buf_ow_uni_, 64, &ow__.waitToCmd);
}

//0xF2 - read number of sections (8bit)
void cmd_0xF2(){
  OW_send_bytes( &number_of_sections_, 1, &ow__.waitToCmd );
}

//0xF3 - read from CONTROL register (8bit) - control register selected section
void cmd_0xF3__finish();
void cmd_0xF3(){
  ow__.read(&actual_section_, 1, &cmd_0xF3__finish);
} //    ||
  //    \/
void cmd_0xF3__finish(){
  OW_send_bytes(&(p_sections_+actual_section_)->control, 1, &ow__.waitToCmd);
}

//0x53 - write into CONTROL register (8bit)
void cmd_0x53__finish();
void cmd_0x53(){
  OW_get_bytes( buf_ow_uni_, 2, &cmd_0x53__finish, true);
} //    ||
  //    \/
void cmd_0x53__finish(){
  actual_section_=buf_ow_uni_[0];
  uint8_t newControl=buf_ow_uni_[1] & 0b11000000; // only bit who set min, max alarm is allowed to write
  (p_sections_+actual_section_)->control=( (p_sections_+actual_section_)->control & 0b00111111) | newControl;
  ow__.waitToCmd();
}

//0xF5 - read from ACTUAL_VALUE
void cmd_0xF5__send_value();
void cmd_0xF5__finish();
void cmd_0xF5(){
  ow__.read(&actual_section_, 1, &cmd_0xF5__send_value);
} //    ||
  //    \/
void cmd_0xF5__send_value(){
  if (actual_section_>=number_of_sections_) actual_section_=number_of_sections_-1;
  LOCK
  memcpy(buf_ow_uni_, &((p_sections_+actual_section_)->actualValue), 4);
  UNLOCK
  OW_send_bytes(buf_ow_uni_, 4, &cmd_0xF5__finish);
} //    ||
  //    \/
void cmd_0xF5__finish(){
  (p_sections_+actual_section_)->control &=~C_ALARM_STATUS; //reset section alarm status flag
  check_off_ow_alarm();
  ow__.waitToCmd();
}

//0xFA - read from MIN_ALARM_VALUE
void cmd_0xFA__finish();
void cmd_0xFA(){
  ow__.read(&actual_section_, 1, &cmd_0xFA__finish);
} //    ||
  //    \/
void cmd_0xFA__finish(){
  EEcmp(buf_ow_uni_,(EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_MIN_ALARM_VALUE),4);
  OW_send_bytes(buf_ow_uni_, 4, &ow__.waitToCmd);
}

//0xFB - read from MAX_ALARM_VALUE
void cmd_0xFB__finish();
void cmd_0xFB(){
  ow__.read(&actual_section_, 1, &cmd_0xFB__finish);
} //    ||
  //    \/
void cmd_0xFB__finish(){
  EEcmp(buf_ow_uni_,(EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_MAX_ALARM_VALUE),4);
  OW_send_bytes(buf_ow_uni_, 4, &ow__.waitToCmd);
}

//0xF7 - read from DESCRIPTION register (64Byte)
void cmd_0xF7__finish();
void cmd_0xF7(){
  ow__.read(&actual_section_, 1, &cmd_0xF7__finish);
} //    ||
  //    \/
void cmd_0xF7__finish(){
  for (uint8_t i=0; i<64; i++){
    buf_ow_uni_[i]=pgm_read_byte_near( (p_sections_+actual_section_)->shortDescription + i);
  }
  OW_send_bytes(buf_ow_uni_, 64, &ow__.waitToCmd);
}

//0xF8 - read from NOTE register (32Bytes)
void cmd_0xF8__finish();
void cmd_0xF8(){
  ow__.read(&actual_section_, 1, &cmd_0xF8__finish);
}
void cmd_0xF8__finish(){
  if ( (p_sections_+actual_section_)->memory == NULL ){
    EEcmp(buf_ow_uni_, (EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_USER_NOTE),32);
    //sprintf(buf_ow_uni_,"pozice:%d",(EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_USER_NOTE));
    OW_send_bytes(buf_ow_uni_, 32, &ow__.waitToCmd);
  }
  else {
    //buf_ow_uni_[0]='A';
    //buf_ow_uni_[1]=0;
    //OW_send_bytes(buf_ow_uni_, 32, &ow__.waitToCmd);
    OW_send_bytes((p_sections_+actual_section_)->memory, 32, &ow__.waitToCmd);
  }
}

//0x55 - write into ACTUAL_VALUE
void cmd_0x55__finish();
void cmd_0x55(){
  OW_get_bytes( buf_ow_uni_, 5, &cmd_0x55__finish, true);
}
void cmd_0x55__finish(){
  actual_section_=buf_ow_uni_[0];
  if ( C_WRITE & (p_sections_+actual_section_)->control ) { // is writeable
    memcpy(&(p_sections_+actual_section_)->actualValue ,&buf_ow_uni_[1], 4);
  }
  ow__.waitToCmd();
}

//0x5A - write into MIN_ALARM_VALUE
void cmd_0x5A__finish();
void cmd_0x5A(){
  OW_get_bytes( buf_ow_uni_, 5, &cmd_0x5A__finish, false);
} //    ||
  //    \/
void cmd_0x5A__finish(){
    //An EEPROM write 4B takes approximately 15ms (1B=3.3 ms) to complete.
    actual_section_=buf_ow_uni_[0];
    cmpEE(&buf_ow_uni_[1], (EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_MIN_ALARM_VALUE), 4);
    // send confirm
    buf_ow_uni_[0]=0xAA;
    ow__.write(buf_ow_uni_, 1, &ow__.waitToCmd);
}

//0x5B - write into MAX_ALARM_VALUE
void cmd_0x5B__finish();
void cmd_0x5B(){
  OW_get_bytes( buf_ow_uni_, 5, &cmd_0x5B__finish, false);
} //    ||
  //    \/
void cmd_0x5B__finish(){
    //An EEPROM write 4B takes approximately 15ms (1B=3.3 ms) to complete.
    actual_section_=buf_ow_uni_[0];
    cmpEE(&buf_ow_uni_[1], (EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_MAX_ALARM_VALUE), 4);
    // send confirm
    buf_ow_uni_[0]=0xAA;
    ow__.write(buf_ow_uni_, 1, &ow__.waitToCmd);
}

//0x58 - write into NOTE register (32Bytes)
void cmd_0x58_finish();
void cmd_0x58(){
  OW_get_bytes(buf_ow_uni_, 33, &cmd_0x58_finish, false);
} //    ||
  //    \/
void cmd_0x58_finish(){
    // !!! bufer musi byt globalni promenna ne lokalni
    //An EEPROM write 32B takes approximately 125ms (1B=3.3 ms) to complete.
    actual_section_=buf_ow_uni_[0];
    if ( (p_sections_+actual_section_)->memory == NULL ){
      cmpEE(&buf_ow_uni_[1], (EE_START_POINT)+(actual_section_)*(EE_SIZE_OF_SECTION)+(EE_USER_NOTE), 32);
    }
    else {
      memcpy((p_sections_+actual_section_)->memory, &buf_ow_uni_[1], 32);
    }
    // send confirm
    buf_ow_uni_[0]=0xAA;
    ow__.write(buf_ow_uni_, 1, &ow__.waitToCmd);
}

//------------------------ RUN MEASUERD FUNCTIONS ON FOREGROUND (main loop)----------------------------
//-----------------------------------------------------------------------------------------------------


/**
 * Write new value into global array of sections.
 * Wait to end lock, to prevent fail on One wire comunication (may be lock during reading or writing actual valu by One Wire).
 * Do test min, max and test of change value if this test is set for this section.
 * And clear mearumnet flag.
 * 
 * @param section Is number section to which one will be write new value
 * @param mewValue Is union type varibales whre is stored new value
 * @return void.
 */ 
void OneWireSlave0xCD::write_new_value(uint8_t section, union value0xCD mewValue){
  while ( (p_sections_+section)->lock && _OW_lock )  {}; // wait to unlock (section is lock only if is lock both section lock and  global _OW_lock)
  union value0xCD minValue;
  union value0xCD maxValue;
  uint8_t control=(p_sections_+section)->control;
  bool alarm=false;
  EEcmp(minValue.u8t, section*EE_SIZE_OF_SECTION+EE_MIN_ALARM_VALUE,4); // get minValue from EEPROM
  EEcmp(maxValue.u8t, section*EE_SIZE_OF_SECTION+EE_MAX_ALARM_VALUE,4); // get maxValue from EEPROM
  switch( control & 0b00011000 ) { // filtr from contol byte of section pnly value types bit 3-5
    case C_BOOL:   // 0b00001000
      if ( control & 0b01000000 ) alarm=alarm || ( mewValue.u32<=minValue.u32 );  // <
      if ( control & 0b10000000 ) alarm=alarm || ( mewValue.u32>=maxValue.u32 );  // >
      (p_sections_+section)->actualValue.u32=mewValue.u32; // save new value
      break;
    case C_U32BIT:  // 0b00010000
      if ( control & 0b01000000 ) alarm=alarm || ( mewValue.u32<=minValue.u32 );
      if ( control & 0b10000000 ) alarm=alarm || ( mewValue.u32>=maxValue.u32 );
      (p_sections_+section)->actualValue.u32=mewValue.u32;
      break;
    case C_32BIT:   // 0b00011000
      if ( control & 0b01000000 ) alarm=alarm || ( mewValue.i32<=minValue.i32 );
      if ( control & 0b10000000 ) alarm=alarm || ( mewValue.i32>=maxValue.i32 );
      (p_sections_+section)->actualValue.i32=mewValue.i32;
      break;
  }
  (p_sections_+section)->control|=C_READ; // clear flag for new measurment
  (p_sections_+section)->control|=C_ALARM_STATUS; // set section alarm
  //(p_sections_+section)->alarm=alarm; // set section alarm
  _OW_alarm|=alarm; // set global alarm
}


/**
 * Run one cycle measuring on foreground (from main loop).
 * Individually mesurment functions who has defined in global array of sections wil be run with parameter=1 etc. fun(1)
 * This means that measurement time does not matter.
 *
 * @param all  If is true run all measurments, otherwise run only measurment who has set alarm or measurment flag. 
 *             (measurment flag = clear C_READ bin in control byte of section)
 * @return void.
 */ 
void OneWireSlave0xCD::foreground_measure(bool all)
{
  for ( uint8_t i=0; i<number_of_sections_; i++ ){ // for all sections
    if ( (p_sections_+i)->measreCmd==NULL) continue; // section have not measure function
    if (all==false) { // not force for all sections
      if ( (p_sections_+i)->control & ~C_READ ) { // is not clear C_READ (section is not flag to new measurment)
        if (( (p_sections_+i)->control & 0b11000000)==0) continue; // section have not set alarm
      }
    }
    // sectin has set check alarm of set flag for new measurment
    uint8_t control=(p_sections_+i)->control;
    (p_sections_+i)->control&=~C_READ; // write 0 into read flag - mean not ready for read date ecause measure now
    (p_sections_+i)->measreCmd(true); // un custom measure function param= true means that does not matter on measurment time.
  }
}



  

//------------------------ MAIN FUNCTION  ---------------------------------------------------
//----------(this funcion is call from service of interruption)  ----------------------------
//-------------------------------------------------------------------------------------------

void ow_main_0xDD(uint8_t cmd)
{
  switch(cmd){
    case 0x44:
      cmd_0x44(); // measure command (measure selected section)
      break;
    case 0xF1:
      cmd_0xF1(); // read factory description device (64Bytes)
      break;
    case 0xF2:
      cmd_0xF2(); // read number of sections (8bit)
      break;
    case 0xF3:
      cmd_0xF3(); // read from CONTROL register (8bit) - control register selected section
      break;
    case 0xF5:
      cmd_0xF5(); // read from ACTUAL_VALUE register (1-4Byte depends on setting in control register)
      break;
    case 0xF7:
      cmd_0xF7(); // read from DESCRIPTION register (64Byte)
      break;
    case 0xF8:
      cmd_0xF8(); // read from NOTE register (32Bytes)
      break;
    case 0xFA:
      cmd_0xFA(); // read from MIN_ALARM_VALUE register (1-4Bytes depends on setting in control register)
      break;
    case 0xFB:
      cmd_0xFB(); // read from MAX_ALARM_VALUE register (1-4Bytes depends on setting in control register)
      break;
    case 0x53:
      cmd_0x53(); // write into CONTROL register (8bit) - control register selected section
      break;
    case 0x55:
      cmd_0x55(); // write into ACTUAL_VALUE (1-4Bytes depends on setting in control register)
      break;
    case 0x58:
      cmd_0x58(); // write into NOTE register (32Bytes)
      break;
    case 0x5A:
      cmd_0x5A(); // write into MIN_ALARM_VALUE (1-4 Byte depends on setting in control register)
      break;
    case 0x5B:
      cmd_0x5B(); // write into MAX_ALARM_VALUE (1-4 Byte depends on setting in control register)
      break;
  }
}
