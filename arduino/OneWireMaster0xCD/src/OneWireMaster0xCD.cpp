/*
  OneWireMaster0xCD - Library for easy use 1-Wire multi sensor/actor devices with family code 0xCD in arduino projects
  Created by Ondrej Lycka, 2020.
  Released into the public domain.
*/

#include "Arduino.h"
#include "OneWireMaster0xCD.h"


/*
 * PUBLIC FUNCTIONS
 * ----------------
 */


/*  
 *   INI
 */
//OneWireMaster0xCD::OneWireMaster0xCD(int pin): ds(pin)
//{
//  
//}
//OneWireMaster0xCD::OneWireMaster0xCD(OneWire *ow, int pin): ds(pin)
OneWireMaster0xCD::OneWireMaster0xCD(OneWire *ow)
{
  p_ow=ow;
}



void OneWireMaster0xCD::setOw(OneWire& ow){
  //ds=ow;
}

void OneWireMaster0xCD::setAddr(uint8_t addrres[]){
  memcpy(addr,addrres,8);
}

void OneWireMaster0xCD::setSection(uint8_t section_number){
  #ifdef OneWireMaster0xCDDebug
  Serial.print("Set actual section: ");
  Serial.println(section_number);
  #endif
  section=section_number;
}

/*
 * GET NUMBER OF SECTIONS
 */
uint8_t OneWireMaster0xCD::get_number_of_sections(){
  uint8_t number;
  if (read_uni(0xF2, NULL , &number, 1, 0)){
      #ifdef OneWireMaster0xCDDebug
      Serial.print("Number of sections: ");
      Serial.println(number);
      #endif
  }
  else {
      #ifdef OneWireMaster0xCDDebug
      Serial.print("BAD get number of sections: ");
      #endif
      number=0;
  }
  return number;
}


/*
 * GET STATUS BYTE
 * result is returned and stored into global value - controlByte
 * return 0 means error
 */
uint8_t OneWireMaster0xCD::get_status_byte(){
  uint8_t B;
  if (read_uni(0xF3, &section , &B, 1, 0)){
      controlByte=B; //save new value only if no comuncation error
      #ifdef OneWireMaster0xCDDebug
      Serial.print("Status Byte: ");
      Serial.println(controlByte);
      #endif
      return controlByte;
  }
  else {
      #ifdef OneWireMaster0xCDDebug
      Serial.print("BAD get number controlByte: ");
      #endif
      return 0;
  }
}

/*
 * PRINT STATUS BYTE
 */
void OneWireMaster0xCD::print_status_byte(){
  if (controlByte&0b00000001)  Serial.println("Actual value is readable.");
  else                        Serial.println("Actual value is not readable.");
  if (controlByte&0b00000010)  Serial.println("Actual value is writeable.");
  else                        Serial.println("Actual value is not writeable.");
  if (controlByte&0b00000100)  Serial.println("Support alarm.");
  else                        Serial.println("Not support alarm.");
  if (controlByte&0b00001000)  Serial.println("!! ALARM !!");
  else                        Serial.println("No alarm now.");
  if ((uint8_t)(((uint8_t)controlByte&0b00110000)>>4)==0) Serial.println("Type 32B memory.");
  if ((uint8_t)(((uint8_t)controlByte&0b00110000)>>4)==1) Serial.println("Type boll.");
  if ((uint8_t)(((uint8_t)controlByte&0b00110000)>>4)==2) Serial.println("Type unsigned 32bit.");
  if ((uint8_t)(((uint8_t)controlByte&0b00110000)>>4)==3) Serial.println("Type signed 32bit.");
  if (controlByte&0b01000000)  Serial.println("Min. value alarm is active.");
  else                        Serial.println("Min. value alarm is not active.");
  if (controlByte&0b10000000)  Serial.println("Max. value alarm is active.");
  else                        Serial.println("Max. value alarm is not active.");
}

/*
 * IS ready for reead new value?
 */
bool OneWireMaster0xCD::is_ready(){
  if (get_status_byte()==false) return false;
  return controlByte&0b00000001;
}

/*
 * MEASURE REQUIREST FOR ACTUAL SECTION
 */
void OneWireMaster0xCD::start_measure(){
  if (  write_uni(0x44, &section, NULL, 0, 0) ){
        #ifdef OneWireMaster0xCDDebug
	Serial.println("Send new measure requirest.");
        #endif
  }
  else {
        #ifdef OneWireMaster0xCDDebug
	Serial.println("Can't send measure requirest.");
        #endif
  }
}

/*
 * MEASURE REQUIREST FOR ALL SECTIONS
 */
void OneWireMaster0xCD::start_measure_all(){
  const uint8_t sections=0xFF;
  if (  write_uni(0x44, &sections, NULL, 0, 0) ){
        #ifdef OneWireMaster0xCDDebug
	Serial.println("Send new measure requirest for all sections.");
        #endif
  }
  else {
        #ifdef OneWireMaster0xCDDebug
	Serial.println("Can't send measure requirest for all sections.");
        #endif
  }
}


/*
 * GET NEW MEASURED VALUE
 */
bool OneWireMaster0xCD::get_measured_value(val *value){
  for (uint8_t i=0; i<100; i++){
    if (is_ready()) return get_value(value, 0xF5);
    else delayMicroseconds(5000);
    #ifdef OneWireMaster0xCDDebug
    Serial.println("Wait for ready actual value");
    #endif
  }
  #ifdef OneWireMaster0xCDDebug
  Serial.println("To long wait for ready actual value");
  #endif
  return false;
}

/*
 * get ACTUAL_VALUE from actual section
 */
bool OneWireMaster0xCD::get_actual_value(val *value){
  return get_value(value, 0xF5);
}
bool OneWireMaster0xCD::get_actual_value(val *value, uint8_t section){
  setSection(section);
  return get_value(value, 0xF5);
}
/*
 * set ACTUAL_VALUE from actual section
 */
bool OneWireMaster0xCD::set_actual_value(val *value){
  return set_value(value,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(val *value, uint8_t section){
  setSection(section);
  return set_value(value,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(uint32_t value, uint8_t section){
  val newvalue;
  newvalue.u32bit=value;
  setSection(section);
  return set_value(&newvalue,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(int32_t value, uint8_t section){
  val newvalue;
  newvalue.s32bit=value;
  setSection(section);
  return set_value(&newvalue,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(uint16_t value, uint8_t section){
  val newvalue;
  newvalue.u32bit=(uint32_t)value;
  setSection(section);
  return set_value(&newvalue,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(int16_t value, uint8_t section){
  val newvalue;
  newvalue.s32bit=(int32_t)value;
  setSection(section);
  return set_value(&newvalue,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(uint8_t value, uint8_t section){
  val newvalue;
  newvalue.u32bit=(uint32_t)value;
  setSection(section);
  return set_value(&newvalue,0x55, 0);
}
bool OneWireMaster0xCD::set_actual_value(int8_t value, uint8_t section){
  val newvalue;
  newvalue.s32bit=(int32_t)value;
  setSection(section);
  return set_value(&newvalue,0x55, 0);
}


/*
 * print value by type defined into control register for debug purpouse
 */
void OneWireMaster0xCD::print_value(val *value, char head[]){
  Serial.print(head);
  switch( (uint8_t)(((uint8_t)controlByte&0b00110000)>>4) ){
    case 0: //memory value allways 0x00000000
      Serial.println(value->u32bit);
      return;
    case 1: // bool
      Serial.println(value->Bool);
      return;
    case 2: // u32bit
      Serial.println(value->u32bit);
      return;
    case 3: // s32bit
      Serial.println(value->s32bit);
      return;
  }
}

/*
 * get/set MIN_ALARM_VALUE from actual section
 */
bool OneWireMaster0xCD::get_min_alarm_value(val *value){
  return get_value(value, 0xFA);
}
bool OneWireMaster0xCD::set_min_alarm_value(val *value){
  return set_value(value,0x5A,15);
}

/*
 * get/set MAX_ALARM_VALUE from actual section
 */
bool OneWireMaster0xCD::get_max_alarm_value(val *value){
  return get_value(value, 0xFB);
}
bool OneWireMaster0xCD::set_max_alarm_value(val *value){
  return set_value(value,0x5B,15);
}

/*
 * get/set secttion user note (!len of buf must be min 32 !)
 */
bool OneWireMaster0xCD::get_note(char buf[]){
  if (read_uni(0xF8, &section , buf, 32, 500)==false) return false;
  #ifdef OneWireMaster0xCDDebug
  Serial.println(buf);
  #endif
  return true;
}
bool OneWireMaster0xCD::get_note(char buf[], uint8_t section){
  setSection(section);
  return get_note(buf);
}

bool OneWireMaster0xCD::set_note(char buf[]){
  if (write_uni(0x58, &section, buf, 32, 1000)==false) {
	Serial.println("zapis se nepovedl");
	return false;
  }
  return true;
}
bool OneWireMaster0xCD::set_note(char buf[], uint8_t section){
  setSection(section);
  return set_note(buf);
}

/*
 * get section description (!len of buf must be min 64 !)
 */
bool OneWireMaster0xCD::get_description(char buf[]){
  if (read_uni(0xF7, &section , buf, 64, 500)==false) return false;
  #ifdef OneWireMaster0xCDDebug
  Serial.println(buf);
  #endif
  return true;
}
bool OneWireMaster0xCD::get_description(char buf[], uint8_t section){
  setSection(section);
  return get_description(buf);
}

/*
 * get device description (!len of buf must be min 64 !)
 */
bool OneWireMaster0xCD::get_device_description(char buf[]){
  if (read_uni(0xF1, NULL , buf, 64, 500)==false) return false;
  #ifdef OneWireMaster0xCDDebug
  Serial.println(buf);
  #endif
  return true;
}



/*
 * PRIVATE FUNCTIONS
 * -----------------
 */


/*
* READ sekvencion over 1-Wire for device with family code 0xCD
*/
bool OneWireMaster0xCD::read_uni(uint8_t cmd, uint8_t *section , void *buf, uint8_t len, uint8_t Delay){
  uint16_t crc_read;
  uint16_t crc_calc;
  uint16_t pokus=0xBBAA;
  #ifdef OneWireMaster0xCDDebug
  Serial.print("CMD:");
  Serial.print(cmd,HEX);
  Serial.print(" Section:");
  if (section!=NULL) Serial.print(*section);
  else Serial.print("--");
  Serial.print(" Read BYTES len:");
  Serial.println(len);
  #endif
  p_ow->reset();
  p_ow->select(addr);
  // write
  p_ow->write(cmd); // write cmd
  if (section!=NULL) p_ow->write(*section); // write section number
  delayMicroseconds(Delay); // wait to device prepare data (read from EEPROM etc.)
  // read
  p_ow->read_bytes((uint8_t*)buf, len);
  p_ow->read_bytes((uint8_t*)&crc_read, 2);
  // crc
  crc_calc=p_ow->crc16(&cmd,1,0);
  if (section!=NULL)crc_calc=p_ow->crc16(section,1,crc_calc);
  crc_calc=p_ow->crc16(buf,len,crc_calc);
  crc_calc=~crc_calc;
  if (crc_read!=crc_calc) {
    #ifdef OneWireMaster0xCDDebug
    //memcpy(&crc_read,&pokus,2);
    //memcpy((char*)&crc_read,&pokus,2);
    Serial.println("Read BYTES: BAD CRC");
    Serial.print("CRC_calc: ");
    Serial.println(crc_calc, HEX);
    Serial.print("CRC_read: ");
    Serial.println(crc_read, HEX);
    Serial.print("Data: ");
    for (uint8_t i=0; i<len; i++){
       Serial.print(*((uint8_t*)buf+i), HEX);
    }
    Serial.println("");
    #endif
    return false;
  }
  return true;
}

/*
* WRITE sekvencion over 1-Wire for device with family code 0xCD
*/
bool OneWireMaster0xCD::write_uni(uint8_t cmd, uint8_t *section, void *buf, uint8_t len, uint8_t Delay){
  uint16_t crc_calc;
  uint8_t confirm;
  #ifdef OneWireMaster0xCDDebug
  Serial.print("CMD:");
  Serial.print(cmd,HEX);
  Serial.print(" Section:");
  if (section!=NULL) Serial.print(*section);
  else Serial.print("--");
  Serial.print(" Write BYTES len:");
  Serial.println(len);
  #endif
  p_ow->reset();
  p_ow->select(addr);
  p_ow->write(cmd); // write cmd
  p_ow->write(*section); // write section number
  p_ow->write_bytes(buf, len); // write data
  // calc crt
  crc_calc=p_ow->crc16(&cmd,1,0);
  crc_calc=p_ow->crc16(section,1,crc_calc);
  crc_calc=p_ow->crc16(buf,len,crc_calc);
  crc_calc=~crc_calc;
  p_ow->write_bytes((uint8_t*)&crc_calc, 2); // write crc
  delayMicroseconds(Delay); // wait to device save date (write to EEPROM etc.)
  confirm=p_ow->read();
  if (confirm!=0xAA) {
    #ifdef OneWireMaster0xCDDebug
    Serial.print ("BAT write n-bytes: ");
    Serial.println(len);
    #endif
    return false;
  }
  return true;
}



/*
 * GET/SET  VALUE over 1-Wire
 * get/set into/from value (used for actual, min and max value)
 * cmd must be code for command read/write actualValue, min_alarm_value or max_alarm_value
 */
bool OneWireMaster0xCD::get_value(val *value, uint8_t cmd){
  uint8_t buf[4]={0,0,0,0};
  if (read_uni(cmd, &section , buf, 4, 0)==false) return false;
  memcpy(value,buf,4);
  #ifdef OneWireMaster0xCDDebug
  	print_value(value, "Get value: ");
  #endif
  return true;
}

bool OneWireMaster0xCD::set_value(val *value, uint8_t cmd, uint8_t Delay){
  if (write_uni( cmd, &section, value, 4, Delay)==false) return false;
  #ifdef OneWireMaster0xCDDebug
  print_value(value, "Set value: ");
  #endif
  return true;
}

