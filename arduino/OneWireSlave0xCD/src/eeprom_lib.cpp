#include "Arduino.h"
#include "eeprom_lib.h"
#include <EEPROM.h>
/*
   normalne bych pouzil fci EEPROM.update(adr,val) , jenze to by musela fungovat, mne zapisovala nahodne hodnoty
   tak jsem si naprogramoval vlastni fci pro zapis do EEPROM
*/


/*
 * write byte to eeprom
 */
void EEPROM_write_byte(unsigned int adr, uint8_t val) {
  //Serial.print("EE WRITE: ");
  //Serial.println(val);
  //An EEPROM write 1B takes 3.3 ms to complete. The EEPROM memory has a specified life of 100,000 write/erase cycles, so you may need to be careful about how often you write to it.
  if (EEPROM.read(adr) != val) {
    #ifdef OneWireSlaveDebug
    Serial.print("EEPROM: ");
    Serial.print(adr);
    Serial.print(", ");
    Serial.println(val);
    #endif
    EEPROM.write(adr, val);
  }
}


/*
 * write int (16bit) to eeprom
 */
void EEPROM_write_int(unsigned int adr, unsigned int val) {
  uint8_t byte_L = 0;
  uint8_t byte_H = 0;
  //Serial.print("SEND: ");
  //Serial.println(val);
  byte_L = val & 0x00FF;
  byte_H = (val & 0xFF00) / 256;
  EEPROM_write_byte(adr,byte_L);
  EEPROM_write_byte(adr + 1,byte_H);
}

/*
 * function for copy data from EEPROM to RAM buffer 
 */
void EEcmp(uint8_t buf[], int EEaddress, uint8_t len){
  for (int i=0; i<len; i++){
    buf[i]=EEPROM.read(EEaddress+i);
  }
}

/*
 * function for copy data from RAM buffer to EEPROM 
 */
void cmpEE(uint8_t buf[], int EEaddress, uint8_t len){
  for (int i=0; i<len; i++){
    EEPROM_write_byte(EEaddress+i, buf[i]);
  }
}
