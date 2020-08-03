#ifndef EepromLib_h
  #define EepromLib_h
/*
 * because standart EEPROM.update(adr,val) has bug I programer own eeprom write function
 */


//#define OneWireSlaveDebug

/*
 * write byte to eeprom
 */
void EEPROM_write_byte(unsigned int adr, uint8_t val);


/*
 * write int (16bit) to eeprom
 */
void EEPROM_write_int(unsigned int adr, unsigned int val);
 

/*
 * function for copy data from EEPROM to RAM buffer 
 */
void EEcmp(uint8_t buf[], int EEaddress, uint8_t len);

/*
 * function for copy data from RAM buffer to EEPROM 
 */
void cmpEE(uint8_t buf[], int EEaddress, uint8_t len);

#endif

