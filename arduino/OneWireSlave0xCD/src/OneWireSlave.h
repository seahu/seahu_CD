#ifndef OneWireSlave_h
#define OneWireSlave_h
/*
 * This OneWireSlave librray exist thanks Erik Palsbo and his perfect code in:
 * http://palsbo.com/arduino/OneWireSlave.html
 * oreginal by Copyright (C) 2016 erik (at) palsbo.com
 * modify by Ondrej Lycka  (C) 2020 info (at) seahu.cz
 * 
 * Description of One Wire Slave emulation:
 * ----------------------------------------
 *  Now the OneWireSlave protocol is isolated in it’s own library as an object. 
 *  The OneWireSlave object has four methods/events: 
 * 
 * uint8_t crc8(const uint8_t* data, uint8_t numBytes);
 *  // calculate crc8 of a buffer "data" with the length "numBytes"
 *  
 *  void begin(void (*onCommand)(uint8_t), uint8_t* owId);  
 *  // Initialize the OneWireSlave object and sets the callback  
 *  // routine "onCommand".  
 *  // When the OneWireSlave object detects a Command (set pio,   
 *  // get pio, set scratchpad, get scratchpad), "onCommand" is called.  
 *  // Based on the command, the OneWire slave will respond by   
 *  // reading or writing a buffer of bytes.
 *  
 *  void read(uint8_t* buffer, uint8_t numBytes, void (*complete)());  
 *  // ask OneWireSlave object to receive ‘length’ bytes into ‘buffer’.  
 *  // call function "complete" on fininsh. 
 *  
 *  void write(uint8_t* buffer, uint8_t numBytes, void (*complete)());  
 *  // ask OneWireSlave object to send 'numBytes' from 'buffer'  
 *  // call function "complete" on fininsh.     
 *  
 *  void reset();   
 *  // ask OneWireSlave to reset connection (send break)
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "suportedMCU.h"

extern uint8_t*         _owid;           // pointer to ROM code of One Wire device
extern volatile bool    _OW_rc;          // ready to control flag 1=device is selected for OW comunication
extern volatile bool    _OW_alarm;       // alarm status 1=alarm, 0=normal status
extern volatile bool    _OW_lock;        // lock update data during his reading or writeing
                                         //  This lock can set and unset by higger level service program who service One Wire comunication,
                                         //  and always is clear by One Wire reset signal.
extern volatile uint8_t    _OW_crc_mode; // crc mode used for calculate crc during read or write 0=no calculate 1=crc8 2=crc16
extern volatile uint8_t    _OW_crc8;     // crc8 check sum calculate during write or read
extern volatile uint16_t    _OW_crc16;    // crc16 check sum calculate during write or read
extern volatile uint8_t     _OW_crc8c;     // pro ladeni crc pozdeji smazat

// hack to force another MCU speed than get from arduino IDE
//F_CPU 8000000 // fake speed MCU definition (for arduino time functions)

// define speed timer constad C=1 for 8Mhz/64(timer prescalar), C=2 for 16Mhz/64(timer prescalar)
#ifndef TIMER_SPEED_C // TIMER_SPEED_C may be defined in suportedMCU.h . If not defined try automaticaly detect 8/16Mhz from arduino IDE bay macro F_CPU (suitable for most AVR MCU)
  #if F_CPU==8000000
    #define TIMER_SPEED_C 1
  #endif 
  #if F_CPU==16000000
    #define TIMER_SPEED_C 2
  #endif 
#endif

//States / Modes
#define C TIMER_SPEED_C // C=1 for 8Mhz, C=2 for 16Mhz
#define OWT_MIN_RESET 51*C
#define OWT_RESET_PRESENCE 4*C
#define OWT_PRESENCE 20*C
#define OWT_READLINE 2*C //3 for fast master, 4 for slow master and long lines (I have tested 2*C OK, 3*C-sometime error , 5*C-most error, 6*C- no functions)
#define OWT_LOWTIME 3*C //for fast master, 4 for slow master and long lines 
#define OWM_SLEEP 0  //Waiting for next reset pulse
#define OWM_RESET 1  //Reset pulse received 
#define OWM_PRESENCE 2  //sending presence pulse
#define OWM_READ_COMMAND 3 //read 8 bit of command
#define OWM_READ_ROM_COMMAND 4 // read 8 bit of ROM command
#define OWM_SEARCH_ROM 5  //SEARCH_ROM algorithms
#define OWM_MATCH_ROM 6  //test number
#define OWM_READ_SCRATCHPAD 7
#define OWM_WRITE_SCRATCHPAD 8
#define OWM_CHK_RESET 9  //waiting of rising edge from reset pulse
#define OWM_READ_PIO 10
#define OWM_WRITE_PIO 11
#define OWM_READ_MEMORY 12
#define OWM_WRITE_MEMORY 13
#define OWM_READ 14 //  read from master
#define OWM_WRITE 15  //  write to master

//Write a bit after next falling edge from master
//its for sending a zero as soon as possible
#define OWW_NO_WRITE 2
#define OWW_WRITE_1 1
#define OWW_WRITE_0 0

// CRC mode druing read/write
#define CRC_NO 0
#define CRC_8  1
#define CRC_16 2

class OneWireSlave {
  private:
  void beginReceiveBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)());
  void beginWriteBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)());
  public:
  void begin(void (*onCommand)(uint8_t), uint8_t* owId);
  uint8_t crc8(const uint8_t* data, uint8_t numBytes);
  void read(uint8_t* buffer, uint8_t numBytes, void (*complete)());
  void write(uint8_t* buffer, uint8_t numBytes, void (*complete)());
  static void reset();
  static void waitToCmd();
};




#endif

