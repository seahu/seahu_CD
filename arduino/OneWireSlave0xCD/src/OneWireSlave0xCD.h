#ifndef OneWireSlave0xCD_h
  #define OneWireSlave0xCD_h

  #include "OneWireSlave.h"
  #include <avr/pgmspace.h>
  #include <EEPROM.h>
  /*
   * 1-Wire slave with family code 0xDD
   * This devices is specialy for fi. SEAHU and his 1-wire devices measure as humudity, presure, wind, light, curent, counter and multi measurment devices as huminity+temperature,actors, ... 
   * more on (www.seahu.cz)
   * 
   *  The OneWireSlave0xCD object extend object OneWireSlaveExtend and OneWireSlave about methods/events sutables for easy create 1-Wire slave device with famyly code 0xDD
   *  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
   *  
   *  This 1-Wire devices may have more sensors or actors. Every sesnror or actor has your own section. Every section has factory (your) description (only for read), user descriton (read/write), 
   *  space for data in RAM, own configuration byte indicating data type and some more, function from measuren or actor.
   *  
   *  Create your own device is easy, just create a field with description of sections and define functions for measured.
   *  Then run the init function from the arduini setup section. Its all. The program is running into backgroud through the interrupt.
   *  In the main loop do not use interrupt (this may break 1-Wire communication). Measured function is call from iterrupt therefore must be short as is posible. 
   *  If measure take more time then in measure function create only flag and make measure into main loop.
   *  After end measure to not forget set READ_FLAG in Control register for measured section this flag is automatic cleared before masured function is call.
   *  1-Wire addres will be generated in compile proces from actual date-time.
   *  
   * In detail:
   * Every measurment or actor section have our own status register and actual vulue, min. value, max. value for alarm, 32B description and 64B for notes registers, 
   * stored in structure "section"..
   * 
   * Exapmle of code:
   * ----------------
   * 
   *     OneWireSlave0xCD ow; //define One Wire object
   *     
   *     void mTemperature(bool force_now) // frist must be declared every measurent function used into section array.
   *                   {   
   *                    if (force_now==false) return; // force_now=true means that this function is called as part interrup service and must be a short as is posible. If measrment take more time, than leave for later from main loop.
   *                    union value0xCD mewValue; // declare place for new value
   *                    ...
   *                    mesurment code;
   *                    ...
   *                    mewValue.u8=250; // store value (mewValue.b, mewValue.u16, mewValue.u32, mewValue.i16, mewValue.i32, mewValue.ch[0-3], mewValue.u8t[0-3]) as definition union more bellow
   *                    ow.write_new_value(0, mewValue); // write new valu into setion 0 with all tests
   *                   }
   *     
   *     const char humidity[] PROGMEM = "Humidity [%]"; // factory description is good to store in PROGMEM
   *     const char temperature[] PROGMEM = "Temperature [C]";
   *
   *     section sections[]={ // define section array (this array must be definet as global varialbe (because must be aviable throucht interruot).
   *                         //{ seting and type, actual value, descrition, measure function, actual alarm status, lock }
   *                           { C_READ + C_WRITE + C_ALARM + C_U8BIT, { .u8t={0, 0, 0, 0} }, humidity, mTemperature, false, false },
   *                           { C_READ + C_WRITE + C_ALARM + C_16BIT, { .u8t={0, 0, 0, 0} }, temperature, NULL, false, false },
   *                         };
   *    void setup() 
   *                {
   *                 ow.ini(CONT_OF_SECTIONS, sections); // intialization of one wire interface in bacground throught interrupt
   *                }
   *    
   *    void loop() 
   *               {
   *                 ow.foreground_measure(false); // if argunet is true, then will be call every meseruent functions(force_now=true), otherwise
   *                                               // wil be call only mesurent function witch section has set alarm or before was called as part
   *                                               // of interrupt service, bat return whithou measurment.
   *                 ...
   *                 do what you want
   *                 ...
   *               }
   * 
   * 
   *  GENERY DESCRIPTION OF ONE WIRE DEVICE WITH FAMILY CODE 0xDD
   *  ------------------------------------------------------------
   * 
   *  REGISTER and SECTION DESCRIPTION:
   *  ---------------------------------
   *  STATUS - contain number of measurment or output sections
   *  SELECT - contain number of actual selected section (after start default is 0)
   *  
   *    STRUCTURE OF MEASURMENT SECTION:
   *    -------------------------------
   *      CONTROL register - 8bit 
   *        
   *      
   *        bit 0 R   : READ FLAG  1 - section value can be read , 0 - measuring now not ready for read
   *        bit 1 R   : WRITE FLAG 1 - section value is writable for output devices, 0 - actual value is no writable (for only input section)
   *        bit 2 R   : ALARM FLAG 1 - section have alarm function
   *        bit 3 R   : \
   *        bit 4 R   :  = TYPE VALUE FLAG 0 - bool, 1 - unsigned 8bit, 2 - unsigned 16bit, 3 - unsigned 24bit, 4 - unsigned 32bit,
   *        bit 5 R   : ALARM STATUS FLAG - actual sesion alarm status
   *        bit 6 R/W : 1 - on min. value alarm,  0 - off min. value alarm
   *        bit 7 R/W : 1 - on max. value alarm , 0 - off max. value alarm
   *        
   *        For easy set control register is prepared next constants:
   *        C_READ      - actual value can be read
   *        C_WRITE     - actual value is writable for output devices
   *        C_ALARM     - section have alarm function
   *        C_MIN_ALARM - on min. value alarm
   *        C_MAX_ALARM - on max. value alarm
   *        C_MEMORY    - section not have value (value is always 0x00), instead it of value is stored into 32B RAM like user description of section
   *        C_BOOL      - section contain bool value
   *        C_U32BIT    - section contain unsigned 32 bit binary value
   *        C_32BIT     - section contain signed 32 bit binary value
   *        
   *        etc. value (C_READ+C_WRITE+C_ALARM+C_U16BIT) means value in section is readable, writeable. witch alarm options curently alarm min max off and witch data type of value is unsignet integer 16b.
   *        
   *        PS: for bool value can activated min and max  alarm and set min value 0 a max value 1 id equal set sessitivy alarm for change.
   *        This usefull for create for PIO pins with alarm functions.
   *        
   *        
   *      TYPE OF ONE WIRE REGISTERS OF EVERY SECTION
   *      -------------------------------------------
   *      ACTUAL_VALUE register R/W       - 32bit space for store actual value
   *      MIN_ALARM_VALUE register R/W    - 8/16bit (depand on bit6 of CONTROL register) min value for alarm
   *      MAX_ALARM_VALUE register R/W    - 8/16bit (depand on bit7 of CONTROL register) max value for alarm
   *      DESCRIPTION register R          - 64Byte for short factory description of section
   *      NOTE register R/W               - 32Byte for custom notes of section
   *      
   *      
   *  1-Wire commands:
   *  ---------------
   *   measurment:
   *  -  0x44 - measure command
   *    
   *   read registers:
   *    16bit
   *  -  0xF5 - read from ACTUAL_VALUE register
   *  -  0xFA - read from MIN_ALARM_VALUE register
   *  -  0xFB - read from MAX_ALARM_VALUE register
   *    
   *    description and notes
   *  -  0xF1 - read factory description device (64Bytes)
   *  -  0xF7 - read from factory DESCRIPTION of section (64Bytes)
   *  -  0xF8 - read from user NOTE of section (32Bytes)
   *    
   *    read from control, select  and status register
   *  -  0xF2 - read number of sections (8bit)
   *  -  0xF3 - read from CONTROL register (8bit) of section
   *    
   *    write into registers:
   *  -  0x55 - write into ACTUAL_VALUE
   *  -  0x5A - write into MIN_ALARM_VALUE
   *  -  0x5B - write into MAX_ALARM_VALUE
   *    
   *    write description and notes
   *  -  0x58 - write into NOTE of section (32Bytes)
   *    
   *    write from control, select  and status register
   *  -  0x53 - write into CONTROL register (8bit) of section
   *    
   *    
   *    STRUCTURE OF EEPROM:
   *    --------------------
   *    record of one section:
   *    1B - CONTROL REGISTER
   *    4B - ACTUAL VALUE (etc. for counter when power is go down)
   *    4B - MIN ALARM VALUE
   *    4B - MAX ALARM VALUE
   *    32B - USER NOTE
   *    ---------------
   *    
   *  
   */

 
  // enable or disable debug
  //#define OneWireSlaveDebug // if you want use debug, must define this definition in main project file before include this file. 
                              // czech note: toto se nesmi odkomentovat, protoze compiler .cpp kompiluje nejak zvlast mimo hlavni .uni soubor a oak jaksi nezan pojem Serial ci Serial1 (proc to tak dela mi neni jasne)
  #ifdef OneWireSlaveDebug // if enabled debug then Serial port depend of type MCU
    #ifdef __AVR_ATmega32U4__
      #define serial Serial1  // serial port for arduino Leonardo
    #endif
    #ifdef __AVR_ATmega8__
      #define serial Serial  // serial port for arduino NG or older
    #endif
    #ifdef __AVR_ATmega328__
      #define serial Serial  // serial port for arduino Nano
    #endif
  #endif

  // family ROM code
  #define FAM 0xCD

  // EEPROM SIZE OF SECTION ENTRY
  #define EE_SIZE_CONTROL          1
  #define EE_SIZE_VALUE            4 // the same of ACTUL, MIN, MAX VALUE
  #define EE_SIZE_USER_NOTE        32
  // EEPROM OFFSET FOR SECTIONS ENTRY
  #define EE_FIRST_RUN        0 // EEPROM address for check frist run (frist run write default values into EEPROM and save 0xAA on this address)
  #define EE_START_POINT      1
  #define EE_CONTROL          0
  #define EE_ACTUAL_VALUE     EE_CONTROL         + EE_SIZE_CONTROL    // 1
  #define EE_MIN_ALARM_VALUE  EE_ACTUAL_VALUE    + EE_SIZE_VALUE      // 5
  #define EE_MAX_ALARM_VALUE  EE_MIN_ALARM_VALUE + EE_SIZE_VALUE      // 9
  #define EE_USER_NOTE        EE_MAX_ALARM_VALUE + EE_SIZE_VALUE      // 13
  #define EE_SIZE_OF_SECTION  EE_USER_NOTE       + EE_SIZE_USER_NOTE  // 77
  
   
  // CONTROLL BYTE FLAGS ---------------------
  #define C_READ            0b00000001
  #define C_WRITE           0b00000010
  #define C_ALARM           0b00000100
  #define C_MEMORY          0b00000000
  #define C_BOOL            0b00001000
  #define C_U32BIT          0b00010000
  #define C_32BIT           0b00011000
  #define C_ALARM_STATUS    0b00100000
  #define C_MIN_ALARM       0b01000000
  #define C_MAX_ALARM       0b10000000

  #define LOCK              { _OW_lock=true; (p_sections_+actual_section_)->lock;} // lock actual section (frist lock global _OW_lock)
  #define UNLOCK            { (p_sections_+actual_section_)->lock=false; _OW_lock=false;} // unlock actual section (frist ublock section lock than global _OW_lock)

/*
 * actual value in section structure can be contains various types, therefore is preferred wrap it into union
 */
  union value0xCD {
    bool       b;
    uint32_t   u32;
    int32_t    i32;
    char       ch[4];
    uint8_t    u8t[4];
  };
 
  // structure of section
  typedef struct {
      uint8_t control;
      //uint8_t actualValue[4];
      union value0xCD actualValue;
      const char* shortDescription PROGMEM ;
      const char* memory; // for section who use userDescription as 32B RAM memory, otherwise set to null
      //void (*measreCmd)(void); // measure function return true if set new value ale false. Argument now tell if measured function must measure now (during server interrup or letter in main loop.
      void (*measreCmd)(bool force_now); // measure function, argument now tell if measured function must measure now (during server interrupt or letter in main loop).
      //bool alarm;
      bool lock;  // lock for write new data during his One Wire reading or writing, real lock is logical AND between this lock and global _OW_lock (because only _OW_lock is clean by One Wire reset pulse, whilst this lock unlock only if read or write function ends OK) 
    } section;

  // global variables

  // class
  class OneWireSlave0xCD:public OneWireSlave {
    public:
      // functions - public declaration
      void ini(uint8_t number_of_sections, section userSections[], const char* device_description PROGMEM); // ini universal measurment library
      void write_new_value(uint8_t section, union value0xCD mewValue); // write new value into section with all control lock, min. max, change + update measurment flag (=read flag in control byte of section)
      void foreground_measure(bool all); // foregrount measuring if all=false then measu only measurment with set alarm chcek or new measurment flag
      void save_default_values(); // save default values into EEPROM used only for frist run device
      void save_values(); // save all actual values and seting from control byte (other values as min, max alarm value and user note not necessary save because are directly stored only in EEPROM)
      void load_values(); // load all actual values and seting from control byte (other values as min, max alarm value and user note not necessary save because are directly stored only in EEPROM)
  };
  

#endif
