#ifndef platform_h
#define platform_h
    // frist define some macros fefore include header of OneWireSlave librarry
    #define __AVR_ATtiny85__ // more about suported platform in file suportedMCU.h of OneWireSlave0xCD library
                             // some platform as arduino NG or later, leonardo, micro, nano are auto detected by arduino IDE
    //#define Enable_Analog_comapartor // enable Analog Comaparto for check loss power (check or update suportedMCU.h in OneWireSalve0x55 library)
    #include "OneWireSlave.h"
#endif
