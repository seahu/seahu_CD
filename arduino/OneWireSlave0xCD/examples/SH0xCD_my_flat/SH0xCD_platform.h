#ifndef SH0xCD_platform_h
#define SH0xCD_platform_h
    // frist define some macros fefore include header of OneWireSlave0xCD librarry
    #define __AVR_ATtiny85__ // more about suported platform in file suportedMCU.h of OneWireSlave0xCD library
                             // some platform as arduino NG or later, leonardo, micro, nano are auto detected by arduino IDE
    #define Enable_Analog_comapartor // enable Analog Comaparto for check loss power (check or update suportedMCU.h in OneWireSalve0x55 library)
    #include "OneWireSlave0xCD.h"
#endif
