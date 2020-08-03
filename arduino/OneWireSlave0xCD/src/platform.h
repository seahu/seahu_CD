#ifndef SH0xDD_platform_h
#define SH0xDD_platform_h
    // frist define some macros before include header of OneWireSlave0xDD librarry
    #define __AVR_ATtiny85__ // more about suported platform in file suportedMCU.h of OneWireSlave0xDD library
                             // some platform as arduino NG or later, leonardo, micro, nano are auto detected by arduino IDE
    #define Enable_Analog_comapartor // enable Analog Comaparto for check loss power (check or update suportedMCU.h in OneWireSalve0x55 library)
    #include "OneWireSlave0xCD.h"
#endif
