#ifndef suportedMCU_h
#define suportedMCU_h
/*
 * 
 * This OneWireSlave librray exist thanks Erik Palsbo and his perfect code in:
 * http://palsbo.com/arduino/OneWireSlave.html
 * oreginal by Copyright (C) 2016 erik (at) palsbo.com
 * modify by Ondrej Lycka  (C) 2020 info (at) seahu.cz
 * 
 * Macro definitions for supported MCU in OneWireSlave librarry
 * 
 *  for new MCU must be defined:
 *  // OW_global setting
 *  #define OW_ADD_TO_GLOBAL // something declaration of global variables need for server new MCU
 *  //Pin setting
 *  #define CONF_PORT    // prepare 1-Wire pin as input on start (may set more registers)
 *  #define SET_LOW      // set 1-Wire line to low (as short as possible)
 *  #define RESET_LOW    // set 1-Wire pin as input (as short as possible)
 *  #define READ_PIN     // read from pin and return 0 or 1 (as short as possible)
 *  //Pin interrupt
 *  #define EN_OWINT     // enable pin interrupt 
 *  #define DIS_OWINT    // disable pin interrupt
 *  #define SET_RISING   // set pin interrupt at rising edge
 *  #define SET_FALLING  // set pin interrupt at falling edge
 *  #define CHK_INT_EN   // test if pin interrupt enabled
 *  #define PIN_INT      // the pin interrupt service routine
 *  //Timer setting and interrupt
 *  #define              // prepare Timer (may set more registers, prescalars,...) on start
 *  #define              // enable timer interrupt
 *  #define              // disable timer interrupt
 *  #define SET_TIMER(x) // set Timer to waint  the specified value x
 *  #define TIMER_INT    // the timer interrupt service routine
 *  //optimaly Analog comparator (for check loss power)
 *  #define CONF_AC	// prepare analog comparator to check loss power
 *  #define AC_INT      // the analog comparator interrupt service routine
 *  //optimaly timer speed constand
 *  TIMER_SPEED_C        // optimaly define speed timer constad TIMER_SPEED_C=1 for 8Mhz/64(timer prescalar), TIMER_SPEED_C=2 for 16Mhz/64(timer prescalar).
 *                       // If not defined will be try automaticaly detect 8/16Mhz from arduino IDE bay macro F_CPU (suitable for most AVR MCU).
 */


// Type of MCU
// on select:
            //#define __AVR_ATtiny85__       // auto select by arduino IDE TEST OK
            //#define __AVR_ATtiny84__       // NOT TESTED select manualy NO TEST YET
            ///#define __AVR_ATmega8__       // auto select by arduino IDE TEST OK
            //#define __AVR_ATmega328P__     // auto select by arduino IDE TEST OK
            //#define __AVR_ATmega32U4__     // auto select by arduino IDE TESY OK
            //#define __ESP32__              // NOT TESTED select manualy NO TEST YET
            
//#define __AVR_ATtiny85__   // uncomnet if not automaticaly select by arduino IDE
#ifdef __AVR_ATtiny85__
  /*
   *
   *                   AVR_ATtiny85
   *                  +------\/-----+   
   *   (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC
   *        A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) !prefered for OW!
   * (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)
   *           GND 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)
   *                  +-------------+
   *        
   *   #DX - meens pin with PWM
   *   
   *                   -------------
   *   GPIO FOR OW:
   *   pin n. | AVR label | arduino label | INT  | colision
   *   ----------------------------------------------
   *    7     | PB2       | D2            | INT0 | SCL
   *    
   *    TIMER FOR OW:
   *    timer  | comparator             | colision
   *    -------------------------------------------
   *    Timer0 | TOV0 (timer overflow) | arduino dely(), millis() functions  and #D0, #D1 PWM (cannot be used witch arduino core - colision with time function in arduino/hardware/arduino/avr/cores/arduino/wiring.c)
   *    Timer0 | OCR0A                 | #D0, #D1 PWM (OCR0A and OCR0B share PWM configuration, if one set to PWM, other can't be user other way. Because in PWM mode OCR0A and OCR0B use doble buffered registers and aplicate new value after counter overflow)
   *    Timer0 | OCR0B                 | (#D0, #D1 PWM nay be, because this pim may be connect to conflict OC0B or no conflict OC1A, depents how to use arduino pwm function, must be test)
   *    Timer1 | OCR1A                 | <-- best choice
   *    Timer1 | OCR1B                 | #D4 PWM
   *    
   *    best choice is Timer0-OCR0B becouse teoreticanly meby be without any conflict
   *    
   *    arduino-AVR_ATtiny85:
   *    ---------------------
   *    arduino not have native support for AVR_ATtiny85. This supoort have to add :
   *    the procedure applies for arduino IDE 1.6.4 and hight
   *    In menu File/Preferences. In the dialg you fill item Additional Board Manager with: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json a stiskněte OK. Toto jádro je sice uváděny jako první, ale nedoporučuji ho používat. Bylo první z historických důvodů, ale jeho autor nemá na jeho vývoj tolik času, jako je jádro, zmíněné v odstavci o alternativních jádrech.
   *    And then use function ools/Board/Boards Manager. In the list find attiny and install it.
   *     Beafore complile select:
   *      Board: ATtiny25/45/85
   *      Procesor: ATtiny85
   *      Clock: Internal16MHz
   *      Select Programmer: (I use secundary arduino as ISP "Arduino as ISP")
   *      Burn Bootloader (this is necessary for set correct MCU clock)
   *      Burn by Sketch/Upload Using Programmer
   *
   *    More usefull information about ATtiny and arduino:
   *    https://www.arduinoslovakia.eu/page/attiny85
   */
  // OW_global setting
  #define OW_ADD_TO_GLOBAL // nothing for this MCU
  //OW Pin  
  // OW_PORT Pin 7  - PB2
  //Pin setting 
  #define OW_PORT PORTB //1 Wire Port
  #define OW_PIN PINB //1 Wire Pin as number
  #define OW_DDR DDRB  //pin direction register
  #define OW_PIN_MASK (1<<PINB2)  //Pin as bit mask for use in output, input and direction setting registers
  #define CONF_PORT {OW_DDR&=~OW_PIN_MASK;OW_PORT&=~OW_PIN_MASK;} // set 1-wire pin on start as input
  #define SET_LOW   {OW_DDR|=OW_PIN_MASK;}   //set 1-Wire line to low
  #define RESET_LOW {OW_DDR&=~OW_PIN_MASK;}  //set 1-Wire pin as input
  #define READ_PIN  ((OW_PIN & OW_PIN_MASK) == OW_PIN_MASK) // read from pin and return 0 or 1
  //Pin interrupt 
  #define EN_OWINT    {GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
  #define DIS_OWINT   GIMSK&=~(1<<INT0);  //disable interrupt
  #define SET_RISING  MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
  #define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
  #define CHK_INT_EN  (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
  #define PIN_INT     ISR(INT0_vect)  // the interrupt service routine
  //Timer setting and interrupt
  // on select:
	    #ifndef Timer0_TOV0
	    #ifndef Timer0_OCR0A
	    #ifndef Timer0_OCR0B
	    #ifndef Timer1_OCR1A
	    #ifndef Timer1_OCR1B
		// in not defined then set defalt Timmer:
	    	//#define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() and PWN on D0.D1 with use timer0 (D1 steel can use PWM by timer1, but not from arduino librrary) )
	    	#define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D0,D1 PWM (D1 steel can use PWM by timer1, but not from arduino librrary))
	    	//#define Timer0_OCR0B // select timer0 with comparator B interrupt (colision with D0,D1 PWM, (D1 steel can use PWM by timer1, but not from arduino librrary))
	    	//#define Timer1_OCR1A // select timmer1 with comparator A interrupt <-- best choice
	    	//#define Timer1_OCR1B // select timmer1 with comparator B interrupt (colision with D4 PWM)
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
  #ifdef Timer0_TOV0
    #define CONF_TIMER   {TCCR0A &= ~(1<<WGM00) & ~(1<<WGM01); TCCR0B &= ~(1<<WGM02); TCCR0B=(1<<CS00)|(1<<CS01);} // Off PWM, set prescalar 8mhz /64 couse 8 bit Timer interrupt every 8us
    #define EN_TIMER     {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
    #define DIS_TIMER    TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
    #define TIMER_INT    ISR(TIM0_OVF_vect) //the timer interrupt service routine
    #define SET_TIMER(x) TCNT0 = ~( x ) // set Timer0 by the specified value below
  #endif
  #ifdef Timer0_OCR0A
    #define CONF_TIMER   {TCCR0A &= ~(1<<COM0A0) & ~(1<<COM0A1) & ~(1<<WGM00) & ~(1<<WGM01); TCCR0B &= ~(1<<WGM02);TCCR0B=(1<<CS00)|(1<<CS01);} // timer normal mode (no connect to pin), Off PWM, set prescalar 8mhz /64 couse 8 bit Timer interrupt every 8us
    #define EN_TIMER     {TIMSK |= (1<<OCIE0A); TIFR|=(1<<OCF0A);} //enable timer interrupt
    #define DIS_TIMER    TIMSK  &= ~(1<<OCIE0A); // disable timer interrupt
    #define TIMER_INT    ISR(TIM0_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x) OCR0A = TCNT0 +  x // set compare register for new time
  #endif
  #ifdef Timer0_OCR0B
    #define CONF_TIMER   {TCCR0A &= ~(1<<COM0B0) & ~(1<<COM0B1) & ~(1<<WGM00) & ~(1<<WGM01); TCCR0B &= ~(1<<WGM02); TCCR0B=(1<<CS00)|(1<<CS01); TCCR0B &= ~(1<<CS02);} // timer normal mode (no connect to pin), Off PWM, set prescalar 8mhz /64 couse 8 bit Timer interrupt every 8us
    #define EN_TIMER     {TIMSK |= (1<<OCIE0B); TIFR|=(1<<OCF0B);} //enable timer interrupt
    #define DIS_TIMER    TIMSK  &= ~(1<<OCIE0B); // disable timer interrupt
    #define TIMER_INT    ISR(TIM0_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x) OCR0B = TCNT0 + x // set compare register for new time
  #endif
  #ifdef Timer1_OCR1A
    #define CONF_TIMER   {TCCR1 &= ~(1<<CTC1) & ~(1<<PWM1A) & ~(1<<COM1A1) & ~(1<<COM1A0) & ~(1<<CS13); TCCR1|= (1<<CS12) | (1<<CS11) | (1<<CS10); PLLCSR=0;}  // Off top by OCR1C, Off PWM on  OCR1A, set prescalar 8mhz /64 couse 8 bit Timer interrupt every 8us, clk from system
    #define EN_TIMER     {TIMSK |= (1<<OCIE1A); TIFR|=(1<<OCF1A);} //enable timer interrupt
    #define DIS_TIMER    TIMSK  &= ~(1<<OCIE1A); // disable timer interrupt
    #define TIMER_INT    ISR(TIM1_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x) OCR1A = TCNT1 +  x // set compare register for new time
  #endif
  #ifdef Timer1_OCR1B
    #define CONF_TIMER   {GTCCR &= ~(1<<PWM1B) & ~(1<<COM1B1) & ~(1<<COM1B0); TCCR1 &= ~(1<<CTC1) & ~(1<<CS13); TCCR1|= (1<<CS12) | (1<<CS11) | (1<<CS10); PLLCSR=0;}  // Off top by OCR1C, Off PWM on  OCR1B, set prescalar 8mhz /64 couse 8 bit Timer interrupt every 8us, clk from system
    #define EN_TIMER     {TIMSK |= (1<<OCIE1B); TIFR|=(1<<OCF1B);} //enable timer interrupt
    #define DIS_TIMER    TIMSK  &= ~(1<<OCIE1B); // disable timer interrupt
    #define TIMER_INT    ISR(TIM1_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x) OCR1B = TCNT1 +  x // set compare register for new time
  #endif
  //Analog comparator (for check loss power)
  // on select:
	//#define Enable_Analog_comapartor
  #ifdef Enable_Analog_comapartor
    #define CONF_AC      {ACSR &= ~(1<<ACD); ACSR |= (1<<ACBG)| (1<<ACI) | (1<<ACIE) | (1<<ACIS1)| (1<<ACIS0); ADCSRB&=~(1<<ACME);} // prepare analog comparator to check loss power
    #define AC_INT       ISR(ANA_COMP_vect) // the analog comparator interrupt service routine
  #endif
  #define TIMER_SPEED_C 1
#endif



#ifdef __AVR_ATtiny84__
  /*
   * NOT TESTED YET
   *                                       AVR_ATtiny84
   *                                     +------\/------+   
   *                                VCC -|1 VCC   GND 14|- GND
   *                                D10 -|2 PB0   PA0 13|- D0,A0 (AREF)
   *                                 D9 -|3 PB1   PA1 12|- D1,A1
   *                             /RESET -|4 PB3   PA2 11|- D2,A2
   *  !prefered for OW! (INT0,OC0A) #D8 -|5 PB2   PA3 10|- D3,A3
   *                  (ICP,OC0B) A7,#D7 -|6 PA7   PA4  9|- D4,A4 (SCL,T1)
   *             (OC1A,SDA,NOSI) A6,#D6 -|7 PA6   PA5  8|- #D5,A6 (OC1B,MISO)
   *                                     +--------------+
   *
   *   #DX - meens pin with PWM
   *   
   *                   -------------
   *   GPIO FOR OW:
   *   pin n. | AVR label | arduino label | INT  | colision
   *   -----------------------------------------------------
   *    5     | PB2       | D2            | INT0 | D8 PWM
   *    
   *    TIMER FOR OW:
   *    timer  | comparator             | colision
   *    -------------------------------------------
   *    Timer0 | TOV0 (timer overflow) | arduino dely(), millis() functions  and #D0 PWM
   *    Timer0 | OCR0A                 | #D8 PWM  <-- best choice with use Timer2 and comparator OCR2A (INT and timer share same pin)
   *    Timer0 | OCR0B                 | #D7 PWM 
   *    best choice is Timer0-OCR0B becouse teoreticanly meby be without any conflict
   *    
   *    arduino-AVR_ATtiny85/84:
   *    ---------------------
   *    arduino not have native support for AVR_ATtiny85. This supoort have to add :
   *    the procedure applies for arduino IDE 1.6.4 and hight
   *    In menu File/Preferences. In the dialg you fill item Additional Board Manager with: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json a stiskněte OK. Toto jádro je sice uváděny jako první, ale nedoporučuji ho používat. Bylo první z historických důvodů, ale jeho autor nemá na jeho vývoj tolik času, jako je jádro, zmíněné v odstavci o alternativních jádrech.
   *    And then use function ools/Board/Boards Manager. In the list find attiny and install it.
   *    
   *    More usefull information about ATtiny and arduino:
   *    https://www.arduinoslovakia.eu/page/attiny85
   */
  // OW_global setting
  #define OW_ADD_TO_GLOBAL // nothing for this MCU
  //OW Pin  
  // OW_PORT Pin 5  - PB2
  //Pin setting 
  #define OW_PORT PORTB //1 Wire Port
  #define OW_PIN PINB //1 Wire Pin as number
  #define OW_DDR DDRB  //pin direction register
  #define OW_PIN_MASK (1<<PINB2)  //Pin as bit mask for use in output, input and direction setting registers
  #define CONF_PORT {OW_DDR&=~OW_PIN_MASK;OW_PORT&=~OW_PIN_MASK}; // set 1-wire pin on start as input
  #define SET_LOW   {OW_DDR|=OW_PIN_MASK;}   //set 1-Wire line to low
  #define RESET_LOW {OW_DDR&=~OW_PIN_MASK;}  //set 1-Wire pin as input
  #define READ_PIN  ((OW_PIN & OW_PIN_MASK) == OW_PIN_MASK) // read from pin and return 0 or 1
  //Pin interrupt 
  #define EN_OWINT {GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
  #define DIS_OWINT  GIMSK&=~(1<<INT0);  //disable interrupt
  #define SET_RISING MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
  #define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
  #define CHK_INT_EN (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
  #define PIN_INT ISR(INT0_vect)  // the interrupt service routine
  //Timer setting and interrupt
  // on select:
	    #ifndef Timer0_TOV0
	    #ifndef Timer0_OCR0A
	    #ifndef Timer0_OCR0B
		// in not defined then set defalt Timmer:
            	// #define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() )
            	// #define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D0 PWM)
            	#define Timer0_OCR0B // select timer0 with comparator B interrupt (may be colison with D1 PWM, but not testetd yet) <-- best choice
	    #endif
	    #endif
	    #endif
  #ifdef Timer0_TOV0
    #define CONF_TIMER TCCR0B=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);} //enable timer interrupt
    #define DIS_TIMER TIMSK0  &= ~(1<<TOIE0); // disable timer interrupt
    #define TIMER_INT ISR(TIM0_OVF_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  TCNT0 = ~( x ) // set Timer0 by the specified value below
  #endif
  #ifdef Timer0_OCR0A
    #define CONF_TIMER TCCR0B=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER {TIMSK0 |= (1<<OCIE0A); TIFR|=(1<<OCF0A);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<OCIE0A); // disable timer interrupt
    #define TIMER_INT ISR(TIM0_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR0A = TCNT0 +  x // set compare register for new time
  #endif
  #ifdef Timer0_OCR0B
    #define CONF_TIMER TCCR0B=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER {TIMSK0 |= (1<<OCIE0B); TIFR|=(1<<OCF0B);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<OCIE0B); // disable timer interrupt
    #define TIMER_INT ISR(TIM0_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR0B = TCNT0 +  x // set compare register for new time
  #endif
#endif



#ifdef __AVR_ATmega8__
  /* 
   *  Arduino NG or later
   *  ATMEL ATMEGA8 & 168 / ARDUINO
   *  TESTED OK
   *                        
   *                                 +-------\/-------+
   *                   (RESET)      -|1  PC6    PC5 28|- A5   (SCL)   
   *                   (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)   
   *                   (TXD)     D1 -|3  PD1    PC3 26|- A3      
   *                   (INT0)    D2 -|4  PD2    PC2 25|- A2
   * !prefered for OW! (INT1) +#D3  -|5  PD3    PC1 24|- A1
   *                            D4  -|6  PD4    PC0 23|- A0
   *                                -|7  VCC    GND 22|-  
   *                                -|8  GND   AREF 21|-  
   *                   (XTAL1) *D20 -|9  PB6   AVCC 20|-  
   *                   (XTAL2) *D21 -|10 PB7    PB5 19|- D13  (SCK)
   *                   (T1)   +#D5  -|11 PD5    PB4 18|- D12  (MISO)
   *                   (AIN0) +#D6  -|12 PD6    PB3 17|- #D11 (MOSI,OC2)
   *                   (AIN1)   D7  -|13 PD7    PB2 16|- #D10 (OC1B)
   *                   (IPC1)   D8  -|14 PB0    PB1 15|- #D9  (OC1A)
   *                                 +----------------+
   * 
   * (#  indicates thw PWM pins)
   * (+# indicates the additional PWM pins on the ATmega168.)
   * (*  indicates additional two free pins instead external crystal if is used core: https://github.com/MCUdude/MiniCore this core have more options atc, change clk of MCU)
   * 
   *   GPIO FOR OW:
   *   pin n. | AVR label | arduino label | INT  | colision
   *   -----------------------------------------------------
   *    4     | PD2       | D2            | INT0 | free
   *    5     | PD3       | D3            | INT1 | free <-- best choice for pin compatibility with best choice for ATmega328P
   *    
   *    TIMER FOR OW:
   *    timer  | comparator             | colision
   *    -------------------------------------------
   *    Timer0 | TOV0 (timer overflow) | arduino dely(), millis() functions  and #D0 PWM
   *    Timer2 | OCR2                  | #D11 PWM <-- best choice because disalbe only PWM mode on D11 and still left two PWM pins (on ATmega168 five PWM pins)
   * 
   * 
   *  For HW is based on Atmega8 MCU (8KB ROM + 1KBRAM)
   *  in arduino IDE must be select tools->Board->Arduino NG or older
   *  tools->Processor->ATmega8
   *  programing direct withou boot loader Sketch->Upload Using Programmer
   *  
   *  1-Wire bus is hard for timming so must be used external crystel on 16MHz MCU and must be setting for using external crystal.
   *  This is czech howto for this:
   *    Pro pokusy s ATmega8a, jak jsem zjistil tak vyvojeove prostredi arduina nenahrava fuse bits, ale jen samotny program.
   *    Od vyroby je MCU nastaveno na vnitrni oscilator 1MHz ja vsak potrebuji nastavit vnesi 16MHz krystal. To se musi provest samostatne
   *    pomoci averdude (tj. je potreba si tento program nainstaovat protoze  mam pocit ze ten co je v arduinu zakomponovan nelze tak jednoduse
   *    samostatne spustit). Nicmene pokud se v ardujinu v menu file->preference zaskrkne show verbose output during uplolat, tak je vystupupovidanejsi
   *    a lse s neho odkoukat jake je potreba pouzit parametry za prikaz averdude a specificky programator. Hodnoty pro horni a dolni pojistku lze odkoukat
   *    ze souoru arduino-x.x.x/hardware/arduino/avr/boards.txt nebo si je nechat vygenerovat na weovem kalkulatoru http://www.engbedded.com/fusecalc .
   *     
   *     Ja pouzivam jedno arduino jako ISP programator a cilovy mcu ATmega8a  pro nej je potreba zadat:
   *     avrdude -v -patmega8 -cstk500v1 -P/dev/ttyACM0 -b19200 -U lfuse:w:0xdf:m -U hfuse:w:0xca:m
   *     
   *     Do puvodniho nastaveni je pak potreba zadat:
   *     avrdude -v -patmega8 -cstk500v1 -P/dev/ttyACM0 -b19200 -U lfuse:w:0xe1:m -U hfuse:w:0xd9:m 
   *     
   *     No a kdybych si chel vycist aktualni nastaveni techto fuse bits tak nasledujicim zpusobem: (vysledek se uozi do souboru low_fuse_val.hex a :high_fuse_val.hex)
   *     avrdude -v -patmega8 -cstk500v1 -P/dev/ttyACM0 -b19200 -U lfuse:r:low_fuse_val.hex:h -U hfuse:r:high_fuse_val.hex:h
   * 
   */
  // OW_global setting
  #define OW_ADD_TO_GLOBAL // nothing for this MCU
  //OW Pin  
  // on select:
	    #ifndef OWpin_PD2
	    #ifndef OWpin_PD3
		// in not defined then set defalt OWpin:
            	//#define OWpin_PD2  // OW_PORT Pin 4  - PD2 with interrupt INT0
            	#define OWpin_PD3  // OW_PORT Pin 5  - PD3 with interrupt INT1
	    #endif
	    #endif
  #ifdef OWpin_PD2
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND2)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {GICR|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
    #define DIS_OWINT  GICR&=~(1<<INT0);  //disable interrupt
    #define SET_RISING MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
    #define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
    #define CHK_INT_EN (GICR&(1<<INT0))==(1<<INT0) //test if interrupt enabled
    #define PIN_INT ISR(INT0_vect)  // the interrupt service routine
  #endif
  #ifdef OWpin_PD3 (tested OK)
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND3)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {GICR|=(1<<INT1);GIFR|=(1<<INTF1);}  //enable interrupt 
    #define DIS_OWINT  GICR&=~(1<<INT1);  //disable interrupt
    #define SET_RISING MCUCR|=(1<<ISC11)|(1<<ISC10);  //set interrupt at rising edge
    #define SET_FALLING { MCUCR|=(1<<ISC11); MCUCR&=~(1<<ISC10);} //set interrupt at falling edge
    #define CHK_INT_EN (GICR&(1<<INT1))==(1<<INT1) //test if interrupt enabled
    #define PIN_INT ISR(INT1_vect)  // the interrupt service routine
  #endif
  //Pin setting for all variants
  #define CONF_PORT {OW_DDR&=~OW_PIN_MASK;OW_PORT&=~OW_PIN_MASK;} // set 1-wire pin on start as input
  #define SET_LOW   {OW_DDR|=OW_PIN_MASK;}   //set 1-Wire line to low
  #define RESET_LOW {OW_DDR&=~OW_PIN_MASK;}  //set 1-Wire pin as input
  #define READ_PIN  ((OW_PIN & OW_PIN_MASK) == OW_PIN_MASK) // read from pin and return 0 or 1

  //Timer setting and interrupt
  // on select:
	    #ifndef Timer0_TOV0
	    #ifndef Timer1_TOV1
	    #ifndef Timer1_OCR1A
	    #ifndef Timer1_OCR1B
	    #ifndef Timer2_TOV2
	    #ifndef Timer2_OCR2
		// in not defined then set defalt Timmer:
            	//#define Timer0_TOV0  // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() )
            	//#define Timer1_TOV1  // select timer1 with overflow interrupt (colision with PWM output on D9 and D10 ) (tested OK)
            	//#define Timer1_OCR1A // select timer1 with comparator OCR1A interrupt (colision with PWM output on D9 and D10 ) (tested OK)
            	//#define Timer1_OCR1B // select timer1 with comparator OCR1B interrupt (colision with PWM output on D9 and D10 ) (tested OK)
            	//#define Timer2_TOV2  // select timer2 with overflow interrupt (colison with arduino PWM output on D11 ) (tested OK)
            	#define Timer2_OCR2 // select interrupt by timer2 who has only one comparator OCR2. (colison with arduino PWM output on D11 ) (tested OK) <-- best choice
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
  #ifdef Timer0_TOV0
    #define CONF_TIMER TCCR0=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
    #define TCNT_REG TCNT0  //register of timer-counter
    #define TIMER_INT ISR(TIMER0_OVF_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  TCNT_REG = ~( x ) // set Timer0 by the specified value below
  #endif
  #ifdef Timer1_TOV1
    #define TCNT_REG TCNT1  //register of timer-counter
    #define CONF_TIMER {TCCR1A=0; TCCR1B=~(1<<CS12)&(1<<CS11)|(1<<CS10);} // 8mhz /64 couse 8 bit Timer interrupt every 8us (Timer2 has anotger prescalar then timer0, CS22=1,CS21=0,CS20=0 => clk/64)
    #define EN_TIMER {TIMSK |= (1<<TOIE1); TIFR|=(1<<TOV1);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<TOIE1); // disable timer interrupt
    #define TIMER_INT ISR(TIMER1_OVF_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  TCNT_REG = ~( x ) // set compare register for new time
  #endif
  #ifdef Timer1_OCR1A
    #define TCNT_REG TCNT1  //register of timer-counter
    #define OCR OCR1A // register of compare with timee-counter
    #define CONF_TIMER {TCCR1A=0; TCCR1B=~(1<<CS12)&(1<<CS11)|(1<<CS10);} // 8mhz /64 couse 8 bit Timer interrupt every 8us (Timer2 has anotger prescalar then timer0, CS22=1,CS21=0,CS20=0 => clk/64)
    #define EN_TIMER {TIMSK |= (1<<OCIE1A); TIFR|=(1<<OCF1A);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<OCIE1A); // disable timer interrupt
    #define TIMER_INT ISR(TIMER1_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR = TCNT_REG +  x // set compare register for new time
  #endif
  #ifdef Timer1_OCR1B
    #define TCNT_REG TCNT1  //register of timer-counter
    #define OCR OCR1B // register of compare with timee-counter
    #define CONF_TIMER {TCCR1A=0; TCCR1B=~(1<<CS12)&(1<<CS11)|(1<<CS10);} // 8mhz /64 couse 8 bit Timer interrupt every 8us (Timer2 has anotger prescalar then timer0, CS22=1,CS21=0,CS20=0 => clk/64)
    #define EN_TIMER {TIMSK |= (1<<OCIE1B); TIFR|=(1<<OCF1B);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<OCIE1B); // disable timer interrupt
    #define TIMER_INT ISR(TIMER1_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR = TCNT_REG +  x // set compare register for new time
  #endif
  #ifdef Timer2_TOV2
    #define TCNT_REG TCNT2  //register of timer-counter
    #define CONF_TIMER TCCR2=(1<<CS22); // 8mhz /64 couse 8 bit Timer interrupt every 8us (Timer2 has anotger prescalar then timer0, CS22=1,CS21=0,CS20=0 => clk/64)
    #define EN_TIMER {TIMSK |= (1<<TOIE2); TIFR|=(1<<TOV2);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<TOIE2); // disable timer interrupt
    #define TIMER_INT ISR(TIMER2_OVF_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  TCNT_REG = ~( x ) // set compare register for new time
  #endif
  #ifdef Timer2_OCR2
    #define TCNT_REG TCNT2  //register of timer-counter
    #define OCR OCR2 // register of compare with timee-counter
    #define CONF_TIMER TCCR2=(1<<CS22); // 8mhz /64 couse 8 bit Timer interrupt every 8us (Timer2 has anotger prescalar then timer0, CS22=1,CS21=0,CS20=0 => clk/64)
    #define EN_TIMER {TIMSK |= (1<<OCIE2); TIFR|=(1<<OCF2);} //enable timer interrupt
    #define DIS_TIMER TIMSK  &= ~(1<<OCIE2); // disable timer interrupt
    #define TIMER_INT ISR(TIMER2_COMP_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR = TCNT_REG +  x // set compare register for new time
  #endif
  //Analog comparator (for check loss power)
  //If voltage on AIN1 pin will be smaller than internal reference 2,56V then cause interrupt.
  // on select:
  #define Enable_Analog_comapartor
  #ifdef Enable_Analog_comapartor
    #define CONF_AC      {ACSR &= ~(1<<ACD); ACSR |= (1<<ACBG)| (1<<ACI) | (1<<ACIE) | (1<<ACIS1)| (1<<ACIS0); SFIOR &=~(1<<ACME);} // prepare analog comparator to check loss power
    #define AC_INT       ISR(ANA_COMP_vect) // the analog comparator interrupt service routine
  #endif
#endif


#ifdef __AVR_ATmega328P__
  /* 
   *  Arduino Nano
   *  TESTED OK (only problem with Timer0 TOV0, but this is general arduin problem, TOV0 is used form time function already)
   *  
   *                                       ATMEL ATMEGA328P
   *                                      +-------\/-------+
   *                        (RESET)      -|1  PC6    PC5 28|- A5   (SCL)   
   *                        (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)   
   *                        (TXD)     D1 -|3  PD1    PC3 26|- A3      
   *                        (INT0)    D2 -|4  PD2    PC2 25|- A2
   * !prefered for OW! (OC2B,INT1)   #D3 -|5  PD3    PC1 24|- A1
   *                                  D4 -|6  PD4    PC0 23|- A0
   *                                     -|7  VCC    GND 22|-  
   *                                     -|8  GND   AREF 21|-  
   *                                     -|9  PB6   AVCC 20|-  
   *                                     -|10 PB7    PB5 19|- D13  (SCK)
   *                     (OC0B,T1)   #D5 -|11 PD5    PB4 18|- D12  (MISO)
   *                   (OC0A,AIN0)   #D6 -|12 PD6    PB3 17|- #D11 (MOSI,OC2A)
   *                        (AIN1)    D7 -|13 PD7    PB2 16|- #D10 (OC1B)
   *                                  D8 -|14 PB0    PB1 15|- #D9  (OC1A)
   *                                      +----------------+
   * 
   * 
   *   GPIO FOR OW:
   *   pin n. | AVR label | arduino label | INT  | colision
   *   -----------------------------------------------------
   *    4     | PD2       | D2            | INT0 | free
   *    5     | PD3       | D3            | INT1 | PWM D3 <-- best choice with use Timer2 and comparator OCR2A (INT and timer share same pin)
   *    
   *    TIMER FOR OW:
   *    timer  | comparator             | colision
   *    -------------------------------------------
   *    Timer0 | TOV0 (timer overflow) | arduino dely(), millis() functions  and #D0 PWM !!!Compile problem inside arduino !! DO NOT USE !!(INT used for time function, already)
   *    Timer0 | OCR0A                 | #D6 PWM
   *    Timer0 | OCR0B                 | #D5 PWM
   *    Timer1 | OCR0A                 | #D9 PWM
   *    Timer1 | OCR0B                 | #D10 PWM note: it is not appropriate to use this 16bit timmer for this purpose
   *    Timer2 | OCR2A                 | #D11 PWM note: it is not appropriate to use this 16bit timmer for this purpose
   *    Timer2 | OCR2B                 | #D3 PWM <-- best choice because when is uset D3 with INT1 for One-Wire pin, then it cannot be used for PWM anyway.
   *    
   * 
   */
  // OW_global setting
  #define OW_ADD_TO_GLOBAL // nothing for this MCU
  //OW Pin  
  // on select:
	    #ifndef OWpin_PD2
	    #ifndef OWpin_PD3
		// in not defined then set defalt OWpin:
            	//#define OWpin_PD2  // OW_PORT Pin 4  - PD2 with interrupt INT0
            	#define OWpin_PD3  // OW_PORT Pin 5  - PD2 with interrupt INT1 <-- best choice with use Timer2 and comparator OCR2A (INT and timer share same pin)
	    #endif
	    #endif
  #ifdef OWpin_PD2
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND2)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT0);EIFR|=(1<<INTF0);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT0);  //disable interrupt
    #define SET_RISING  {EICRA|=(1<<ISC01)|(1<<ISC00);}  //set interrupt at rising edge
    #define SET_FALLING {EICRA|=(1<<ISC01);EICRA&=~(1<<ISC00);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
    #define PIN_INT ISR(INT0_vect)  // the interrupt service routine
  #endif
  #ifdef OWpin_PD3
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND3)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT1);EIFR|=(1<<INTF1);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT1);  //disable interrupt
    #define SET_RISING  {EICRA|=(1<<ISC11)|(1<<ISC10);}  //set interrupt at rising edge
    #define SET_FALLING {EICRA|=(1<<ISC11);EICRA&=~(1<<ISC10);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT1))==(1<<INT1) //test if interrupt enabled
    #define PIN_INT ISR(INT1_vect)  // the interrupt service routine
  #endif
  //Pin setting for all variants
  #define CONF_PORT {OW_DDR&=~OW_PIN_MASK;OW_PORT&=~OW_PIN_MASK;} // set 1-wire pin on start as input
  #define SET_LOW   {OW_DDR|=OW_PIN_MASK;}   //set 1-Wire line to low
  #define RESET_LOW {OW_DDR&=~OW_PIN_MASK;}  //set 1-Wire pin as input
  #define READ_PIN  ((OW_PIN & OW_PIN_MASK) == OW_PIN_MASK) // read from pin and return 0 or 1

  //Timer setting and interrupt
  // on select:
	    #ifndef Timer0_TOV0
	    #ifndef Timer0_OCR0A
	    #ifndef Timer0_OCR0B
	    #ifndef Timer1_OCR1A
	    #ifndef Timer1_OCR1B
	    #ifndef Timer2_OCR2A
	    #ifndef Timer2_OCR2B
		// in not defined then set defalt Timmer:
		//#define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() ) !!!Compile problem inside arduino DO NOT USE (INT is used for time function, allready)
            	//#define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D6 PWM)
            	//#define Timer0_OCR0B // select timer0 with comparator B interrupt (colision with D5 PWM)
            	//#define Timer1_OCR1A // select timer1 with comparator A interrupt (colision with D9 PWM)
            	//#define Timer1_OCR1B // select timer1 with comparator B interrupt (colision with D10 PWM) note: it is not appropriate to use this 16bit timmer for this purpose
            	//#define Timer2_OCR2A // select timer2 with comparator A interrupt (colision with D11 PWM) note: it is not appropriate to use this 16bit timmer for this purpose
            	#define Timer2_OCR2B // select timer2 with comparator B interrupt (colision with D3 PWM) <-- best choice because when is uset D3 with INT1 for One-Wire pin, then it cannot be used for PWM anyway.
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
  #ifdef Timer0_TOV0
    #define CONF_TIMER    {TCCR0A=0 ; TCCR0B=(1<<CS00)|(1<<CS01);} // 8mhz /64 couse 8 bit Timer interrupt every 8us*/ 
    #define EN_TIMER      {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);} //enable timer interrupt
    #define DIS_TIMER     TIMSK0 &= ~(1<<TOIE0); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER0_OVF_vect) //the timer interrupt service routine (TIMER0_OVF_vect get compile error, is used already)
    #define SET_TIMER(x)  TCNT0 = ~( x ) // set Timer0 by the specified value below
  #endif
  #ifdef Timer0_OCR0A
    #define CONF_TIMER    {TCCR0A=0 ; TCCR0B=(1<<CS00)|(1<<CS01);} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK0 |= (1<<OCIE0A); TIFR0|=(1<<OCF0A);} //enable timer interrupt
    #define DIS_TIMER     TIMSK0  &= ~(1<<OCIE0A); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER0_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR0A = TCNT0 +  x // set compare register for new time
  #endif
  #ifdef Timer0_OCR0B
    #define CONF_TIMER    {TCCR0A=0 ; TCCR0B=(1<<CS00)|(1<<CS01);} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK0 |= (1<<OCIE0B); TIFR0|=(1<<OCF0B);} //enable timer interrupt
    #define DIS_TIMER     TIMSK0  &= ~(1<<OCIE0B); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER0_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR0B = TCNT0 +  x // set compare register for new time
  #endif
  #ifdef Timer1_OCR1A
    #define CONF_TIMER    {TCCR1A=0 ; TCCR1B=(1<<CS00)|(1<<CS01); TCCR1C=0 ;} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK1 |= (1<<OCIE1A); TIFR1|=(1<<OCF1A);} //enable timer interrupt
    #define DIS_TIMER     TIMSK1  &= ~(1<<OCIE1A); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER1_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR1A = TCNT1 +  x // set compare register for new time
  #endif
  #ifdef Timer1_OCR1B
    #define CONF_TIMER    {TCCR1A=0 ; TCCR1B=(1<<CS00)|(1<<CS01); TCCR1C=0 ;} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK1 |= (1<<OCIE1B); TIFR1|=(1<<OCF1B);} //enable timer interrupt
    #define DIS_TIMER     TIMSK1  &= ~(1<<OCIE1B); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER1_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR1B = TCNT1 +  x // set compare register for new time
  #endif
  #ifdef Timer2_OCR2A
    #define CONF_TIMER    {TCCR2A=0 ; TCCR2B=(1<<CS22);} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK2 |= (1<<OCIE2A); TIFR2|=(1<<OCF2A);} //enable timer interrupt
    #define DIS_TIMER     TIMSK2  &= ~(1<<OCIE2A); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER2_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR2A = TCNT2 +  x // set compare register for new time
  #endif
  #ifdef Timer2_OCR2B
    #define CONF_TIMER    {TCCR2A=0 ; TCCR2B=(1<<CS22);} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK2 |= (1<<OCIE2B); TIFR2|=(1<<OCF2B);} //enable timer interrupt
    #define DIS_TIMER     TIMSK2  &= ~(1<<OCIE2B); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER2_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR2B = TCNT2 +  x // set compare register for new time
  #endif
  //Analog comparator (for check loss power)
  //If voltage on AIN1 pin will be smaller than internal reference 2,56V then cause interrupt.
  // on select:
	#define Enable_Analog_comapartor
  #ifdef Enable_Analog_comapartor
    #define CONF_AC      {ACSR &= ~(1<<ACD); ACSR |= (1<<ACBG)| (1<<ACI) | (1<<ACIE) | (1<<ACIS1)| (1<<ACIS0); ADCSRB&=~(1<<ACME);} // prepare analog comparator to check loss power
    #define AC_INT       ISR(ANALOG_COMP_vect) // the analog comparator interrupt service routine
  #endif
#endif



#ifdef __AVR_ATmega32U4__
  /* 
   *  Arduino leonardo + arduoino micro
   *  TEST OK
   *  
   *   GPIO FOR OW:
   *   pin n. | AVR label | arduino label | INT  | colision
   *   -----------------------------------------------------
   *    18    | PD0       | D3            | INT0 | OC0B PWM, SCL
   *    19    | PD1       | D2            | INT1 | SDA 
   *    20    | PD2       | D0            | INT2 | RX
   *    21    | PD3       | D1            | INT3 | TX
   *    1     | PE6       | D7            | INT6 | AIN0 = free <-- best choice
   *    
   *    TIMER FOR OW:
   *    timer  | comparator             | colision
   *    -------------------------------------------
   *    Timer0 | TOV0 (timer overflow) | arduino dely(), millis() functions  and #D0 PWM (colision with arduino time functions) DO NOT USE!!!
   *    Timer0 | OCR0A                 | PB7 PWM <-- best choice
   *    Timer0 | OCR0B                 | PD0 PWM
   *    
   *    ATmega32U4 has also 16bit Timer1, Timer3 and 10bit Timer 4, but 8bit Timer0 for OW slave is enough.
   */
  // OW_global setting
  #define OW_ADD_TO_GLOBAL // nothing for this MCU
  //OW Pin  
  // on select:
	    #ifndef OWpin_PD0
	    #ifndef OWpin_PD1
	    #ifndef OWpin_PD2
	    #ifndef OWpin_PD3
	    #ifndef OWpin_PE6
		// in not defined then set defalt OWpin:
            	//#define OWpin_PD0  // OW_PORT Pin 18  - PD0 with interrupt INT0
            	//#define OWpin_PD1  // OW_PORT Pin 19  - PD1 with interrupt INT1
            	//#define OWpin_PD2  // OW_PORT Pin 20  - PD2 with interrupt INT2
            	//#define OWpin_PD3  // OW_PORT Pin 21  - PD3 with interrupt INT3
            	#define OWpin_PE6  // OW_PORT Pin 1  - PE6 with interrupt INT6 <-- best choice with use Timer2 and comparator OCR2A (INT and timer share same pin)
	    #endif
	    #endif
	    #endif
	    #endif
	    #endif
  #ifdef OWpin_PD0 // INT0
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND0)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT0);EIFR|=(1<<INTF0);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT0);  //disable interrupt
    #define SET_RISING  {EICRA|=(1<<ISC01)|(1<<ISC00);}  //set interrupt at rising edge
    #define SET_FALLING {EICRA|=(1<<ISC01);EICRA&=~(1<<ISC00);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
    #define PIN_INT ISR(INT0_vect)  // the interrupt service routine
  #endif
  #ifdef OWpin_PD1 // INT1
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND1)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT1);EIFR|=(1<<INTF1);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT1);  //disable interrupt
    #define SET_RISING  {EICRA|=(1<<ISC11)|(1<<ISC10);}  //set interrupt at rising edge
    #define SET_FALLING {EICRA|=(1<<ISC11);EICRA&=~(1<<ISC10);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT1))==(1<<INT1) //test if interrupt enabled
    #define PIN_INT ISR(INT1_vect)  // the interrupt service routine
  #endif
  #ifdef OWpin_PD2 // INT2
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND2)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT2);EIFR|=(1<<INTF2);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT2);  //disable interrupt
    #define SET_RISING  {EICRA|=(1<<ISC21)|(1<<ISC20);}  //set interrupt at rising edge
    #define SET_FALLING {EICRA|=(1<<ISC21);EICRA&=~(1<<ISC20);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT2))==(1<<INT2) //test if interrupt enabled
    #define PIN_INT ISR(INT2_vect)  // the interrupt service routine
  #endif
  #ifdef OWpin_PD3 // INT3
    //Pin setting 
    #define OW_PORT PORTD //1 Wire Port
    #define OW_PIN PIND //1 Wire Pin as number
    #define OW_DDR DDRD  //pin direction register
    #define OW_PIN_MASK (1<<PIND3)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT3);EIFR|=(1<<INTF3);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT3);  //disable interrupt
    #define SET_RISING  {EICRA|=(1<<ISC31)|(1<<ISC30);}  //set interrupt at rising edge
    #define SET_FALLING {EICRA|=(1<<ISC31);EICRA&=~(1<<ISC30);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT3))==(1<<INT3) //test if interrupt enabled
    #define PIN_INT ISR(INT3_vect)  // the interrupt service routine
  #endif
  #ifdef OWpin_PE6 // INT6
    //Pin setting 
    #define OW_PORT PORTE //1 Wire Port
    #define OW_PIN PINE //1 Wire Pin as number
    #define OW_DDR DDRE  //pin direction register
    #define OW_PIN_MASK (1<<PINE6)  //Pin as bit mask for use in output, input and direction setting registers
    //Pin interrupt 
    #define EN_OWINT {EIMSK|=(1<<INT6);EIFR|=(1<<INTF6);}  //enable interrupt 
    #define DIS_OWINT  EIMSK&=~(1<<INT6);  //disable interrupt
    #define SET_RISING  {EICRB|=(1<<ISC61)|(1<<ISC60);}  //set interrupt at rising edge
    #define SET_FALLING {EICRB|=(1<<ISC61);EICRB&=~(1<<ISC60);} //set interrupt at falling edge
    #define CHK_INT_EN (EIMSK&(1<<INT6))==(1<<INT6) //test if interrupt enabled
    #define PIN_INT ISR(INT6_vect)  // the interrupt service routine
  #endif
  //Pin setting for all variants
  #define CONF_PORT {OW_DDR&=~OW_PIN_MASK;OW_PORT&=~OW_PIN_MASK;} // set 1-wire pin on start as input
  #define SET_LOW   {OW_DDR|=OW_PIN_MASK;}   //set 1-Wire line to low
  #define RESET_LOW {OW_DDR&=~OW_PIN_MASK;}  //set 1-Wire pin as input
  #define READ_PIN  ((OW_PIN & OW_PIN_MASK) == OW_PIN_MASK) // read from pin and return 0 or 1
  //Timer setting and interrupt
  // on select:
	    #ifndef Timer0_TOV0
	    #ifndef Timer0_OCR0A
	    #ifndef Timer0_OCR0B
		// in not defined then set defalt Timmer:
            	// #define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() ) DO NOT USE!!!
            	#define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D6 PWM) <-- best choice
            	//#define Timer0_OCR0B // select timer0 with comparator B interrupt (colision with D5 PWM)
	    #endif
	    #endif
	    #endif
  #ifdef Timer0_TOV0
    #define CONF_TIMER    {TCCR0A=0 ; TCCR0B=(1<<CS00)|(1<<CS01);} // 8mhz /64 couse 8 bit Timer interrupt every 8us*/ 
    #define EN_TIMER      {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);} //enable timer interrupt
    #define DIS_TIMER     TIMSK0 &= ~(1<<TOIE0); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER0_OVF_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  TCNT0 = ~( x ) // set Timer0 by the specified value below
  #endif
  #ifdef Timer0_OCR0A
    #define CONF_TIMER    {TCCR0A=0 ; TCCR0B=(1<<CS00)|(1<<CS01);} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK0 |= (1<<OCIE0A); TIFR0|=(1<<OCF0A);} //enable timer interrupt
    #define DIS_TIMER     TIMSK0  &= ~(1<<OCIE0A); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER0_COMPA_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR0A = TCNT0 +  x // set compare register for new time
  #endif
  #ifdef Timer0_OCR0B
    #define CONF_TIMER    {TCCR0A=0 ; TCCR0B=(1<<CS00)|(1<<CS01);} /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    #define EN_TIMER      {TIMSK0 |= (1<<OCIE0B); TIFR0|=(1<<OCF0B);} //enable timer interrupt
    #define DIS_TIMER     TIMSK0  &= ~(1<<OCIE0B); // disable timer interrupt
    #define TIMER_INT     ISR(TIMER0_COMPB_vect) //the timer interrupt service routine
    #define SET_TIMER(x)  OCR0B = TCNT0 +  x // set compare register for new time
  #endif
#endif

#ifdef __ESP32__
  /*
   * NOT TESTED YET
   *                   ESP32
   *  
   *   
   *                   -------------
   *   GPIO FOR OW: can be use anyvere pin
   *    
   *    TIMER FOR OW: can be use any timer
   */
  // OW_global setting
  #define OW_ADD_TO_GLOBAL {hw_timer_t * timer = NULL; volatile bool pinInt_=1;} // add some global varibale (timer pointer and pin INT status variable)
  //OW Pin  
  // OW_PORT Pin 7  - PB2
  //Pin setting
  #define OW_PORT button1.PIN
  #define CONF_PORT {pinMode(OW_PORT, INPUT);} // set 1-wire pin on start as input
  #define SET_LOW   {pinMode(OW_PORT, OUTPUT);digitalWrite(OW_PORT, 0);}   //set 1-Wire line to low
  #define RESET_LOW {pinMode(OW_PORT, INPUT)}  //set 1-Wire pin as input
  #define READ_PIN  digitalRead(OW_PORT) // read from pin and return 0 or 1
  //Pin interrupt 
  #define EN_OWINT    {attachInterrupt(button1.PIN, isr, FALLING);pinInt_=1;}  //enable interrupt 
  #define DIS_OWINT   {detachInterrupt(button1.PIN);pinInt_=0;}  //disable interrupt
  #define SET_RISING  {detachInterrupt(button1.PIN);attachInterrupt(button1.PIN, isr, RISING;}  //set interrupt at rising edge
  #define SET_FALLING {detachInterrupt(button1.PIN);attachInterrupt(button1.PIN, isr, FALLING;} //set interrupt at falling edge
  #define CHK_INT_EN  (pinInt_==1) //test if interrupt enabled
  #define PIN_INT     void IRAM_ATTR isr()  // the interrupt service routine
  //Timer setting and interrupt
  #ifdef Timer0_TOV0
    #define CONF_TIMER   {timer = timerBegin(0, 500, true);} // (80Mhz of ESP32)/500(prescalar)=4us or 80Mhz/1000=interrupt every 8us*/
    #define EN_TIMER     {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
    #define DIS_TIMER    TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
    #define TIMER_INT    void IRAM_ATTR onTimer() //the timer interrupt service routine
    #define SET_TIMER(x) timer = timerBegin(x, 500, true) // set Timer by the specified value x
  #endif
#endif




#endif

