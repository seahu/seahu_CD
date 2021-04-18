/*
   OneWireSlave0xCD - A skeleton 1-Wire multisensor emulator for AVR Microcontroller

   Copyright (C) 2020 Ondrej Lycka info (at) seahu.cz

   The skeleton requres an incllude file with the device-specific dependent command handling.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   1-Wire to 2x servo interface (using MCU AVR ATmega8/16/238)
   ------------------------------------------------------
   This device enable control up to two digital servos by 1-Wire bus.
   features:
          - drive up two servos with excelent timing precision
          - enable set rotation speed
          - enable power save mode (off servo after rotation into set angle)
          - enable save actual seting as default seting after power on
          
   Device contain next sections:
      section0 :  servo 1 - angle in deg, enabled values is from  0 to 180
      section1 :  servo 1 - rotation speed [deg/s], 0=max servo speed
      section2 :  servo 1 - power save mode 0=Off, 1=On (stop power motor after rotation to set angle)
      section3 :  servo 2 - angle in deg, enabled values is from  0 to 180
      section4 :  servo 2 - rotation speed [deg/s], 0=max servo speed
      section5 :  servo 2 - power save mode 0=Off, 1=On (stop power motor after rotation to set angle)
      section6 :  for all - 1=save actual values as default values (into EEPROM.)
    
   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )

  SCHEME:
  -------
  Suported MCU AVR ATmega8,16,328

                                    +-------\/-------+
                      (RESET)      -|1  PC6    PC5 28|- A5   (SCL)   
                      (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)   
                      (TXD)     D1 -|3  PD1    PC3 26|- A3      
                      (INT0)    D2 -|4  PD2    PC2 25|- A2
    !prefered for OW! (INT1) +#D3  -|5  PD3    PC1 24|- A1
                               D4  -|6  PD4    PC0 23|- A0
                                   -|7  VCC    GND 22|-  
                                   -|8  GND   AREF 21|-  
                      (XTAL1) *D20 -|9  PB6   AVCC 20|-  
                      (XTAL2) *D21 -|10 PB7    PB5 19|- D13  (SCK)
                      (T1)   +#D5  -|11 PD5    PB4 18|- D12  (MISO)
                      (AIN0) +#D6  -|12 PD6    PB3 17|- #D11 (MOSI,OC2)
                      (AIN1)   D7  -|13 PD7    PB2 16|- #D10 (OC1B) --> SERVO2
                      (IPC1)   D8  -|14 PB0    PB1 15|- #D9  (OC1A) --> SERVO1
                                    +----------------+
    
    (#  indicates the PWM pins)
    (+# indicates the additional PWM pins on the ATmega168.)
    (*  indicates additional two free pins instead external crystal if is used core: https://github.com/MCUdude/MiniCore this core have more options atc, change clk of MCU)



   SERVO SIGNAL TIMING
   -------------------
       angle = 0 deg                        angle = 90 deg                        angle = 180 deg
       +--+                    +--+         +----+                  +----+        +------+                +------+
       |  |                    |  |         |    |                  |    |        |      |                |      | 
   ----+  +--------------------+  +----- ---+    +------------------+    +--   ---+      +----------------+      +--
   
       |--| 0,5ms                           |----| 1,5 ms                         |------| 2,5 ms
       |-------- 20 ms---------|            |-------- 20 ms---------|             |-------- 20 ms---------|
 
*/

/*
 * !!!! BEFORE COMPILE chek suportedMCU.h !!!!
 * !-----------------------------------------!
 * 1-wire librrary must use another intterupt then servo rutine that use timer1.
 * Please check into inside librrary 'OneWireSlave0xCD' in file suportedMCU.h must be select for corect MCU timer2
 * as show below:
 *   //Timer setting and interrupt
 *  // on select:
 *           //#define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() ) !!!Compile problem inside arduino DO NOT USE (INT is used for time function, allready)
 *           //#define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D6 PWM)
 *           //#define Timer0_OCR0B // select timer0 with comparator B interrupt (colision with D5 PWM)
 *           //#define Timer1_OCR1A // select timer1 with comparator A interrupt (colision with D9 PWM)
 *           //#define Timer1_OCR1B // select timer1 with comparator B interrupt (colision with D10 PWM) note: it is not appropriate to use this 16bit timmer for this purpose
 *           //#define Timer2_OCR2A // select timer2 with comparator A interrupt (colision with D11 PWM) note: it is not appropriate to use this 16bit timmer for this purpose
 *           #define Timer2_OCR2B // select timer2 with comparator B interrupt (colision with D3 PWM) <-- best choice because when is uset D3 with INT1 for One-Wire pin, then it cannot be used for PWM anyway.
 */
#include "OneWireSlave0xCD.h"

#define CONT_OF_SECTIONS 7 // number of section (sensors or actors)
#define PIN_SERVO1 9
#define PIN_SERVO2 10

#define MIN_LENGH_OF_IMPULS 500 // leng of signal in us standard servo 500us=0,5ms => 0 deg
#define MAX_LENGH_OF_IMPULS 2500 // leng of signal in us standard servo 2500us=2,5ms => 180 deg
#define LENGH_OF_SERVO_CYCLE 20000 // leng of whole cycle in us standard servo 20000us=20ms => 50 Hz

// struct for one servo
struct servo {
  unsigned long start_time=0;     // start time of rotation 
  int           start_angle=0;    // start angle [deg x10]
  int           actual_angle=0;   // actual angle [deg x10]
  int           stop_angle=0;     // stop angle [deg x10]
  byte          angle_speed=0;    // rotation spees [deg/s]
  unsigned int  time_of_signal=0; // atual angle calculates as leng of time singnal in clk of hrdware timer1
};

// define global variables and objects
servo servo1,servo2;
unsigned long start_cycle;

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
                                       // "1234567890123456789012345678901234567890123456789012345678901234
const char device_description[] PROGMEM = "SEAHU 1-Wire 2xservo driver C2020 by Ing. Ondrej Lycka          ";
const char servo1_angle[] PROGMEM       = "Angle [deg]{0-180} angle of servo1                              ";
const char servo1_speed[] PROGMEM       = "Speed [deg/s]{0-65535} 0=max speed servo1                       ";
const char servo1_power_save[] PROGMEM  = "Switch {0,1} 0=normal mode 1=power save mode                    ";

const char servo2_angle[] PROGMEM       = "Angle [deg]{0-180} angle of servo2                              ";
const char servo2_speed[] PROGMEM       = "Speed [deg/s]{0-65535} 0=max speed servo2                       ";
const char servo2_power_save[] PROGMEM  = "Switch {0,1} 0=normal mode 1=power save mode                    ";

const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections                 "; // mainly for devolepment
  
//declaration measured functions (this exxample do not use measured functions)

/* 
 * define sections array (this array must be definet as global varialbe (because must be aviable throucht interruot). 
 *        For easy set control register is prepared next constants:
 *        C_READ      - actual value can be read
 *        C_WRITE     - actual value is writable for output devices
 *        C_ALARM     - section have alarm function
 *        C_MIN_ALARM - on min. value alarm  (can be change by One Wire)
 *        C_MAX_ALARM - on max. value alarm  (can be change by One Wire)
 *        C_MEMORY    - section not have value (value is always 0x00), instead it of value is stored into 32B RAM like user description of section
 *        C_BOOL      - section contain bool value
 *        C_U32BIT    - section contain unsigned 32 bit binary value
 *        C_32BIT     - section contain signed 32 bit binary value
*        
 *        (more about control byte (register) refer file OneWireSlave0xCD.h in OneWireSlave0xCD librarry)
 */
section sections[CONT_OF_SECTIONS]={
  // { contril byte os section           , actual value  , descrition      , user description buffer, measure function, lock }
  // servo1
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo1_angle     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo1_speed     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.u32=0}      , servo1_power_save, NULL                   , false, false },
  // servo2
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo2_angle     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo2_speed     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.u32=0}      , servo2_power_save, NULL                   , false, false },
  // save actual value to eeprom
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save           , NULL                   , false, false},
};
typedef enum { SERVO1_ANGLE, SERVO1_SPEED, SERVO1_POWER, SERVO2_ANGLE, SERVO2_SPEED, SERVO2_POWER, SERVO3_ANGLE, SERVO3_SPEED, SERVO3_POWER, SERVO4_ANGLE, SERVO4_SPEED, SERVO4_POWER, SAVE }; // enumerate of intem in section array for clear human read code


//------------------------ MEASURE FUNCTIONS --------------------------
//------------------------------------------------------------------------
/* this example have not measured functions */

//------------------------ END MEASURE FUNCTIONS --------------------------

//------------------------ OTHER FUNCTIONS --------------------------------------------

void check_save_values(){
  union value0xCD newValue;
  newValue.b=0; // prepare new value = zero
  
  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    ow.write_new_value(SAVE, newValue); // section Save off
    ow.save_values();
  }
}
//------------------------ END OTHER FUNCTIONS --------------------------------------------

//------------------------ SERVO FUNCTIONS --------------------------------------------

// calcute degrees value to time of servo impuls in clk of hardware timer1
unsigned int deg_to_val(int deg){ // for beter precision value of angle is in deg x10 (etc. 1800=180[deg])
  unsigned long t;
  // clk=2Mhz => one clock 0.5us range=2000us=>4000tick, 4000tick=180x10degrees, 4000/(180x10)=1degree, X degress= X x 4000/1800, => X x 20/9
  t=((unsigned long)deg*20)/9+1000;
  //Serial.print("mezivypocet uhel:");
  //Serial.println( deg );
  //Serial.print("mezivypocet t:");
  //Serial.println( t );
  return (unsigned int) t;
}

// check new values form 1-Wire bus (for specific servo)
void check_new_w1_value(byte section_angle, byte section_speed, servo *p_servo){
  if ( sections[section_angle].actualValue.i32 > 360 ) sections[section_angle].actualValue.i32=360; // prevention of unreasonably high values
  if ( (unsigned int)sections[section_angle].actualValue.i32*10 != p_servo->stop_angle ) {
    p_servo->start_angle=p_servo->actual_angle;
    p_servo->stop_angle=(unsigned int)sections[section_angle].actualValue.i32*10;
    p_servo->start_time=micros();
    //Serial.println(p_servo->stop_angle);
  }
  if ((unsigned int)sections[section_speed].actualValue.i32!=p_servo->angle_speed){
    p_servo->start_angle=p_servo->actual_angle;
    p_servo->angle_speed=(byte)sections[section_speed].actualValue.i32;
    p_servo->start_time=micros();
  }

}

// check new values form 1-Wire bus (for all servos)
void chcek_new_w1_values(){
  check_new_w1_value(SERVO1_ANGLE, SERVO1_SPEED, &servo1);
  check_new_w1_value(SERVO2_ANGLE, SERVO2_SPEED, &servo2);
}

// calculate new angle and time_of_signal value for specific servo
void calculate_new_servo_values(servo *p_servo, byte section_power_save){
  if (p_servo->actual_angle != p_servo->stop_angle ){
    if ( p_servo->angle_speed > 0) {
      //Serial.println("----------------------------");
      //Serial.print("cas:");
      //Serial.println((micros()- p_servo->start_time));
      //Serial.print("delta uhel:");
      //Serial.println( ((micros()- p_servo->start_time) * p_servo->angle_speed)/100000 );
      if (p_servo->stop_angle > p_servo->start_angle) {
        p_servo->actual_angle=  (unsigned long)(((micros()- p_servo->start_time) * p_servo->angle_speed)/10000 + p_servo->start_angle);
        if ( p_servo->actual_angle > p_servo->stop_angle ) p_servo->actual_angle=p_servo->stop_angle;
      }
      else {
        p_servo->actual_angle=  (unsigned long)(p_servo->start_angle - ((micros()- p_servo->start_time) * p_servo->angle_speed)/10000);
        if ( p_servo->actual_angle < p_servo->stop_angle ) p_servo->actual_angle=p_servo->stop_angle;
      }
    }
    else {
      p_servo->actual_angle = p_servo->stop_angle;
    }
    //Serial.print("novy uhel:");
    //Serial.println( p_servo->actual_angle );
    //Serial.print("delka impulsu:");
    //Serial.println( deg_to_val(p_servo->actual_angle) );
    //Serial.println("----");
    //Serial.print("actual servo 1:");
    //Serial.print(servo1.actual_angle);
    //Serial.print("  stop servo 1:");
    //Serial.print(servo1.stop_angle);
    //Serial.print("  speed servo 1:");
    //Serial.println(servo1.angle_speed);
    
    //Serial.print("actual servo 2:");
    //Serial.print(servo2.actual_angle);
    //Serial.print(" stop servo 2:");
    //Serial.print(servo2.stop_angle);
    //Serial.print(" speed servo 2:");
    //Serial.println(servo2.angle_speed);
    
  }
  else { // servo is in stop_angle
    if ( sections[section_power_save].actualValue.b == 1 ) { // activate power save mode (stop timing) for selected servo
      p_servo->time_of_signal=0;
      return;
    }
  }
  p_servo->time_of_signal=deg_to_val(p_servo->actual_angle);
  return;
}

// calculate new angles and time_of_signal values for all servos
void calculate_new_servos_values(){
  calculate_new_servo_values(&servo1, SERVO1_POWER);
  calculate_new_servo_values(&servo2, SERVO2_POWER);
}


/*
 * I cannot use standard servo librarry becouse use run under interrupt and his serve take time what has negative effect 
 * for serve events on 1-Wire bus. This is a reason why I use hardware pwm pulse. 
 * Hardware pwm pulse have excelent precision, but must use 16 bit timer who is only one.
 * This way I can serve only two servos.
 */
 
void  servo_impuls(){
  if ( (micros()-start_cycle) > LENGH_OF_SERVO_CYCLE ) { // end of servor cycle => start new cycle
    start_cycle=micros();
    chcek_new_w1_values();
    calculate_new_servos_values();
    OCR1A=servo1.time_of_signal;
    OCR1B=servo2.time_of_signal;
  }
}




//------------------------ END SERVO FUNCTIONS --------------------------------------------

// setup
void setup() {
  //Serial.begin(9600);
  pinMode(PIN_SERVO1, OUTPUT);
  pinMode(PIN_SERVO2, OUTPUT);
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
  // prepare 16bit timer1 (write directly into MCU registers)
  // WGM mode of timer set to PWM with top on ICR1
  TCCR1A&=~(1<<WGM10); //WGM10=0
  TCCR1A|= (1<<WGM11); //WGM11=1
  TCCR1B|= (1<<WGM12); //WGM12=1
  TCCR1B|= (1<<WGM13); //WGM13=1
  // coparator A (output start high and go low after compare)
  TCCR1A&=~(1<<COM1A0); //COM1A0=0
  TCCR1A|= (1<<COM1A1); //COM1A1=1
  // coparator B (output start high and go low after compare)
  TCCR1A&=~(1<<COM1B0); //COM1B0=0
  TCCR1A|= (1<<COM1B1); //COM1B1=1
  // clk timer1 => clk/8 (16MHz/8=2MHz one tick=0.5us) (timing range is 2,5ms-0,5ms=2ms => precision is 2000us/0.5us=4000 pulses for 180 degrees => precision 180/4000=0.045 degree
  TCCR1B&=~(1<<CS10); //CS10=0
  TCCR1B|= (1<<CS11); //CS11=1
  TCCR1B&=~(1<<CS12); //CS12=0
  // ICR top value of 16 bit timer for one pwm cycle (servo use 20ms cycyle => 50 Hz, top value => 2000000Hz/50Hz=40000)
  ICR1=40000;
  // start values
  OCR1B=0;
  OCR1A=0;
  // end seting tomer1
  chcek_new_w1_values();
  calculate_new_servos_values();
  start_cycle=micros();
}

void loop() {
  servo_impuls();
  check_save_values();
}
