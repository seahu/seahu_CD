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

   1-Wire to 4x servo interface (using MCU AVR ATtiny85)
   ------------------------------------------------------
   This device enable control up to for digital servos by 1-Wire bus.
   features:
          - drive up four servos
          - enable set rotation speed
          - enable power save mode (off servo after rotation into set angle)
          - enable save actual seting as default seting after power on
    disadvantages:
          - servo signal timing is software management. Small compute cpu performance who
          must serve except servo timing also 1-Wire bus, may be make timing do less precise.
          But affect is barely perceptible.
          
   Device contain next sections:
      section0 :  servo 1 - angle in deg, enabled values is from  0 to 180
      section1 :  servo 1 - rotation speed [deg/s], 0=max servo speed
      section2 :  servo 1 - power save mode 0=Off, 1=On (stop power motor after rotation to set angle)
      section3 :  servo 2 - angle in deg, enabled values is from  0 to 180
      section4 :  servo 2 - rotation speed [deg/s], 0=max servo speed
      section5 :  servo 2 - power save mode 0=Off, 1=On (stop power motor after rotation to set angle)
      section6 :  servo 3 - angle in deg, enabled values is from  0 to 180
      section7 :  servo 3 - rotation speed [deg/s], 0=max servo speed
      section8 :  servo 3 - power save mode 0=Off, 1=On (stop power motor after rotation to set angle)
      section9 :  servo 4 - angle in deg, enabled values is from  0 to 180
      section10:  servo 4 - rotation speed [deg/s], 0=max servo speed
      section11:  servo 4 - power save mode 0=Off, 1=On (stop power motor after rotation to set angle)
      section12:  for all - 1=save actual values as default values (into EEPROM.)
    
   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )

   arduino-AVR_ATtiny85/84:
   ------------------------
   arduino not have native support for AVR_ATtiny85. This supoort have to add :
   the procedure applies for arduino IDE 1.6.4 and hight
   In menu File/Preferences. In the dialg you fill item Additional Board Manager with: https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json a stiskněte OK. Toto jádro je sice uváděny jako první, ale nedoporučuji ho používat. Bylo první z historických důvodů, ale jeho autor nemá na jeho vývoj tolik času, jako je jádro, zmíněné v odstavci o alternativních jádrech.
   And then use function Tools/Board/Boards Manager. In the list find attiny and install it.
   Beafore complile select:
    Board: ATtiny25/45/85
    Procesor: ATtiny85
    Clock: Internal16MHz
    Select Programmer: (I use secundary arduino as ISP "Arduino as ISP"
    Burn Bootloader (this is necessary for set correct MCU clock)
    Burn by Sketch/Upload Using Programmer
   
   More usefull information about ATtiny and arduino:
    https://www.arduinoslovakia.eu/page/attiny85


                                     AVR_ATtiny85   
                                   +------\/-----+ 
                    (RESET) N/U 1 -|1 PB5   VCC 8|- 8 VCC
     SERVO3  <---------- A3  D3 2 -|2 PB3   PB2 7|- 7 D2 A1 (SLC,INT0) ----> 1-Wire
     SERVO4  <--- (OC1B) A2 #D4 3 -|3 PB4   PB1 6|- 6 #D1   (MISO,OC0B,AIN1,OC1A)----> SERVO2
                            GND 4 -|4 GND   PB0 5|- 5 #D0   (MOSI,OC0A,AIN0,SDA,AREF)----> SERVO1
                                   +-------------+               



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
 * This program is writen for AVR_ATtiny85 and for 1-Wire comunication use timer0 with comparatorA therefore 
 * into inside librrary 'OneWireSlave0xCD' in file suportedMCU.h must be comnet/uncomnet lines as showed below
 * check lines:
 * #define __AVR_ATtiny85__
 * .
 * ..
 * .
 *  //Timer setting and interrupt
 *  // on select:
 *      //#define Timer0_TOV0 // select timer0 with overflow interrupt (colision with arduino time functions dealy(), millis() and PWN on D0.D1 with use timer0 (D1 steel can use PWM by timer1, but not from arduino librrary) )
 *      #define Timer0_OCR0A // select timer0 with comparator A interrupt (colision with D0,D1 PWM (D1 steel can use PWM by timer1, but not from arduino librrary))
 *      //#define Timer0_OCR0B // select timer0 with comparator B interrupt (colision with D0,D1 PWM, (D1 steel can use PWM by timer1, but not from arduino librrary))
 *      //#define Timer1_OCR1A // select timmer1 with comparator A interrupt <-- best choice
 *      //#define Timer1_OCR1B // select timmer1 with comparator B interrupt (colision with D4 PWM)
 *      .
 *      ..
 *      .
 *      //Analog comparator (for check loss power)
 *      // on select:
 *       //#define Enable_Analog_comapartor
 */
#include "OneWireSlave0xCD.h"

#define CONT_OF_SECTIONS 13 // number of section (sensors or actors)
#define PIN_SERVO1 0
#define PIN_SERVO2 1
#define PIN_SERVO3 3
#define PIN_SERVO4 4

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
servo servo1,servo2,servo3,servo4;
unsigned long start_cycle;

OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
                                       // "1234567890123456789012345678901234567890123456789012345678901234
const char device_description[] PROGMEM = "SEAHU 1-Wire 4xservo driver C2020 by Ing. Ondrej Lycka          ";
const char servo1_angle[] PROGMEM       = "Angle [deg]{0-180} angle of servo1                              ";
const char servo1_speed[] PROGMEM       = "Speed [deg/s]{0-65535} 0=max speed servo1                       ";
const char servo1_power_save[] PROGMEM  = "Switch {0,1} 0=normal mode 1=power save mode                    ";

const char servo2_angle[] PROGMEM       = "Angle [deg]{0-180} angle of servo2                              ";
const char servo2_speed[] PROGMEM       = "Speed [deg/s]{0-65535} 0=max speed servo2                       ";
const char servo2_power_save[] PROGMEM  = "Switch {0,1} 0=normal mode 1=power save mode                    ";

const char servo3_angle[] PROGMEM       = "Angle [deg]{0-180} angle of servo3                              ";
const char servo3_speed[] PROGMEM       = "Speed [deg/s]{0-65535} 0=max speed servo3                       ";
const char servo3_power_save[] PROGMEM  = "Switch {0,1} 0=normal mode 1=power save mode                    ";

const char servo4_angle[] PROGMEM       = "Angle [deg]{0-180} angle of servo4                              ";
const char servo4_speed[] PROGMEM       = "Speed [deg/s]{0-65535} 0=max speed servo4                       ";
const char servo4_power_save[] PROGMEM  = "Switch {0,1} 0=normal mode 1=power save mode                    ";

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
  // servo3
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo3_angle     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo3_speed     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.u32=0}      , servo3_power_save, NULL                   , false, false },
  // servo4
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo4_angle     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.i32=0}      , servo4_speed     , NULL                   , false, false },
  { C_READ + C_WRITE           + C_32BIT, {.u32=0}      , servo4_power_save, NULL                   , false, false },
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
unsigned int deg_to_val(int deg){
  unsigned long t;
  t=((unsigned long)deg*4000)/1800+1000;
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
  check_new_w1_value(SERVO3_ANGLE, SERVO3_SPEED, &servo3);
  check_new_w1_value(SERVO4_ANGLE, SERVO4_SPEED, &servo4);
}

// calculate new angle and time_of_signal value for specific servo
void calculate_new_servo_values(servo *p_servo, byte section_power_save){
  if (p_servo->actual_angle != p_servo->stop_angle ){
    if ( p_servo->angle_speed > 0) {
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
    //Serial.print("uhel:");
    //Serial.println( p_servo->actual_angle );
    //Serial.print("delka impulsu:");
    //Serial.println( deg_to_val(p_servo->actual_angle) );
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
  calculate_new_servo_values(&servo3, SERVO3_POWER);
  calculate_new_servo_values(&servo4, SERVO4_POWER);
}


/*
 * serve servo signal, servo signal has max 2,5ms in this interval have to serve this signal how fast as it possible
 * after this time function ceck and calculate new values and return. (timing cycle repeat avery 20 ms.)
 * PS: bacause MCU AVR ATtiny85 have not 16 bit timer and system clock timer have not enough precision, then 
 * I software extend 8 bit timer to 16 bit (I cannot use timer interrupt, because 
 * next interrupt serve may be danger slow down the response for 1-Wire signals).
 */
 
void  servo_impuls(){
  byte flag;
  unsigned int counter;
  byte *p_counter_l=(byte*)&counter;
  byte *p_counter_h=p_counter_l+1;
  unsigned int start_counter;
  unsigned int actual;
  bool ser1,ser2,ser3,ser4;
  //int j=400*8;
  int j=1000;
  if ( (micros()-start_cycle) > LENGH_OF_SERVO_CYCLE ) { // end of servor cycle => start new cycle
    
    ser1=true;
    ser2=true;
    ser3=true;
    ser4=true;
    start_cycle=micros();
    //noInterrupts();
    if ( servo1.time_of_signal > 0 ) digitalWrite(PIN_SERVO1,HIGH);
    if ( servo2.time_of_signal > 0 ) digitalWrite(PIN_SERVO2,HIGH);
    if ( servo3.time_of_signal > 0 ) digitalWrite(PIN_SERVO3,HIGH);
    if ( servo4.time_of_signal > 0 ) digitalWrite(PIN_SERVO4,HIGH);
    *p_counter_h=0;
    *p_counter_l=TCNT1;
    //*p_counter_l=0;
    
    flag=0b10000000 & *p_counter_l;
    start_counter=counter;
    //Serial.println(counter);
    //return true;
    while (counter<5000*8){
      actual=counter-start_counter;
      if (actual>servo1.time_of_signal and ser1==true) {digitalWrite(PIN_SERVO1,LOW); ser1=false;};
      if (actual>servo2.time_of_signal and ser2==true) {digitalWrite(PIN_SERVO2,LOW); ser2=false;};
      if (actual>servo3.time_of_signal and ser3==true) {digitalWrite(PIN_SERVO3,LOW); ser3=false;};
      if (actual>servo4.time_of_signal and ser4==true) {digitalWrite(PIN_SERVO4,LOW); ser4=false;};
      *p_counter_l=TCNT1;
      if (flag==0b10000000 && (flag & *p_counter_l)==0) {
        flag=0;
        *p_counter_h=*p_counter_h + 1;
        //Serial.println(*p_counter_h);
      }
      else {
        flag=0b10000000 & *p_counter_l;
      }
      //Serial.println(*p_counter_h);
    }
    //interrupts();
  }
  chcek_new_w1_values();
  calculate_new_servos_values();
  return; // after end of servo signal
}




//------------------------ END SERVO FUNCTIONS --------------------------------------------

// setup
void setup() {
  //Serial.begin(9600);
  pinMode(PIN_SERVO1, OUTPUT);
  pinMode(PIN_SERVO2, OUTPUT);
  pinMode(PIN_SERVO3, OUTPUT);
  pinMode(PIN_SERVO4, OUTPUT);
  ow.ini(CONT_OF_SECTIONS, sections, device_description); // intialization of one wire interface in bacground throught interrupt
  TCCR1=4; // nastavit prescaler na clk/8 tj. 2 taky do 1us
  chcek_new_w1_values();
  calculate_new_servos_values();
  start_cycle=micros();
}

void loop() {
  servo_impuls();
  check_save_values();
}