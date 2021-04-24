//#include <Adafruit_NeoPixel.h>

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

   1-Wire interface for control RGB LED STRIP with WS2812 leds
   -----------------------------------------------------------
   This device enable control WS2812 RGB leds. Control setting light color for selected pixel or all pixels.
   Device contain next sections:
      section0 :  set number of pixels (every LED STIP contain diferent number of pixels, here must be set its number)
      section1 :  actually color (as RGB number in decimal format)
      section2 :  pixel position (0=all pixels)
      section3 :  rotation abs(value)=shift rototion, value<0 => left rototion, value>0 => right rotation, value=0 => no rotation (immediately after set will be du and  reset to 0)
      section4 :  control identification led 0=off, 1=on
      section5 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)

    
   OneWire famyly code: 0xCD (more about devices with tgis family code on https://github.com/seahu/seahu_CD   )



                                          ATMEL ATMEGA328P
                                         +-------\/-------+            
                           (RESET)      -|1  PC6    PC5 28|- A5   (SCL)
                           (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)                                  LED STRIP with WS2812                                           LED STRIP with WS2812
                           (TXD)     D1 -|3  PD1    PC3 26|- A3                                        +--------------------------------------------+                  +--------------------------------------------+
                           (INT0)    D2 -|4  PD2    PC2 25|- A2                          <-------- 5V -|      +--+      +--+      +--+      +--+    |- 5V   <--->  5V -|      +--+      +--+      +--+      +--+    |- 5V
    !prefered for OW! (OC2B,INT1)   #D3 -|5  PD3    PC1 24|- A1  ----------------> A1 <--------   Din -|      |  |      |  |      |  |      |  |    |- Dout <---> Din -|      |  |      |  |      |  |      |  |    |- Dout
                                     D4 -|6  PD4    PC0 23|- A0                          <------  GND -|      +--+      +--+      +--+      +--+    |- GNG  <---> GND -|      +--+      +--+      +--+      +--+    |- GNG
                                        -|7  VCC    GND 22|-                                           +--------------------------------------------+                  +--------------------------------------------+
                                        -|8  GND   AREF 21|-  
                                        -|9  PB6   AVCC 20|-  
                                        -|10 PB7    PB5 19|- D13  (SCK)
            beep <----- (OC0B,T1)   #D5 -|11 PD5    PB4 18|- D12  (MISO)
            led  <--- (OC0A,AIN0)   #D6 -|12 PD6    PB3 17|- #D11 (MOSI,OC2A)
                           (AIN1)    D7 -|13 PD7    PB2 16|- #D10 (OC1B)
                                     D8 -|14 PB0    PB1 15|- #D9  (OC1A)
                                         +----------------+





   
*/



//  generate Device ID based on compile date and time
#include "OneWireSlave0xCD.cpp.h"


// NeoPixcel for WS2812 leds
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


#define COUNT_OF_SECTIONS 6 // number of section (sensors or actors)

#define PIN_DATA   A1
#define PIN_LED  6
#define DEFAULT_NUMPIXELS 60
#define MAX_PIXELS 300 // recommended MCU Atmel AT Mega 328 have only 2KB RAM and Adafruit_NeoPixel library use 3Byte of RAM for ear pixel and aproximately 500B RAM use 1-wire library.

unsigned long last_time=millis();
unsigned long stopLED=0;
uint16_t      last_mumpixels;
uint32_t      last_color;
int           last_position;
int           last_left;
int           last_right;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIN_DATA, NEO_GRB + NEO_KHZ800);
OneWireSlave0xCD ow; //define One Wire object

// factory description is good to store in PROGMEM to save RAM
//                                         1234567890123456789012345678901234567890123456789012345678901234 (max 64B)
const char device_description[] PROGMEM = "SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds";
const char Number_pixels[] PROGMEM      = "Mumber pixels";
const char Actual_color[] PROGMEM       = "RGB actual_color";
const char Pixel_position[] PROGMEM     = "Mumber pixel_position";
const char Rotation[] PROGMEM           = "Mumber rotation_-_left_+_right";
const char Led[] PROGMEM                = "Switch identification_led";
const char Save[] PROGMEM               = "Switch button 1=save_actual_values_all_sections"; // mainly for devolepment

//declaration measured functions 
// void mCounter_a(bool force_now); // not necessary, counter is changing its value continuously alone (in main loop)

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
section sections[COUNT_OF_SECTIONS]={
  // { contril byte os section           , actual value  , descrition      , memory          , measure function , lock }
  { C_READ + C_WRITE +         + C_U32BIT, {.u32=0}      , Number_pixels   , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_U32BIT, {.u32=0}      , Actual_color    , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_U32BIT, {.u32=0}      , Pixel_position  , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_U32BIT, {.u32=0}      , Rotation        , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Led             , NULL            , false            , false},
  { C_READ + C_WRITE +         + C_BOOL  , {.u32=0}      , Save            , NULL            , false            , false},
};
typedef enum { NUMPIXELS, ACTUAL_COLOR, PIXEL_POSITION, ROTATION, LED, SAVE}; // enumerate of intem in section array for clear human read code

//------------------------ MEASURE FUNCTIONS --------------------------
// not necessary
//------------------------ END MEASURE FUNCTIONS --------------------------


//------------------------ SETUP --------------------------------------------
void setup() {
  // setup pins
  // make the pushbutton's pin an input:
  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_LED, OUTPUT);

  // setup One Wire
  ow.ini(COUNT_OF_SECTIONS, sections, device_description); // intialization of one wire interface for run bacground throught interrupt and load saved seting from EEPROM or on frist run set defaut values

  // NeoPixcel for WS2812 leds
  last_mumpixels=(uint16_t) sections[NUMPIXELS].actualValue.u32;
  last_mumpixels=DEFAULT_NUMPIXELS;
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  pixels.updateLength(last_mumpixels);
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code
  pixels.begin(); // This initializes the NeoPixel library.
  
  //setup analog comare for detect poweroff 
  //CONF_AC // (defined in suportedMCU.h)

  Serial.begin(9600);
  Serial.println("start");
  last_color=sections[ACTUAL_COLOR].actualValue.u32;
  last_position=(int)sections[PIXEL_POSITION].actualValue.u32;
  set_color(last_color,last_position);
  
}

//------------------------ END SETUP --------------------------------------------

//------------------------ FUNCTIONS --------------------------------------------
union value0xCD setBollValue(bool value) {
  union value0xCD newValue;
  newValue.b=value;
  return newValue;
}

void set_color(uint32_t color, int pixel_position){
  if (pixel_position>0) pixels.setPixelColor(pixel_position-1, color); // Moderately bright green color.
  else {
    for(int i=0;i<last_mumpixels;i++){
      pixels.setPixelColor(i, color); // Moderately bright green color.
    }
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  
}

void check_color_and_position(){
  if (sections[ACTUAL_COLOR].actualValue.u32 != last_color
      || (int)sections[PIXEL_POSITION].actualValue.u32 != last_position ){ // if chenge color or pixel position
        last_color=sections[ACTUAL_COLOR].actualValue.u32;
        last_position=(int)sections[PIXEL_POSITION].actualValue.u32;
        set_color(last_color, last_position);
      }
}

void check_left_rotation(){
  if (sections[ROTATION].actualValue.i32 < 0 & abs(sections[ROTATION].actualValue.i32)<last_mumpixels-1){
    uint32_t new_color, back_color;
    int shift, i, pos, len;

    len=abs((int)sections[ROTATION].actualValue.i32); // save rotation step
    sections[ROTATION].actualValue.i32=0; // clear section value
    // rotate
    for (shift=0; shift<len; shift++){
      new_color=pixels.getPixelColor(shift);
      for (i=len; i<last_mumpixels; i=i+len){
        pos=i+shift;
        if ( pos>(last_mumpixels-1) ) pos=pos-(last_mumpixels-1);
        back_color=pixels.getPixelColor(pos);
        pixels.setPixelColor(pos, new_color);
        new_color=back_color;
      }
      pixels.setPixelColor(shift, new_color);
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
}

void check_right_rotation(){
  if (sections[ROTATION].actualValue.i32 > 0 & sections[ROTATION].actualValue.i32<last_mumpixels-1){
    uint32_t new_color, back_color;
    int shift, i, pos, len;

    len=(int)sections[ROTATION].actualValue.i32; // save rotation step
    sections[ROTATION].actualValue.i32=0; // clear section value
    // rotate
    for (shift=0; shift<len; shift++){
      new_color=pixels.getPixelColor(last_mumpixels-1-shift);
      for (i=last_mumpixels-1; i>=0; i=i-len){
        pos=i-shift;
        if ( pos<0 ) pos=pos+last_mumpixels-1;
        back_color=pixels.getPixelColor(pos);
        pixels.setPixelColor(pos, new_color);
        new_color=back_color;
      }
      pixels.setPixelColor(last_mumpixels-1-shift, new_color);
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
}


void check_mumpixels(){
  if ((uint16_t)sections[NUMPIXELS].actualValue.u32 != last_mumpixels){ // if chenge mumber pixels
    if ((uint16_t)sections[NUMPIXELS].actualValue.u32 > MAX_PIXELS) sections[NUMPIXELS].actualValue.u32=MAX_PIXELS;
    last_mumpixels=(uint16_t)sections[NUMPIXELS].actualValue.u32;
    pixels.updateLength(last_mumpixels);
  }
}

void check_outputs(){
  if  (sections[LED].actualValue.b==1) digitalWrite(PIN_LED, LOW);
  else digitalWrite(PIN_LED, HIGH);
}

void check_save_valus(){
  // if section save is on
  if (sections[SAVE].actualValue.b==1){ // if setion save is on 
    ow.write_new_value(SAVE, setBollValue(0)); // section Save off
    ow.save_values();
    Serial.println("Values all sections was benn saved.");
  }
}


//------------------------ END FUNCTIONS --------------------------------------------

//------------------------ MAIN LOOP --------------------------------------------- 
void loop() {
  check_color_and_position();
  check_left_rotation();
  check_right_rotation();
  check_outputs();
  check_save_valus();

}
