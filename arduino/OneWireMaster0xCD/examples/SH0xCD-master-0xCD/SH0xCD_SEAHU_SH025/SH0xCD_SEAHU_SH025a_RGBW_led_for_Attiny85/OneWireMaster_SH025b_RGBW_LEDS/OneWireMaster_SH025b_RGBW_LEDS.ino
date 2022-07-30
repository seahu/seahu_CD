#include <OneWire.h>
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
//
// Exmaple description:
// CHANGE COLOR AT LED STRIPS
// ---------------------------
// this file conatain four xempales:
// EXAMPLE 1: simply set only constant color of led strips and white led strip
// EXAMPLE 2: set fluent change color of frist color strip beetwen more colors (sone way can be control RGB2 or WHITE led strip)
// EXAMPLE 3: set repeated fluent change between two colors of led strip (sone way can be control RGB2 or WHITE led strip)
// EXAMPLE 4: set repeated flash between two colors of led strip (sone way can be control RGB2 or WHITE led strip)
// for choice one of those examples simly uncomment/comment definition in setup function
// 
// Type device: 1-Wire
// Family code : 0xCD
// Device description: SEAHU 1-Wire to RGBW C2020 by Ing. Ondrej Lycka
// Number sections: 5
//      section0 :  actually color (as RGBW number in binary format)
//      section1 :  duration in [ms] of change, if >0 duration is time of fluent change form actual to new RGB value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
//      section2 :  number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
//      section3 :  for led indicator - Led indication of device led 1=on 0=off (share same pin as white pwm pin)
//      section4 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)

uint8_t addr[8]; // no need set if use automaticaly find
//uint8_t addr[8]={0xDE,0xB3, 0x5F, 0xDA, 0x02, 0x00, 0x01, 0x7B}; // uncoment if you know 1-wire address
char device_description[64];
int last_position;
uint32_t last_color;
uint8_t R=0;
uint8_t G=0;
uint8_t B=0;

#define SECTION_RGBW          0 // section for RGBW value
#define SECTION_DURATION_RGBW 1 // section for RGBW duration for fluent or strip change color
#define SECTION_REPEATE_RGBW  2 // number of fluent or strip changes beetween actual and new RGBW color


//OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
OneWire  ds(22);  // on pin 10 (a 4.7K resistor is necessary)
OneWireMaster0xCD du(&ds);


/*
 * Find device address
 * if find then set address into global variable "addr" and register "addr" for OneWireMaster0xCD librrary
 * If you do not know device address, you can find it
 */
void find_device() {
  uint8_t i;
  for (;;){ // find right w1 device
    for (;;){
      Serial.println("Search device.");
      if ( ds.search(addr)) break;
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
    }
    Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println();
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      continue;
    }
    if (addr[0]!=0xCD) {
      Serial.println("Device have not damily code 0xCD");
      continue;
    }
    Serial.println("Right family code 0xCD");
    du.setAddr(addr);
    du.get_device_description(device_description);
    Serial.print("Descrition = ");
    Serial.println(device_description);
    //if (strncmp("SEAHU 1-Wire to RGBW C2020 by Ing. Ondrej Lycka",device_description,64)==0) {
    //  Serial.println("Succesfully find right device.");
      break;
    //}
    //else {
    //  Serial.println("This is not correct device.");
    //}
  }
  
}

void setup(void) {
  uint32_t color; // 24-bit value of RGB color

  Serial.begin(9600);
  // set and register addr of device
  find_device(); // autoaticaly find and register device addr
  // or manualy set on top this code and register addr into for OneWireMaster0xCD librrary
  // du.setAddr(addr); // register device

  // EXAMPLE 1: simply set only constant color of led strips and white led strip
  //#define set_color // for disable[enable  this example commnet/uncomment this line
  #ifdef set_color
    // set RGBW
    //color=0xFF000000; // White
    color=0x01000000; // White
    //color=0x00FF0000; // RED
    //color=0x00020000; // RED
    //color=0x0000FF00; // GREEN
    //color=0x00000200; // GREEN
    //color=0x000000FF; // BLUE
    //color=0x00000003; // BLUE
    du.set_actual_value(color,SECTION_RGBW); // RED
  #endif

  // EXAMPLE 2: set fluent change color of frist color strip beetwen more colors (sone way can be control RGB2 or WHITE led strip)
  #define set_fluent // for disable[enable  this example commnet/uncomment this line
  #ifdef set_fluent
    du.set_actual_value(1000,SECTION_DURATION_RGBW); // set 500ms time of fluent duration
    du.set_actual_value(0,SECTION_REPEATE_RGBW); // 0=set no repeation change
    du.set_actual_value(0x00000000,SECTION_RGBW); // set BLACK color
    delay(1000); // wait for fluent fange
    du.set_actual_value(0x00FF0000,SECTION_RGBW); // set RED color
    delay(1000); // wait for fluent fange
    du.set_actual_value(0x0000FF00,SECTION_RGBW); // set GREEN color
    delay(1000); // wait for fluent fange
    du.set_actual_value(0x000000FF,SECTION_RGBW); // set BLUE color
    delay(1000); // wait for fluent fange
    du.set_actual_value(0xFF000000,SECTION_RGBW); // set WHITE color
    delay(1000); // wait for fluent fange

  #endif

  // EXAMPLE 3: set repeated fluent change between two colors of led strip (sone way can be control RGB2 or WHITE led strip)
  //#define set_repeated_fluent_chabge_color // for disable[enable  this example commnet/uncomment this line
  #ifdef set_repeated_fluent_chabge_color
    du.set_actual_value(0,SECTION_DURATION_RGBW); // for start set immediately colors without delay
    du.set_actual_value(0x000000,SECTION_RGBW); // set frist color to BLACK
    du.set_actual_value(0xFF0000,SECTION_RGBW); // set secound color to RED
    du.set_actual_value(500,SECTION_DURATION_RGBW); // set 500ms time of fluent duration change between two colors
    du.set_actual_value(100,SECTION_REPEATE_RGBW); // set 100 cycles of repeat, 0=no repeat, -1=repeat to infinity
  #endif

  // EXAMPLE 4: set repeated flash between two colors of led strip (sone way can be control RGB2 or WHITE led strip)
  //#define set_repeated_flash_color // for disable[enable  this example commnet/uncomment this line
  #ifdef set_repeated_flash_color
    du.set_actual_value(0,SECTION_DURATION_RGBW); // for start set immediately colors without delay
    du.set_actual_value(0x000000,SECTION_RGBW); // set frist color to BLACK
    du.set_actual_value(0xFF0000,SECTION_RGBW); // set secound color to RED
    du.set_actual_value(-500,SECTION_DURATION_RGBW); // set 500ms time of flash between two colors (value <0 means flash and value >0 means fluent change)
    du.set_actual_value(10,SECTION_REPEATE_RGBW); // set 100 cycles of repeat, 0=no repeat, -1=repeat to infinity
  #endif

}

// --- basic example ----- 
void loop(void) {
  //Serial.println("MAIN LOOP");
  // do anything you want
}


