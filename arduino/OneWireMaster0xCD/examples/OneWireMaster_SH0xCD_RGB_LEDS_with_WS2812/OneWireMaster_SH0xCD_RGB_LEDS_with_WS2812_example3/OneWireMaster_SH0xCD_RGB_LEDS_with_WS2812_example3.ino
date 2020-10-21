#include <OneWire.h>
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
//
// Exmaple description:
// FILL PIXELS COLOR ON RGB LED STRIP with WS2812 leds by 1-Wire bus with device "SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds"
// ----------------------------------------------------------------------------------------------------------------------------------------------
// Type device: 1-Wire
// Family code : 0xCD
// Device description: SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds
// Number sections: 6
//    section0 :  set number of pixels (every LED STIP contain diferent number of pixels, here must be set its number)
//    section1 :  actually color (as RGB number) apply if change color or position
//    section2 :  pixel position (0=all pixels)
//    section3 :  rotation abs(value)=shift rototion, value<0 => left rototion, value>0 => right rotation, value=0 => no rotation (immediately after set will be du and  reset to 0)
//    section4 :  control identification led 0=off, 1=on
//    section5 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)

uint8_t addr[8];
//uint8_t addr[8]={0xDE,0xB3, 0x5F, 0xDA, 0x02, 0x00, 0x01, 0x7B};
char device_description[64];
int last_position;
uint32_t last_color;

//#define  OneWireMaster0xCDDebug

#define NUMPIXELS 60 // define number of pixels on LED STRIP
#define DELAY 20 // delay between steps (speed rotation)
// define significance of sections numbers 
#define SET_NUMPIXELS 0
#define SET_COLOR 1
#define SET_POSITION 2 // section value=0 => selact all pixels
#define SET_ROTATION 3 // section value is pixels shift, value<0 => left rotation, value>0 => right rotation, value=0 => no rotation, 
// communicaton with WS2812 leds is extremely time consuming, therefore during communication with leds device can not service 1-wire bus requirements. For next 1-wire requirement, 
//  we must wait to end communication with leds. This time depends on number of leds (pixels).
#define WIRE_DELAY (2*NUMPIXELS)/30 // delay between 1-Wire commands becouse "SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds" can-not servere 1-wire bus and pixels in one time (value is experimentally found)
// define some color number
#define BLACK 0
#define WHITE 0xFFFFFF
#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
// define position value
#define ALL 0

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
OneWireMaster0xCD du(&ds);

/*
 * Function for seting color on pixel(s)
 * function test last values for prevent make unnecessary communication
 */
void set_color(uint32_t color, int pos, bool force=false){
  // change position
  if (force==true || last_position!=pos){
    du.set_actual_value(pos,SET_POSITION); // set color as RGB (8+8+8bit) number
    delay(WIRE_DELAY);
    last_position=pos;
  }
  // change color
  if (force==true || last_color!=color){
    du.set_actual_value(color,SET_COLOR); // set color as RGB (8+8+8bit) number
    delay(WIRE_DELAY);
    last_color=color;
  }
}

/*
 * Function for seting integer value on 1-Wire device to actual value in specific section
 */
void set_actual_value(int32_t value, uint8_t section_num){
  du.set_actual_value(value,section_num);
  delay(WIRE_DELAY);
}

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
    if (strcmp("SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds",device_description)==0) {
      Serial.println("Succesfully find right device.");
      break;
    }
    else {
      Serial.println("This is not correct device.");
    }
  }
  
}

void setup(void) {
  Serial.begin(9600);
  // set and register addr of device
  find_device(); // autoaticaly find and register device addr
  // or manualy set on top this code and register addr into for OneWireMaster0xCD librrary
  // du.setAddr(addr); // register device

  // default setting
  du.set_actual_value(NUMPIXELS, SET_NUMPIXELS); // nuber of pixels
  // light off all pixels
  set_color(BLACK, ALL, true);
}

 
void loop(void) {
  int i;
  uint8_t present = 0;
  uint8_t number_sections = 0;
  //clear
  set_color(BLACK, ALL);
  // light frist
  set_color(RED, 1);
  Serial.println("MAIN LOOP");
  for (i=1; i<NUMPIXELS+1; i++){
    set_color(RED, i);
    delay(DELAY);
  }
}
