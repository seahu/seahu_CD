#include <OneWire.h>
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
//
// Exmaple description:
// CHANGE ALL PIXELS COLOR ON RGB LED STRIP with WS2812 leds by 1-Wire bus with device "SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds"
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

byte addr[10];


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
    else {
      Serial.println("OK");
      return;
    }
  }
  
}

void setup(void) {
  Serial.begin(9600);
  // set and register addr of device
  find_device();
  du.setAddr(addr);
  Serial.println("Select addr");
  du.set_actual_value(500,0);
  Serial.println("Senf section 0 to 500");
  //du.set_actual_value(value,section_num);
}

 
void loop(void) {
 //find_device();
}
