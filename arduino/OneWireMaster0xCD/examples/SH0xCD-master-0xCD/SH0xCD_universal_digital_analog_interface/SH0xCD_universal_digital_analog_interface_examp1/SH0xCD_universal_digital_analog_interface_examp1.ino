#include <OneWire.h>
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
//
// Exmaple description:
// REPATLY SERIAL PRINT ANALOG VALUES FROM SEAHU 1-Wire universal digital-analog board (for visualize analog data in arduino serial ploter console)
// ------------------------------------------------------------------------------------------------------------------------------------------------
// this exaple only print analog values into serial output (every line is one value)
//
// Type device: 1-Wire
// Family code : 0xCD
// Device description: SEAHU 1-Wire universal digital-analog board 1xPIO + 1xIN-ANLOG Ing. Ondrej Lycka
//   Number sections: 8
//      section0 :  read from digital pin
//      section1 :  write into digital pin (digital pin is open drain write to 1 enale pin use also as input)
//      section2 :  digital pin counter (increase with decrease signal edge)
//      section3 :  anlog intup 10-bit resolution 0-5V (0=0V 1023=5V)
//      section4 :  anlog intup correction value 
//      section5 :  identification led 
//      section6 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device loADC default values from another project and switch on beeper)
//      section7 :  reset conter



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
      //Serial.println("Search device.");
      if ( ds.search(addr)) break;
      //Serial.println("No more addresses.");
      //Serial.println();
      ds.reset_search();
      delay(250);
    }
    //Serial.print("ROM =");
    for( i = 0; i < 8; i++) {
      //Serial.write(' ');
      //Serial.print(addr[i], HEX);
    }
    //Serial.println();
    if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      continue;
    }
    else {
      //Serial.println("OK");
      return;
    }
  }
  
}

void setup(void) {
  
  Serial.begin(9600);
  // set and register addr of device
  find_device();
  du.setAddr(addr);
  //Serial.println("Select addr:");
}

 
void loop(void) {
 val analog_value;
 du.get_actual_value(&analog_value, 3);
 Serial.println(analog_value.u32bit);

}
