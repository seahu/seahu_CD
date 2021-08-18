#include <OneWire.h>
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
// Example howto use 1-Wire slave device "SEAHU Infrared receiver/transmiter v1 C2021 by Ing. Ondrej Lycka""
// This example show how to receive IR signals. Every detected IR signals will be show on separate line as binary data in hex representation.
// Use one wire library from :
// http://www.pjrc.com/teensy/td_libs_OneWire.html  (most well known library for arduino and control 1-Wire bus)
//
//
// Exmaple description:
// simply transmite buttons codes by 1-Wire bus to simalate push buttons on IR remote controler
// --------------------------------------------------------------------------------------------
// Type device: 1-Wire
// Family code : 0xCD
// Device description: SEAHU Infrared receiver/transmiter v1 C2021 by Ing. Ondrej Lycka
// Number sections: 6
//      section0 :  string buffer of binary data in hexadecimal display (for next data receive or transmit must be previously data deleted by writting binary zero here)
//      section1 :  number of retransmissions
//      section2 :  transmition mode 0=38KHz 1=36KHz
//      section3 :  led indication IR activity 1=enable 0-disable
//      section4 :  control green led 0=off, 1=on
//      section5 :  led idetification of 1-Wire device 1=on 0-off
//      section6 :  always 0  write to 1 to acutal_value start save all  values from all sections to EEPROM
//
// more at:
// https://github.com/seahu/seahu_CD

uint8_t i;
uint8_t addr[8];
char code_on[32]="E66F3EA6410000AAAA2A0080AA00"; // you can get your own  button codes using scanning (uncomnet scaninig in this example)
char code_off[32]="E5713BAB410000AAAA280082AA00";

//uint8_t addr[8]={0xDE,0xB3, 0x5F, 0xDA, 0x02, 0x00, 0x01, 0x7B}; // if you want set 1-wire device address by hand
//#define  OneWireMaster0xCDDebug // for more debug Serial.print outputs

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary) - 1-Wire PIN
OneWireMaster0xCD du(&ds);

void setup(void) {
  Serial.begin(9600);
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
    if (addr[0]==0xCD) break;
    Serial.println("Device is not a UNI MEASURING");
  }
  Serial.println("Device is a UNI MEASURNING");
}

/* tansmite code in chars buffer */
void transmite(char *buf){
  du.setAddr(addr);
  du.set_note("",0); // prevetly reset salve be ready for new value
  du.set_note(buf,0); // reset salve be ready for new value
  delay(10); // leave some time for IR transmit
  du.set_note("",0); // reset salve be ready for new value
}

void loop(void) {
  Serial.println("Sent button on");
  transmite(code_on);
  delay(1000);
  Serial.println("Sent button off");
  transmite(code_off);
  delay(1000);
}
