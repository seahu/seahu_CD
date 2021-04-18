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
// simply recivre IR remote buttons codes by 1-wire bus
// ----------------------------------------------------
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
uint8_t alarm_addr[8];
char buf[32];

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

// suitable if on bus is more devices and more devices need serve fast response to alarm status
void wait_to_new_signal_by_alarm_status(char *buf){
  // find devices in conditional search (in alarm= new code signal is ready)
  for (;;){ // find alarm w1 device
    if ( !ds.search(alarm_addr, false)) {
      ds.reset_search();
      delay(100);
      return;
    }
    if ( memcmp(addr, alarm_addr, 8)==0 ) break;
  }
}

// suitable for bus with less count devices who need fast response
void wait_to_new_signal_by_repeatly_read_data(char *buf){
  for (;;){
    du.setAddr(addr);
    du.get_note(buf,0);
    if (buf[0]!=0) { // clear note to prepare space for new scan
      du.set_note("",0);
      return;
    }
  }
}


void loop(void) {
  wait_to_new_signal_by_alarm_status(buf); // you can use bougth of scan metods simply commnet or uncoment there
  //wait_to_new_signal_by_repeatly_read_data(buf);
  du.setAddr(addr);
  du.get_note(buf,0);
  du.set_note("",0);
  if (buf[0]!=0) {
    Serial.print(" Received code: ");
    Serial.println(buf);
    buf[0]=0; // avoid repeate print the same value
  }
  delay(10);
}
