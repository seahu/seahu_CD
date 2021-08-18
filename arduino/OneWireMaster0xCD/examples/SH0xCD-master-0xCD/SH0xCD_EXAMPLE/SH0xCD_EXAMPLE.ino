#include <OneWire.h>
#include <OneWireMaster0xCD.h>

// OneWireMaster0xCD Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library
uint8_t i;
uint8_t controlByte=1;
uint8_t data[12];
uint8_t addr[8];
//uint8_t addr[8]={0xDE,0xB3, 0x5F, 0xDA, 0x02, 0x00, 0x01, 0x7B};
bool ini;

#define  OneWireMaster0xCDDebug


val actualValue;


OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
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
 
void loop(void) {
  uint8_t present = 0;
  uint8_t number_sections = 0;
  Serial.println("MAIN LOOP");
  /*
  // find devices in conditional search (in alarm= new card is ready)
  if ( !ds.search(addr, false)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(100);
    return;
  }
  */
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
  if (addr[0]!=0xCD){
    Serial.println("No Chip UNI MEASURNING");
    return;
  }
  Serial.println("Is Chip UNI MEASURNING");
  Serial.println("-------------------------------------------------");

  du.setAddr(addr);
  number_sections = du.get_number_of_sections();
  Serial.print("Number of sections: ");
  Serial.println(number_sections);
  
  char buf[64];
  for (uint8_t i=0; i<number_sections; i++){
    Serial.print("SECTION: ");
    Serial.println(i);
    delay(10);
    du.setSection(i);
    delay(10);
    du.get_description(buf);
    Serial.print(" Description: ");
    Serial.println(buf);
    delay(10);
    du.get_status_byte();
    delay(10);
    du.start_measure();
    delay(10);
    du.get_measured_value(&actualValue);
    delay(10);
    du.print_value(&actualValue," Measured value: ");
    du.set_note("jidelna");
    delay(10);
    du.get_note(buf);
    Serial.print(" Note: ");
    Serial.println(buf);

  }
  
  //get_number_of_sections();
  //select_sections(1);
  //get_status_byte();
  //actualValue.u32bit=232;
  //set_actual_value(&actualValue);
  //get_actual_value(&actualValue);
  //set_beep(2000, 100);
  //set_status_byte(0, GREEN_LED_MASK);
  //set_status_byte(0, 0);
  delay(10000);
  //familyDE_red_card_code();

}