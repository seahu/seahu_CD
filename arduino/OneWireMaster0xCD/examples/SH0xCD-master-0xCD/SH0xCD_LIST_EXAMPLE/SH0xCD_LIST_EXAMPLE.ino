#include <OneWire.h>
#include <OneWireMaster0xCD.h>


// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
// based on  OneWire librarry from http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// Exmaple description:
// LIST ON SERIAL PORT ALL 1-Wire DEVICES AND FOR DEVICE WITH FAMY CODE 0xCD SHOW ALL HIS PROTERTIES
// -------------------------------------------------------------------------------------------------
// Type device: 1-Wire
// Family code : 0xCD
// Device description: for all SEAHU 1-Wire devices with family code 0xCD
// Script show:
//    -device ROM address
//    -device desription
//    -number of sections
//      on sections:
//        -section description
//        -section user note
//        -section control byte
//        -section min. value for alarm triger
//        -section max. value for alarm triger

uint8_t i;
uint8_t controlByte = 1;
uint8_t data[12];
uint8_t addr[8];
//uint8_t addr[8]={0xDE,0xB3, 0x5F, 0xDA, 0x02, 0x00, 0x01, 0x7B};
bool ini;

#define  OneWireMaster0xCDDebug

OneWire  ds(22);  // on pin 10 (a 4.7K resistor is necessary)
OneWireMaster0xCD du(&ds);


void setup(void) {
  Serial.begin(9600);
  for (;;) { // find right w1 device
    for (;;) {
      Serial.println("Search device.");
      if ( ds.search(addr)) break;
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
    }
    Serial.print("ROM =");
    for ( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println();
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      continue;
    }
    if (addr[0] == 0xCD) break;
    Serial.println("Device is not a UNI MEASURING");
  }
  Serial.println("Device is a UNI MEASURNING");
}

void seahu0xCDinfo() {
  char buf[64];
  uint8_t number_sections = 0;
  uint8_t controlByte;
  val actualValue;
  val alarmValue;

  du.setAddr(addr);
  // device description
  du.get_device_description(buf);
  Serial.print("Device description: ");
  Serial.println(buf);
  // number sections
  number_sections = du.get_number_of_sections();
  Serial.print("Number of sections: ");
  Serial.println(number_sections);
  // sections info
  for (uint8_t i = 0; i < number_sections; i++) {
    // section number
    Serial.print(" SECTION: ");
    Serial.println(i);
    // sectin description
    du.get_description(buf, i);
    Serial.print("  Description: ");
    Serial.println(buf);
    // user note
    du.get_note(buf);
    Serial.print("  User note: ");
    Serial.println(buf);
    // control byte
    controlByte = du.get_control_byte(i);
    Serial.println                          ("  Control byte: | read  | write | exist | value | status |    set    |    set    |");
    Serial.println                          ("                |       |       | alarm | type  |  alarm | min alarm | max alarm |");
    if (controlByte & C_READ )  Serial.print("                |  Y    |");
    else                        Serial.print("                |  N    |");
    if (controlByte & C_WRITE )                          Serial.print("  Y    |");
    else                                                 Serial.print("  N    |");
    if (controlByte & C_ALARM )                                  Serial.print("  Y    |");
    else                                                         Serial.print("  N    |");
    if ((controlByte & C_TYPE_MASK ) == C_MEMORY )                        Serial.print("memory |");
    if ((controlByte & C_TYPE_MASK ) == C_BOOL )                          Serial.print(" bool  |");
    if ((controlByte & C_TYPE_MASK ) == C_U32BIT )                        Serial.print("uint32 |");
    if ((controlByte & C_TYPE_MASK ) == C_32BIT )                         Serial.print(" int32 |");
    if (controlByte & C_ALARM ) {
      if (controlByte & C_ALARM_STATUS )                                         Serial.print("   Y    |");
      else                                                                       Serial.print("   N    |");

      if (controlByte & C_MIN_ALARM )                                                      Serial.print("     Y     |");
      else                                                                                 Serial.print("     N     |");
      if (controlByte & C_MAX_ALARM )                                                               Serial.println("     Y     |");
      else                                                                                          Serial.println("     N     |");
    }
    else {
      Serial.print("  ---   |");
      Serial.print("    ---    |");
      Serial.println("    ---    |");
    }
    // actual value
    du.start_measure();
    delay(10);
    du.get_measured_value(&actualValue);
    //du.get_actual_value(&actualValue);
    du.print_value(&actualValue, "  Actual value: ");
    if (controlByte & C_ALARM ) {
      // triger min. alarm value
      du.get_min_alarm_value(&alarmValue, i);
      du.print_value(&alarmValue, "  Triger min. alarm value: ");
      // triger min. alarm value
      du.get_max_alarm_value(&alarmValue, i);
      du.print_value(&alarmValue, "  Triger max. alarm value: ");
    }
  }
  delay(5000);
}


void loop(void) {
  uint8_t present = 0;
  uint8_t number_sections = 0;
  Serial.println("MAIN LOOP");

  for (;;) {
    Serial.println("Search device.");
    if ( ds.search(addr)) break;
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
  }
  // print ROM addr
  Serial.println("-------------------------------------------------");
  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  Serial.println();
  // test crc
  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  if (addr[0] == 0xCD) seahu0xCDinfo();
  else Serial.println("Device is not a UNI MEASURING");
}
