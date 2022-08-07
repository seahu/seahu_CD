#include <arduino.h>
#include <EEPROM.h>
#include <UIPEthernet.h>
#include <OneWire.h>
#include "serial_config.h"

uint8_t serial_cur;

//--------------------------------------------------------------------
//----------------------- EEPROM functions ---------------------------
//--------------------------------------------------------------------

void save(unsigned int addr, uint8_t buf[], uint8_t len){
  for (uint8_t i=0; i<len ; i++ ){
    EEPROM.update(addr+i, buf[i]);
  }
  EEPROM.update(CHECK_DEFAULT, 0xAA); // flag for detect first save values (for correct values on frist run)
}

void save_myPORT(){
  uint16_t *pointer16=myPORT;
  uint8_t  *pointer8=(uint8_t *)pointer16;
  save(FIRST, pointer8, PORT);
}

void save_myIP(){
  save(FIRST+PORT, myIP, IP);
}

void save_myMASK(){
  save(FIRST+PORT+IP, myMASK, MASK);
}

void save_myGW(){
  save(FIRST+PORT+IP+MASK, myGW, GATEWAY);
}

void save_myDNS(){
  save(FIRST+PORT+IP+MASK+GATEWAY, myDNS, DNS);
}

void save_myDHCP(){
  save(FIRST+PORT+IP+MASK+GATEWAY+DNS, &myDHCP, DHCP);
}

void save_myRFC2217(){
  save(FIRST+PORT+IP+MASK+GATEWAY+DNS+DHCP, &myRFC2217, RFC2217);
}
void save_myMAC(){
  save(FIRST+PORT+IP+MASK+GATEWAY+DNS+DHCP+RFC2217, myMAC, MAC);
}


void load(unsigned int addr, uint8_t buf[], uint8_t len){
  for (uint8_t i=0; i<len ; i++ ){
    buf[i]=EEPROM.read(addr+i);
  }
}

void load_myPORT(){
  uint16_t *pointer16=myPORT;
  uint8_t  *pointer8=(uint8_t *)pointer16;
  load(FIRST, pointer8, PORT);
}

void load_myIP(){
  load(FIRST+PORT, myIP, IP);
}

void load_myMASK(){
  load(FIRST+PORT+IP, myMASK, MASK);
}

void load_myGW(){
  load(FIRST+PORT+IP+MASK, myGW, GATEWAY);
}

void load_myDNS(){
  load(FIRST+PORT+IP+MASK+GATEWAY, myDNS, DNS);
}

void load_myDHCP(){
  load(FIRST+PORT+IP+MASK+GATEWAY+DNS, &myDHCP, DHCP);
}

void load_myRFC2217(){
  load(FIRST+PORT+IP+MASK+GATEWAY+DNS+DHCP, &myRFC2217, RFC2217);
}

void load_myMAC(){
  load(FIRST+PORT+IP+MASK+GATEWAY+DNS+DHCP+RFC2217, myMAC, MAC);
}

//----------------------------------------------------------------------
//---------- serail command line interface functions -------------------
//----------------------------------------------------------------------

/* only one char by one call
 * used dlobal variables:
 * byte serial_cur - count of actualy readed characters in one line
 * 
 * buf[]: pointer where start readed line
 * len:   max character on one line
 * returns: normaly false, true on end of line of overload buffer
 */
bool get_serial_line(char buf[], uint8_t len){
  char value;
  if (Serial.available()>0) {
     value=Serial.read();
     if (value=='\n' ) { // konec radku
      buf[serial_cur]=0x00;
      //Serial.print("ENTER:");
      //Serial.println(serial_cur);
      serial_cur=0;
      return true;
     }
     buf[serial_cur]=value;
     serial_cur++;
     if (serial_cur>=len-1){ // konce serial buferu
      buf[serial_cur]=0x00;
      serial_cur=0;
      //Serial.print("OVERFLOW");
      return true;
    }
  }
  return false;
}

void read_serial_line(char buf[], uint8_t len){ // whole line by one call
    memset(buf,0,len); // null serial buffer
    for(;;){
      if (get_serial_line(buf,len)==true) break;
    }
    Serial.print(F("value: "));
    Serial.println(buf);
}

uint8_t read_serial_bin_number( char buf[] ){ // whole line with number convert to binary nyber by one call
    uint8_t value;
    read_serial_line(buf, SERAIL_BUF_SIZE); 
    value = (uint8_t)strtol(buf, NULL, 10); 
    Serial.print(F("value: "));
    Serial.println(value);
    return value;
}

uint16_t read_serial_bin_big_number( char buf[] ){ // whole line with number convert to binary number by one call
    uint16_t value;
    read_serial_line(buf, SERAIL_BUF_SIZE); 
    value = (uint16_t)strtol(buf, NULL, 10); 
    Serial.print(F("value: "));
    Serial.println(value);
    return value;
}

uint8_t read_serial_bin_hex_number( char buf[] ){ // whole line with hex number convert to binary nyber by one call
    uint8_t value;
    read_serial_line(buf, SERAIL_BUF_SIZE); 
    value = (uint8_t)strtol(buf, NULL, 16); 
    Serial.print(F("value: "));
    Serial.println(value);
    return value;
}


void serial_read_ip( char IPbuf[], char buf[] ){ // postupne nacte vsechny 4 cisla ip (dns, gatway, netmast) adresy v jednom zavalani
  for (uint8_t i=0; i<4; i++){
    Serial.print(F("Number "));
    Serial.print(i);
    Serial.print(F(":"));
    IPbuf[i]= read_serial_bin_number(buf);
  }
}

void serial_read_mac( char MACbuf[], char buf[] ){ // postupne nacte vsech 6 cisl mac adresy v jednom zavalani
  for (uint8_t i=0; i<6; i++){
    Serial.print(F("Number "));
    Serial.print(i);
    Serial.print(F(":"));
    MACbuf[i]= read_serial_bin_number(buf);
  }
}

void serial_print_IP(byte buf[]){ // z binarniho tvaru vypse IP adresu
    for (uint8_t i=0; i<4 ; i++){
      Serial.print(buf[i]);
      if (i!=3) Serial.print(F(".")); 
    }
    Serial.println(F(""));  
}

void serial_print_mac(byte buf[]){ // z binarniho tvaru vypise mac adresu
    for (uint8_t i=0; i<6 ; i++){
      Serial.print(myMAC[i],HEX);
      if (i!=5) Serial.print(F(" ")); 
    }
    Serial.println(F(""));  
}

/* print 1-wire addres (ROM code)
 *  buf[]:  pointer to 1-wire address
 */
void serial_print_ROM(byte buf[]){
  char hex_buf[3]; // for conver hex string
  for ( uint8_t i = 0; i < 8; i++) {
    Serial.print(F(" "));
    sprintf (hex_buf, "%02X", buf[i]);
    Serial.print(hex_buf);
  }
  Serial.println();
}

/* print list addressies of all searched 1-wire devices on 1-wire bus
 *  used gloab variables
 *  uint8_t addr[] - for store 1-wire address
 */
void list_1wire_devices(){
  OneWire  ds(PIN_OW);  // on pin 3 (a 4.7K resistor is necessary)
  uint8_t addr[ROM];
  ds.reset_search();
  for (;;){
    if ( ds.search(addr, true)==false) break;
    serial_print_ROM(addr);
  }
  Serial.println(F("-------"));
}


/* main function for serial command
 *  in normaly try read one character from serial input and end, but when detec end of line evaluate it
 *  and if is need respense than do it.
 *  used global variables:
 *  char serial_buf[] - place for serial data
 *  char server[], char path[], byte ip[], byte subnet[], byte gateway[], byte dns_[], byte dhcp, char lock_addr[]
 */
void serial_command(){
  char  serial_buf[SERAIL_BUF_SIZE];
  if ( get_serial_line(serial_buf, SERAIL_BUF_SIZE)==false ) return;
  Serial.println(serial_buf);
  //memset(serial_buf,0,SERAIL_BUF_SIZE); // null serial buffer
  //return;
  if ( strncmp(serial_buf,"h", SERAIL_BUF_SIZE )==0){
    Serial.println(F("h- help"));
    Serial.println(F("i- show info"));
    Serial.println(F("m- set MAC address"));
    Serial.println(F("p- set server port"));
    Serial.println(F("a- set IP address"));
    Serial.println(F("n- set NETMASK address"));
    Serial.println(F("g- set GATEWAY address"));
    Serial.println(F("d- set DNS address"));
    Serial.println(F("c- set DHCP 0=no, 1=yes"));
    Serial.println(F("l- enable/disable rfc2217"));
    Serial.println(F("W- list 1-wire devices"));
    Serial.println(F(""));
  }
  if ( strncmp(serial_buf,"i", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Saved configuration")); 
    Serial.println(F("-------------------")); 
    Serial.print(F("Saved Port: "));
    Serial.println(myPORT);
    Serial.print(F("Saved IP Address: "));
    serial_print_IP(myIP);
    Serial.print(F("Saved DNS Address: "));
    serial_print_IP(myDNS);
    Serial.print(F("Saved GATEWAY Address: "));
    serial_print_IP(myGW);
    Serial.print(F("Saved SUBNET Address: "));
    serial_print_IP(myMASK);
    Serial.print(F("Saved DHCP 0=no 1=yes: "));
    Serial.println(myDHCP);
    Serial.print(F("Saved enable rfc2217: "));
    Serial.println(myRFC2217);
    Serial.print(F("Saved MAC Address: "));
    serial_print_mac(myMAC);
    Serial.println(F(""));
    Serial.println(F("Actual configuration")); 
    Serial.println(F("--------------------")); 
    Serial.print(F("Actual Port: "));
    Serial.println(myPORT);
    Serial.print(F("Actual IP Address: "));
    Serial.println(Ethernet.localIP());
    Serial.print(F("Actual DNS Address: "));
    Serial.println(Ethernet.dnsServerIP());
    Serial.print(F("Actual GATEWAY Address: "));
    Serial.println(Ethernet.gatewayIP());
    Serial.print(F("Actual NETMASK Address: "));
    Serial.println(Ethernet.subnetMask());
    Serial.println(F("")); 
  }
  if ( strncmp(serial_buf,"m", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set MAC address:"));
    serial_read_mac(myMAC, serial_buf);
    save_myMAC();
  }
  if ( strncmp(serial_buf,"p", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set server port:"));
    myPORT=read_serial_bin_big_number(serial_buf);
    save_myPORT();
  }
  if ( strncmp(serial_buf,"a", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set IP address:"));
    serial_read_ip(myIP, serial_buf);
    save_myIP();
  }
  if ( strncmp(serial_buf,"n", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set NETMASK address:"));
    serial_read_ip(myMASK, serial_buf);
    save_myMASK();
  }
  if ( strncmp(serial_buf,"g", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set GATEWAY address:"));
    serial_read_ip(myGW, serial_buf);
    save_myGW();
  }
  if ( strncmp(serial_buf,"d", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set DNS address:"));
    serial_read_ip(myDNS, serial_buf);
    save_myDNS();
  }
  if ( strncmp(serial_buf,"c", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set DHCP 0=no, 1=yes:"));
    myDHCP=read_serial_bin_number(serial_buf);
    save_myDHCP();
  }
  if ( strncmp(serial_buf,"l", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set enable rfc2217 1=enable 0=disable:"));
    myRFC2217=read_serial_bin_number(serial_buf);
    save_myRFC2217();
  }
  if ( strncmp(serial_buf,"w", SERAIL_BUF_SIZE)==0){
    Serial.println(F("List 1-wire devices:"));
    list_1wire_devices();
  }
  memset(serial_buf,0,SERAIL_BUF_SIZE); // null serial buffer
}

//----------------------------------------------------------------------
//----------------------------- SETUP ----------------------------------
//----------------------------------------------------------------------

void config_setup(void) {
  if (EEPROM.read(CHECK_DEFAULT)!=0xAA){ // first run
    // default values are defined into ONeWireMasterDS24080b_serial_network_emulation.ino (here only save)
    save_myPORT();
    save_myIP();
    save_myMASK();
    save_myGW();
    save_myDNS();
    save_myDHCP();
    save_myRFC2217();
    save_myMAC(); 
  }
  load_myPORT();
  load_myIP();
  load_myMASK();
  load_myGW();
  load_myDNS();
  load_myDHCP();
  load_myRFC2217();
  load_myMAC();
}
