#include <SPI.h>
#include <UIPEthernet.h>
#include <OneWire.h>
#include <OneWireMaster0xCD.h>
#include <EEPROM.h>

// This program make ethernet interface beatween 1-wire card readers and ethernet entry system.
// Basicly read card codes form 1-wire card reader and ask web service if car code hav access and if yes open door lock.
// Features:
//        enable log and setting (IP seting, wweb path,..) from mini command line by serial console
//
// HW contain:
//   - arduino nano (8bit MCU AVR ATmega328p 32KB FALSH, 2KB RAM)
//   - Ethernet shield for Arduino Nano (ENC28J60), use library UIPEthernet from: https://navody.dratek.cz/docs/texty/0/223/uipethernet.zip 
//                                                           more description on: https://www.tweaking4all.com/hardware/arduino/arduino-enc28j60-ethernet/
//
//                                                                                                     
//                                                                                 +--> +5V
//                                                                                 |
//                                                                                +-+
//                                              ATMEL ATMEGA328P                  | | 2K2
//                                             +-------\/-------+                 | |
// +----------------------+      (RESET)      -|1  PC6    PC5 28|- A5   (SCL)     +-+
// | log and setting      | ---> (RXD)     D0 -|2  PD0    PC4 27|- A4   (SDA)      |
// | serial console       | <--- (TXD)     D1 -|3  PD1    PC3 26|- A3 -------------+-------> OW (master) 
// +----------------------+      (INT0)    D2 -|4  PD2    PC2 25|- A2   
//                          (OC2B,INT1)   #D3 -|5  PD3    PC1 24|- A1   
//                                         D4 -|6  PD4    PC0 23|- A0   
//                                            -|7  VCC    GND 22|-      
//                                            -|8  GND   AREF 21|-  
//                                            -|9  PB6   AVCC 20|-                                 +-----------------+
//                                            -|10 PB7    PB5 19|- D13  (SCK) -------------> SCK <-| Ethernet shield |
//                                        #D5 -|11 PD5    PB4 18|- D12  (MISO) <----------- MISO <-| ENC28J60        |
//                                        #D6 -|12 PD6    PB3 17|- #D11 (MOSI,OC2A) ------> MOSI <-|                 |
//                                         D7 -|13 PD7    PB2 16|- #D10 (OC1B)--------------> SS <-|                 |
//                                         D8 -|14 PB0    PB1 15|- #D9  (OC1A)                     +-----------------+
//                                             +----------------+
//
//
// OneWireMaster0xCD Example
//
// https://github.com/seahu/seahu_CD
// based on  OneWire librarry from http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// This program use next 1-wire card reader:
//   1-Wire interface for RFID CARD READER with RDM6300 module
//   ---------------------------------------------------------
//   This device create interface between 1_Wire bus and RFID card reader (RDM6300 modelue)
//   Except reader intergace this device include control beeper and RGB led includet into card reader + control door lock.
//   This device come from SH0xCD_rfid_reader_Wiegand26 and try keep max. of compatibility. Therefore sections for control red nad blue led on end of list sections.
//   Device description:
//                  SEAHU RFID READER INTERFACE between 1-Wire and RDM6300 module
//   Device contail next section:
//      section0 :  card reader, card code is stored  in user note
//      section1 :  always equal 0. Write to 1 to acutal_value start predefined access allow event (beep, on green led and on door lock for predefinet time, defined in check_start_allow_acceess() code bellow)
//      section2 :  always equal 0. Write to 1 to acutal_value start predefined access deny event (beep for predefinet time, defined in check_start_denny_access() code bellow)
//      section3 :  control green led 0=off, 1=on
//      section4 :  control beep 0=off, other values mean time lenght of sound wave in [us]
//      section5 :  control door lock led 0=close, 1=open
//      section6 :  control red led 0=off, 1=on
//      section7 :  control blue led 0=off, 1=on
//      section8 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)


//#define  OneWireMaster0xCDDebug
//#define  netDebug

//--- setion numbers ----
#define CARD_READER 0
#define ACCESS_ALLOW 1
#define ACCESS_DENNY 2
#define LED_GREEN 3
#define BEEP 4
#define DOOR_LOCK 5
#define LED_RED 6
#define LED_BLUE 7
#define SAVE 8

// definitions and declare variables for configuraion this device
#define SERVER 16
#define PATH 25
#define IP 4
#define SUBNET 4
#define GATEWAY 4
#define DNS 4
#define DHCP 1
#define ROM 8

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[IP] = { 192, 168, 2, 55 };
byte dns_[IP] = { 192, 168, 2, 1 };
byte gateway[IP] = { 192, 168, 2, 1 };
byte subnet[IP] = { 255, 1255, 255, 0 };
char server[SERVER] = { "192.168.2.117"};
char path[PATH] = { "index.php" };
byte dhcp=0; // 0=no (no use) 1=yes (use dhcp)

// 
#define UNLOCK_TIME 5000
#define BUF_LENGHT 64
#define SERAIL_BUF_SIZE 25

char  serial_buf[SERAIL_BUF_SIZE];
byte  serial_cur=0;
char buf[BUF_LENGHT];
uint8_t addr[ROM];
uint8_t lock_addr[ROM];
bool lock_status=false; // true=lock false=unlock
unsigned long time_lock=millis();


EthernetClient client;
OneWire  ds(A3);  // on pin 10 (a 4.7K resistor is necessary)
OneWireMaster0xCD du(&ds);

//----------------------------------------------------------------------
//----------------------- ETHERNET functions ---------------------------
//----------------------------------------------------------------------
bool try_connect(){ // try clinet connect to server with max 10x attempts of succesfully connect control with 0.05s delay beatwin each control
  for (uint8_t i=0;i<10;i++){
    if (client.connected()) {
      #ifdef netDebug
        Serial.println(F("connected"));
      #endif
      return true;
    } else {
      #ifdef netDebug
        Serial.println(F("connection failed"));
      #endif
      delay(50);
    }
  }  
  return false;
}

bool open_connection(){ // try open ethernet (http) connection max 3x attempts
  for (uint8_t i=0;i<3;i++){
    client.connect(server, 80);
    #ifdef netDebug
      Serial.println(F("connecting..."));
    #endif
    if (try_connect()==true) return true;
    client.stop();
  }
  return false;
}

bool get_line(){ // get line from http into global buf
  uint8_t count=0;
  bool result=true;
  for (;;){
    if (count >= BUF_LENGHT-1) {
      #ifdef netDebug
        Serial.println(F("ERROR: overload buf."));
      #endif
      break;
    }
    if (!client.connected()) { //never true
      #ifdef netDebug
        Serial.println(F("Disconnecting."));
      #endif
      client.stop();
      result=false; // end connection = false
      break;
    }
    if (client.available()>0) {
      char c = client.read();
      #ifdef netDebug
        Serial.print(c);
      #endif
      if (c=='\r') continue;
      if (c=='\n' || c=='\r') {
        #ifdef netDebug
          Serial.println(F("End of line."));
        #endif
        break;
      }
      buf[count]=c;
      count++;
    }
  }  
  buf[count]=0x00;
  #ifdef netDebug
    Serial.print(F("Len buf:"));
    Serial.println(strlen(buf));
  #endif
  return result;
}

bool get_data(){ // get http into global buf
  for(;;){
    if (get_line()==false) return false; // connection end beafore data
    if (strlen(buf)==0) break; // end of header
  }  
  get_line(); // next line after header line contain data
  #ifdef netDebug
    Serial.print(F("HTTP DATA:"));
    Serial.println(buf);
  #endif
  return true;
}

//--------------------------------------------------------------------
//----------------------- EEPROM functions ---------------------------
//--------------------------------------------------------------------

void save(unsigned int addr, byte buf[], uint8_t len){
  for (uint8_t i=0; i<len ; i++ ){
    EEPROM.update(addr+i, buf[i]);
  }  
}

void save_server(){
  save(0, server, SERVER);
}

void save_path(){
  save(0+SERVER, path, PATH);
}

void save_ip(){
  save(0+SERVER+PATH, ip, IP);
}

void save_subnet(){
  save(0+SERVER+PATH+IP, subnet, SUBNET);
}

void save_gateway(){
  save(0+SERVER+PATH+IP+SUBNET, gateway, GATEWAY);
}

void save_dns(){
  save(0+SERVER+PATH+IP+SUBNET+GATEWAY, dns_, DNS);
}

void save_dhcp(){
  save(0+SERVER+PATH+IP+SUBNET+GATEWAY+DNS, &dhcp, DHCP);
}

void save_lock_addr(){
  save(0+SERVER+PATH+IP+SUBNET+GATEWAY+DNS+DHCP, lock_addr, ROM);
}


void load(unsigned int addr, byte buf[], uint8_t len){
  for (uint8_t i=0; i<len ; i++ ){
    buf[i]=EEPROM.read(addr+i);
  }
}

void load_server(){
  load(0, server, SERVER);
}

void load_path(){
  load(0+SERVER, path, PATH);
}

void load_ip(){
  load(0+SERVER+PATH, ip, IP);
}

void load_subnet(){
  load(0+SERVER+PATH+IP, subnet, SUBNET);
}

void load_gateway(){
  load(0+SERVER+PATH+IP+SUBNET, gateway, GATEWAY);
}

void load_dns(){
  load(0+SERVER+PATH+IP+SUBNET+GATEWAY, dns_, DNS);
}

void load_dhcp(){
  load(0+SERVER+PATH+IP+SUBNET+GATEWAY+DNS, &dhcp, DHCP);
}

void load_lock_addr(){
  load(0+SERVER+PATH+IP+SUBNET+GATEWAY+DNS+DHCP, lock_addr, ROM);
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

uint8_t read_serial_bin_number(){ // whole line with number convert to binary nyber by one call
    uint8_t value;
    read_serial_line(serial_buf, SERAIL_BUF_SIZE); 
    value = (uint8_t)strtol(serial_buf, NULL, 10); 
    Serial.print(F("value: "));
    Serial.println(value);
    return value;
}

uint8_t read_serial_bin_hex_number(){ // whole line with hex number convert to binary nyber by one call
    uint8_t value;
    read_serial_line(serial_buf, SERAIL_BUF_SIZE); 
    value = (uint8_t)strtol(serial_buf, NULL, 16); 
    Serial.print(F("value: "));
    Serial.println(value);
    return value;
}


void serial_read_ip(byte buf[]){ // postupne nacte vsechny 4 cisla ip (dns, gatway, netmast) adresy v jednom zavalani
  for (uint8_t i=0; i<4; i++){
    Serial.print(F("Number "));
    Serial.print(i);
    Serial.print(F(":"));
    buf[i]= read_serial_bin_number();
  }
}

void serial_read_ROM(byte buf[]){ // postupne nacte vsech 8 hex cisel ROM adresy  v jednom zavalani
  for (uint8_t i=0; i<8; i++){
    Serial.print(F("Number "));
    Serial.print(i);
    Serial.print(F(":"));
    buf[i]= read_serial_bin_hex_number();
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
      Serial.print(mac[i],HEX);
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
  if ( get_serial_line(serial_buf, SERAIL_BUF_SIZE)==false ) return;
  Serial.println(serial_buf);
  //memset(serial_buf,0,SERAIL_BUF_SIZE); // null serial buffer
  //return;
  if ( strncmp(serial_buf,"h", SERAIL_BUF_SIZE )==0){
    Serial.println(F("h- help"));
    Serial.println(F("s- set server IP adres"));
    Serial.println(F("p- set server path"));
    Serial.println(F("i- show info"));
    Serial.println(F("a- set IP address"));
    Serial.println(F("n- set SUBNET address"));
    Serial.println(F("g- set GATEWAY address"));
    Serial.println(F("d- set DNS address"));
    Serial.println(F("c- set DHCP 0=no, 1=yes"));
    Serial.println(F("l- set door lock 1-wire addres"));
    Serial.println(F("W- list 1-wire devices"));
    Serial.println(F(""));
  }
  if ( strncmp(serial_buf,"i", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Saved configuration")); 
    Serial.println(F("-------------------")); 
    Serial.print(F("Saved IP Address: "));
    serial_print_IP(ip);
    Serial.print(F("Saved DNS Address: "));
    serial_print_IP(dns_);
    Serial.print(F("Saved GATEWAY Address: "));
    serial_print_IP(gateway);
    Serial.print(F("Saved SUBNET Address: "));
    serial_print_IP(subnet);
    Serial.print(F("Saved DHCP 0=no 1=yes: "));
    Serial.println(dhcp);
    Serial.println(F("---")); 
    Serial.print(F("Saved Server IP fro connection: "));
    Serial.println(server);
    Serial.print(F("Saved URL path connection: "));
    Serial.println(path);
    Serial.print(F("Saved door lock 1-wire addres: "));
    serial_print_ROM(lock_addr);
    Serial.println(F(""));
    Serial.println(F("Actual configuration")); 
    Serial.println(F("--------------------")); 
    Serial.print(F("Actual IP Address: "));
    Serial.println(Ethernet.localIP());
    Serial.print(F("Actual DNS Address: "));
    Serial.println(Ethernet.dnsServerIP());
    Serial.print(F("Actual GATEWAY Address: "));
    Serial.println(Ethernet.gatewayIP());
    Serial.print(F("Actual SUBNET Address: "));
    Serial.println(Ethernet.subnetMask());
    Serial.print(F("Actual MAC Address: "));
    serial_print_mac(mac);
    Serial.println(F("")); 
  }
  if ( strncmp(serial_buf,"s", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set server IP adres:"));
    read_serial_line(server, SERVER);
    save_server();
  }
  if ( strncmp(serial_buf,"p", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set server path:"));
    read_serial_line(path, PATH);
    save_path();
  }
  if ( strncmp(serial_buf,"a", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set IP address:"));
    serial_read_ip(ip);
    save_ip();
  }
  if ( strncmp(serial_buf,"n", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set SUBNET address:"));
    serial_read_ip(subnet);
    save_subnet();
  }
  if ( strncmp(serial_buf,"g", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set GATEWAY address:"));
    serial_read_ip(gateway);
    save_gateway();
  }
  if ( strncmp(serial_buf,"d", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set DNS address:"));
    serial_read_ip(dns_);
    save_dns();
  }
  if ( strncmp(serial_buf,"c", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set DHCP 0=no, 1=yes:"));
    dhcp=read_serial_bin_number();
    save_dhcp();
  }
  if ( strncmp(serial_buf,"l", SERAIL_BUF_SIZE)==0){
    Serial.println(F("Set door lock 1-wire addres:"));
    serial_read_ROM(lock_addr);
    save_lock_addr();
  }
  if ( strncmp(serial_buf,"w", SERAIL_BUF_SIZE)==0){
    Serial.println(F("List 1-wire devices:"));
    list_1wire_devices();
  }
  memset(serial_buf,0,SERAIL_BUF_SIZE); // null serial buffer
}


//----------------------------------------------------------------------
//-------------------- CARD ACCES FUNCTION -----------------------------
//----------------------------------------------------------------------

/* auxiliary function for check card by ethernet
 *  used global ariables:
 *  char card_code[] - global store card code
 *  char server[] - IP server with card acces control serice
 *  char path[] - http path where run card access control service
 *  EthernetClient client - ethernet instancion
 *  return:
 *  true - allow access
 *  false - deny access
 */
bool net_check_card(){ 
  char hex_buf[3];

  if (open_connection()==false) {
    Serial.println(F("FAILED HTTP CONNECTION"));
    return false; // try start connection
  }
  client.print(F("GET /"));
  client.print(path);
  client.print(F("?q=check&reader="));
  for ( uint8_t i = 0; i < 8; i++) {
    sprintf (hex_buf, "%02X", addr[i]);
    client.print(hex_buf);
  }
  client.print(F("&card_code="));
  client.print(buf);
  client.println(F(" HTTP/1.0"));
  client.println();
  if (get_data()==false) return false; // can not get data to global buf
  Serial.print(F("HTTP DATA:"));
  Serial.println(buf);
  if (strcmp(buf,"y")==0) return true;
  return false;
}

/* main functio fro check cad code, red card code from 1-wire device, check acces and if open door lock
 *  used global variables:
 *  OneWireMaster0xCD du - 1-wire instance for 1-wire deveces with family code 0xCD
 *  unsigned long time_lock - time when lock will be unlock (for later lock)
 *  bool lock_status
 */
void check_card() { // use global addr
  bool net_result;

  //Serial.println("CHECK CARD");
  du.setAddr(addr);
  du.get_note(buf,0);
  Serial.print(F("Card code: "));
  Serial.println(buf);
  // compare buf vs net
  net_result=net_check_card();
  memset(buf,0x00,64);
  du.set_note(buf,0);
  if(net_result==true) {
    du.set_actual_value(1, ACCESS_ALLOW); // spust povolovaci sekvenci na ctece
    lock_status=false; // open
    time_lock=millis();
    du.setAddr(lock_addr);
    du.set_actual_value(1, DOOR_LOCK); // ctecka nemusi myt spojena se yamkem, tak pro jistotu odemkni zamek
    Serial.println(F("ACCESS_ALLOW"));
  }
  else {
    du.set_actual_value(1, ACCESS_DENNY);
    Serial.println(F("ACCESS_DENNY"));
  }
}

/* function for delay door lock
 *  used global variables:
 *  unsigned long time_lock - time when lock will be unlock (for later lock)
 *  bool lock_status
 */
void check_end_lock(){
  if ( lock_status==true) {
    if ( abs(millis()-time_lock) > UNLOCK_TIME ) {
      du.setAddr(lock_addr);
      du.set_actual_value(0, DOOR_LOCK); // unlock
    }
  }
}


//----------------------------------------------------------------------
//----------------------------- SETUP ----------------------------------
//----------------------------------------------------------------------

void setup(void) {
  Serial.begin(9600);
  load_server();
  load_path();
  load_ip();
  load_subnet();
  load_gateway();
  load_dns();
  load_dhcp();
  load_lock_addr();
  if (dhcp==1)  Ethernet.begin(mac);
  else          Ethernet.begin(mac, ip, dns_, gateway, subnet);
  ds.reset_search();
  Serial.println(F("START"));
  Serial.println(F("h - for help"));
}

//----------------------------------------------------------------------
//----------------------------- MAIN LOOP-------------------------------
//----------------------------------------------------------------------

void loop(void) {
  uint8_t present = 0;
  uint8_t number_sections = 0;
  //Serial.println("MAIN LOOP");

  serial_command();
  check_end_lock(); // zkontroluj cas vypnuti zamku

  //Search 1-wire device witch alarm status
  if ( ds.search(addr, false)==false) {
    // No more addresses
    ds.reset_search();
    delay(50);
    return;
  }
  // print ROM addr
  Serial.println(F("-------"));
  Serial.print(F("1-Wire addr reader (ROM): "));
  serial_print_ROM(addr);
  // test crc
  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println(F("CRC is not valid!"));
    return;
  }
  if (addr[0] == 0xCD) check_card();
  else Serial.println(F("Device is not a UNI MEASURING"));
}
