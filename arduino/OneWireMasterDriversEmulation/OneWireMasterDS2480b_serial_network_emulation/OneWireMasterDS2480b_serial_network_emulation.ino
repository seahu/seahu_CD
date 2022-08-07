/*
 * UIPEthernet EchoServer example.
 *
 * UIPEthernet is a TCP/IP stack that can be used with a enc28j60 based
 * Ethernet-shield.
 *
 * UIPEthernet uses the fine uIP stack by Adam Dunkels <adam@sics.se>
 *
 *      -----------------
 *
 * This Hello World example sets up a server at 192.168.1.6 on port 1000.
 * Telnet here to access the service.  The uIP stack will also respond to
 * pings to test if you have successfully established a TCP connection to
 * the Arduino.
 *
 * This example was based upon uIP hello-world by Adam Dunkels <adam@sics.se>
 * Ported to the Arduino IDE by Adam Nielsen <malvineous@shikadi.net>
 * Adaption to Enc28J60 by Norbert Truchsess <norbert.truchsess@t-online.de>
 */

#include <UIPEthernet.h>
// The connection_data struct needs to be defined in an external file.
#include <UIPServer.h>
#include <UIPClient.h>
#include "nvt.h" // nvt.h include sd2480b.h and serial_config.h

#define VERSION "ds2480b network emulation v1.0"

// deklarace funkci ktere pouzuji v kodu drive nez jsou napsane (kvuli kompilatoru)
//bool nvt(byte data);
//void do_stream(byte data);
//void big_reset();

uint8_t a;



uint16_t myPORT = 1000;
uint8_t mac[6] = {MACADDRESS};
uint8_t myMAC[6] = {MACADDRESS};
uint8_t myIP[4] = {MYIPADDR};
uint8_t myMASK[4] = {MYIPMASK};
uint8_t myDNS[4] = {MYDNS};
uint8_t myGW[4] = {MYGW};
uint8_t myDHCP = 0;
uint8_t myRFC2217 = 1;


//EthernetServer server_ = EthernetServer(LISTENPORT);
EthernetClient client_; // globalni adresa pro komunikaci se spojenim

void setup() {
  Serial.begin(9600);
  Serial.println(VERSION);
  config_setup();
  ds2480b_setup();
  // initialize the ethernet device
  if (myDHCP==1)  Ethernet.begin(mac);
  else          Ethernet.begin(mac, myIP, myDNS, myGW, myMASK);
  // start listening for clients
  //server_ = EthernetServer(LISTENPORT);
  //server_.begin();
}

void loop() {
  size_t size;
  // I must start server after reading port in setup procedure therefore I created here secound loop()
  EthernetServer server_ = EthernetServer(myPORT);
  server_.begin();
  while (true) { // secound loop
    if (client_ = server_.available())
      {
        if (client_)
          {
            while((size = client_.available()) > 0)
              {
                uint8_t* msg = (uint8_t*)malloc(size);
                size = client_.read(msg,size);
                #ifdef DEBUG_DS2480B // for debug not use telnet options (definition DEBUG_DS2480B is set in file ds2480.h)
                  hex_numbr_msg_to_bin_streem(msg, size);
                #endif
                #ifndef DEBUG_DS2480B
                  for ( size_t i=0; i<size; i++){
                    if (myRFC2217==1){
                      if ( nvt( msg[i] )==true ) { // zkontroluj zaslany byte jestli se jedna o data nebo volby nastaveni dle rfc2217
                        //Serial.print("ZAPIS: ");
                        //Serial.println(msg[i], HEX);
                        do_stream(msg[i]); // o odpoved se do_stream postra sam diky global promenne EthernetClient client vi kam ma odpoved poslat
                        // kdyy zapisu tak se pokusim i precist odpoved (bud je nebo neni)
                      }
                    }
                    else { // nic neontroluj jen primo posilej
                      do_stream(msg[i]); // o odpoved se do_stream postra sam diky global promenne EthernetClient client vi kam ma odpoved poslat
                    }
                  }
                #endif
                //client.write(msg,size);
                free(msg);
              }
          }
      }
    serial_command();
      
  }
}
