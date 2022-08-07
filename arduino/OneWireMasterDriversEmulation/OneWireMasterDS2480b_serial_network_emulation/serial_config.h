#ifndef _SERIAL_CONFIG_H_
#define _SERIAL_CONFIG_H_

// -- 1-Wire PIN ---
#define PIN_OW 3

// -- serial config
#define SERAIL_BUF_SIZE 25

//--- network ---
#define MACADDRESS 0x00,0x01,0x02,0x03,0x04,0x05
#define MYIPADDR 192,168,2,200
#define MYIPMASK 255,255,255,0
#define MYDNS 192,168,2,1
#define MYGW 192,168,2,1
#define LISTENPORT 1000
#define UARTBAUD 115200

// definitions and declare variables for configuraion this device
#define CHECK_DEFAULT 0 // EEPROM position for test first save values 
#define FIRST 1
#define PORT 2
#define IP 4
#define MASK 4
#define GATEWAY 4
#define DNS 4
#define DHCP 1
#define RFC2217 1
#define MAC 6
#define ROM 8


extern uint16_t myPORT;
extern uint8_t myMAC[6];
extern uint8_t myIP[4];
extern uint8_t myMASK[4];
extern uint8_t myDNS[4];
extern uint8_t myGW[4];
extern uint8_t myDHCP;
extern uint8_t myRFC2217;

void config_setup(void);
void serial_command(void);

#endif
