#ifndef _DS2480B_H_
#define _DS2480B_H_
#include <UIPEthernet.h>
// The connection_data struct needs to be defined in an external file.
#include <UIPServer.h>
#include <UIPClient.h>
#include <OneWire.h>
#include "serial_config.h"

extern EthernetClient client_; // globalni adresa pro komunikaci se spojenim





//#define DEBUG_DS2480B // enable human readlable input and output by hex representation binary number on serial port for debug use telnet

//#define PIN_OW 3
#define PIN_12V 4

#define COMAND_MODE 0xE3
#define DATA_MODE   0xE1
#define CHECK_MODE  0x01 //  jen rozlisujici cislo, yadna navaznost na dokumentaci chipu
#define ENABLE_CHECK  true
#define DISABLE_CHECK  false


#define POWER_ON  0
#define POWER_OFF 1

// OW reset
#define SHORT           0b00000000 // response for 1-wire is shoted
#define PRESENCE        0b00000001 // response for presence pulse
#define ALARM_PRESENCE  0b00000010 // response for Alarm presence pulse
#define NO_PRESENCE     0b00000011 // response for no presence pulse
#define OW_RESET_ANSWER 0b11001100 // base code serial answer to owReset

// OW read/write bit
#define OW_BIT_DATA_MASK     0b00010000 // command mask where is stored bit value to write
#define OW_BIT_PULSE_MASK    0b00000010 // mask for pulse bit
#define OW_BIT_RESPONSE_MASK 0b10011100 // mask for help make response


// OW search accelertion Contril
#define OW_SEARCH_ACCEL_ON    0b10110001
#define OW_SEARCH_ACCEL_OFF   0b10100001
#define OW_SEARCH_ACCEL_SPEED 0b00001100 // speed mask (do not use speed)
#define OW_SEARCH_ACCEL_MASK  0b11110011 // mask for detec on/off search accel

// OW pulse
#define OW_PULSE_MASK          0b11101101 // mask for detect pulse comunication command
#define OW_PULSE_RESPONSE_MASK 0b00011100 // mask for help make response
#define OW_PULSE_VOLT_MASK     0b00010000 // mask for detect 5/12 volt of pulse
#define OW_PULSE_ARM_MASK      0b00000010 // mask for detect 5/12 volt of pulse
#define OW_PULSE_5V            0b00000000 // same as OW_PULSE_DISARM_5V but better sound
#define OW_PULSE_12V           0b00010000 // same as OW_PULSE_DISARM_12V but better sound


// COMMUNICATION COMMANDS CODES
#define CMD_COMMUNICATION_MASK          0b11100001
#define CMD_SINGLE_BIT                  0b10000001
#define CMD_SEARCH_ACCELERATOR_CONTROL  0b10100001
#define CMD_RESET                       0b11000001
#define CMD_PULSE                       0b11100001
#define CMD_STOP_PULSE                  0b11110001
// SPEED FROM COMMUNICATION COMMANDS
#define CMD_SPEED_MASK                  0b00001100
#define REGULAR1_SPEED                  0b00000000
#define FLEXIBLE_SPEED                  0b00000100
#define OVERDRIVE_SPEED                 0b00001000
#define REGULAR2_SPEED                  0b00001100
// CONFIGURATION COMMANDS CODES
#define CMD_CONFIGURATION_MASK          0b11110001
#define CMD_CONF_WRITE_VALUE_MASK       0b00001110
#define CMD_CONF_PDSRC_W                0b00010001 // write Pulldown Slew Rate Control
#define CMD_CONF_PDD_W                  0b00100001 // write Programing Pulse Duration
#define CMD_CONF_SPUD_W                 0b00110001 // write Strong Pullup Duration
#define CMD_CONF_WILT_W                 0b01000001 // write Write 1 Low Time
#define CMD_CONF_DSO_W0RT_W             0b01010001 // write Data Sample Offset and Write 0 Recorvery Time
#define CMD_CONF_LOAD_W                 0b01100001 // write load ?
#define CMD_CONF_RBR_W                  0b01110001 // write RS232 baud Rate
#define CMD_CONF_R                      0b00000001 // read config values
#define CMD_CONF_READ_PARAM_MASK        0b00001111 
#define CMD_CONF_PDSRC_R                0b00000011 // read Pulldown Slew Rate Control
#define CMD_CONF_PDD_R                  0b00000101 // read Programing Pulse Duration
#define CMD_CONF_SPUD_R                 0b00000111 // read Strong Pullup Duration
#define CMD_CONF_WILT_R                 0b00001001 // read Write 1 Low Time
#define CMD_CONF_DSO_W0RT_R             0b00001011 // read Data Sample Offset and Write 0 Recorvery Time
#define CMD_CONF_LOAD_R                 0b00001101 // read load ?
#define CMD_CONF_RBR_R                  0b00001111 // read RS232 baud Rate 


void big_reset();
void ds2480b_setup();
void do_stream(byte data);

#ifdef DEBUG_DS2480B
  void hex_numbr_msg_to_bin_streem(byte msg[], size_t size);
#endif

#endif
