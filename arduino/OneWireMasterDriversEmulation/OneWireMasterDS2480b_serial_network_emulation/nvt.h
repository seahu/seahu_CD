#ifndef _NVT_H_
#define _NVT_H_

#include "ds2480b.h"

//#define DEBUG_NVT

// kody ricich funkci , ktere zaroven pouzivam pro rozliseni stavu
#define DATA 0
#define IAC 0xff
#define SB 0xfa
#define SE 0xf0
#define WILL 0xfb
#define WONT 0xfc
#define DO 0xfd
#define DONT 0xfe

//base NVT commands
// 2byte
#define BRK 0xf3
//3 byte
#define ECHO 0x01
#define SUPPRES_GO_AHEAD 0x03
#define COM_PORT_OPTIONS 0x2c

//#otions of COM_POER_OPTIONS=0x2c
#define CAS_SET_BAUDRATE 0x01
#define CAS_SET_DATASIZE 0x02
#define CAS_SET_PARITY 0x03
#define CAS_SET_STOPSIZE 0x04

#define MAX_NVT_BUF 15

bool nvt(byte data);

#endif
