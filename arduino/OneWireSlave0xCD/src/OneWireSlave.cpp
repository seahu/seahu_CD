#include "OneWireSlave.h"

//#define CONF_DEBUFG_IMPULS {DDRD|=(1<<PIND4);}
//#define DEBUFG_IMPULS {PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD|=(1<<PIND4);PORTD&=~(1<<PIND4);}
#define CONF_DEBUFG_IMPULS {}
#define DEBUFG_IMPULS {}

// rule of names variables
// _name - global variable aviable ouside from this file (is included in .h file as external)
// name_ - global variable aviable only inside this file
// name  - local variale 

OW_ADD_TO_GLOBAL            // add special blabal variable into speciict MCU definet into suportedMCU.h
uint8_t* _owid;             // pointer to ROM code of One Wire device
volatile uint8_t  bitp_;    //pointer to current Byte
volatile uint8_t  ByteP_;   //pointer to current Bit
volatile uint8_t  mode_;    //state
volatile uint8_t  wmode_;   //if 0 next bit that send the device is  0
volatile uint8_t  actbit_;  //current
volatile uint8_t  srcount_; //counter for search rom
volatile uint8_t  cbuf_;    //Input buffer for a command
volatile bool     _OW_alarm;// alarm status 1=alarm, 0=normal status
volatile bool     _OW_rc;   // ready to control flag 1=device is selected for OW comunication
volatile bool     _OW_lock; // lock update data during his reading or writeing
                            //  This lock can set and unset by higger level service program who service One Wire comunication,
                            //  and always is clear by One Wire reset signal.
volatile uint8_t     _OW_crc_mode; // crc mode used for calculate crc during read or write 0=no calculate 1=crc8 2=crc16
volatile uint8_t     _OW_crc8;     // crc8 check sum calculate during write or read
volatile uint16_t     _OW_crc16;    // crc16 check sum calculate during write or read
volatile uint8_t     _OW_crc8c;     // pro ladeni crc pozdeji smazat

void(*onCommand_)(uint8_t);
uint8_t* rxBuffer_;
uint8_t* txBuffer_;
uint8_t rxBufferLength_;
uint8_t txBufferLength_;
void(*rxCallback_)();
void(*txCallback_)();

PIN_INT {
  uint8_t lwmode = wmode_; //let this variables in registers
  if ((lwmode == OWW_WRITE_0)) {
    SET_LOW;  //if necessary set 0-Bit
    lwmode = OWW_NO_WRITE;
  }
  uint8_t lmode = mode_;
  uint8_t mix;

  DIS_OWINT; //disable interrupt, only in OWM_SLEEP mode it is active
  switch (lmode) {
    case OWM_SLEEP:
      SET_TIMER(OWT_MIN_RESET);
      EN_OWINT; //other edges ?
      break;
    //start of reading with falling edge from master, reading closed in timer isr
    case OWM_MATCH_ROM:  //falling edge wait for receive
    case OWM_READ:
    case OWM_READ_COMMAND:
    case OWM_READ_ROM_COMMAND:
      SET_TIMER(OWT_READLINE); //wait a time for reading
      break;
    case OWM_SEARCH_ROM:   //Search algorithm waiting for receive or send
      if (srcount_ < 2) { //this means bit or complement is writing,
        SET_TIMER(OWT_LOWTIME);
      } else
        SET_TIMER(OWT_READLINE);  //init for read answer of master
      break;
    case OWM_WRITE:
      //OCR = TCNT_REG + OWTrc=0; ByteP_ = 0; mode_ = OWM_MATCH_ROM; break;_LOWTIME; // tento radek jsem nepochopil asi nejaky pozustetek uprav tak jsem ho zakomentoval a nize vytvoril svoji upravu po otedtovani tento zadek smazat !!
      _OW_crc8c++;
      SET_TIMER(OWT_LOWTIME);
      // during wait calculate shift CRC
		if (_OW_crc_mode==CRC_8 ){ //calculate crt8 in flight if need	  
                  //_OW_crc16=(_OW_crc16*3)/5+_OW_crc8c++*7;
		  mix=(_OW_crc8^wmode_)&0x01;
		  _OW_crc8>>=1;
		  if (mix) _OW_crc8^=0x8C;
		}
		else if (_OW_crc_mode==CRC_16 ){ //calculate crt16 in flight if need
		  mix=(_OW_crc16^wmode_)&0x01;
		  _OW_crc16>>=1;
		  if (mix) _OW_crc16^=0xA001;
		}

      break;
    case OWM_CHK_RESET:  //rising edge of reset pulse
      SET_FALLING
      SET_TIMER(OWT_RESET_PRESENCE); //waiting for sending presence pulse
      lmode = OWM_RESET;
      _OW_lock=0; // clear lock
      break;
  }
  EN_TIMER;
  mode_ = lmode;
  wmode_ = lwmode;

}
void OneWireSlave::reset(){
  cbuf_ = 0;
  rxBuffer_=&cbuf_;
  ByteP_=0; bitp_ = 1;
  mode_ = OWM_SLEEP;
}

void OneWireSlave::waitToCmd(){
  cbuf_ = 0;
  rxBuffer_=&cbuf_;
  ByteP_=0; bitp_ = 1;
  _OW_crc8=0;
  _OW_crc16=0;
  mode_= OWM_READ_COMMAND;
}


TIMER_INT {
  //Ask input line sate
  uint8_t p = READ_PIN;
  uint8_t mix;
  //Interrupt still active ?
  if (CHK_INT_EN) {
    //maybe reset pulse
    if (p == 0) {
      mode_ = OWM_CHK_RESET; //wait for rising edge
      SET_RISING;
    }
    DIS_TIMER;
  } else {
    switch (mode_) {
      case OWM_RESET:  //Reset pulse and time after is finished, now go in presence state
        mode_ = OWM_PRESENCE;
        SET_LOW;
        SET_TIMER(OWT_PRESENCE);
        DIS_OWINT;  //No Pin interrupt necessary only wait for presence is done
        break;
      case OWM_SEARCH_ROM:
        RESET_LOW;  //Set low also if nothing send (branch takes time and memory)
        srcount_++;  //next search rom mode
        switch (srcount_) {
          case 1: wmode_ = !actbit_; //preparation sending complement
            break;
          case 3:
            if (p != (actbit_ == 1)) { //check master bit
              mode_ = OWM_SLEEP; //not the same go sleep
            } else {
              bitp_ = (bitp_ << 1); //prepare next bit
              if (bitp_ == 0) {
                bitp_ = 1;
                ByteP_++;
                if (ByteP_ >= 8) {
                  //mode = OWM_SLEEP; //all bits processed // odzkousrt apozdeji smazat
                  mode_ = OWM_READ_COMMAND; //same? get next command
                  _OW_alarm=0;
                  _OW_rc=1;
                  cbuf_ = 0;
                  rxBuffer_=&cbuf_;
                  ByteP_=0;
                  _OW_crc8=0;
                  _OW_crc16=0;
                  break;
                }
              }
              srcount_ = 0;
              actbit_ = (_owid[ByteP_] & bitp_) == bitp_;
              wmode_ = actbit_;
            }
            break;
        }
        break;
      case OWM_PRESENCE:
        RESET_LOW;  //Presence is done now wait for a command
        mode_ = OWM_READ_ROM_COMMAND;
        cbuf_ = 0; rxBuffer_=&cbuf_; ByteP_=0; bitp_ = 1;  //Command buffer have to set zero, only set bits will write in
        break;
      case OWM_READ:
      case OWM_READ_COMMAND:
        if (_OW_crc_mode==CRC_8 ){ //calculate crt8 in flight if need
          mix=(_OW_crc8^p)&0x01;
          _OW_crc8>>=1;
          if (mix) _OW_crc8^=0x8C;
        }
        else if (_OW_crc_mode==CRC_16 ){ //calculate crt16 in flight if need
          mix=(_OW_crc16^p)&0x01;
          _OW_crc16>>=1;
          if (mix) _OW_crc16^=0xA001;
        }
      case OWM_READ_ROM_COMMAND:
        if (p) {  //Set bit if line high
          //cbuf_ |= bitp_;
          rxBuffer_[ByteP_] |= bitp_;
        }
        bitp_ = (bitp_ << 1);
        if (!bitp_) { //8-Bits read
          bitp_ = 1;
          if (mode_==OWM_READ) { // go to superior function
            ByteP_++;
            if (ByteP_ == rxBufferLength_) {
              //cbuf_ = 0; rxBuffer_=&cbuf_; ByteP_=0; bitp_ = 1; mode_ = OWM_SLEEP; // mozna to neni spatny napad, kdyz volana fce zapomene co je potreba delat v dalsim kroku tak prednastavit cekani na reset
              rxCallback_();
              break;
            }
            else {
              rxBuffer_[ByteP_] = 0;
              break;
            }
          }
          if (mode_==OWM_READ_COMMAND) { // go to superior function
              mode_ = OWM_SLEEP; // if superior function not change mode_ (direct or by low level OW functions) defefaut is do nothing
              onCommand_(cbuf_);
              break;
          }
          switch (cbuf_) {
            case 0x33: // Read ROM command
              mode_ = OWM_WRITE;
              _OW_rc=0;
              txBuffer_ = &_owid[0];
              txBufferLength_ = 8;
              ByteP_ = 0;
              bitp_ = 1;
              actbit_ = (bitp_ & txBuffer_[0]) == bitp_;
              wmode_ = actbit_; //prepare for send firs bit
              break;
            case 0x55: _OW_rc=0; ByteP_ = 0; mode_ = OWM_MATCH_ROM; break;
            case 0xEC: // alarm search
              if (!_OW_alarm) {
                mode_ = OWM_SLEEP;
                break;
              }
            case 0xF0:  //initialize search rom
              mode_ = OWM_SEARCH_ROM;
              _OW_rc=0;
              srcount_ = 0;
              ByteP_ = 0;
              actbit_ = (_owid[ByteP_] & bitp_) == bitp_; //set actual bit
              wmode_ = actbit_; //prepare for writing when next falling edge
              break;
            case 0xCC: // Skip ROM Command
              _OW_rc=1; // e.g. ds2408 set int this place rc=0, because device was not be selected for comunication, bat only skip selection (I thing 0XCC command may by used on bus with only one device, and this case is not relevat if device wos selected or not)
            case 0xA5: // Resume command
              if (!_OW_rc) {
                mode_ = OWM_READ_COMMAND; //same? get next command
                _OW_alarm=0;
                _OW_rc=1;
                cbuf_ = 0;
                rxBuffer_=&cbuf_;
                ByteP_=0;
                _OW_crc8=0;
                _OW_crc16=0;
                // mode_ = OWM_SLEEP; // odzkouset a pozdeji smazat
                break;
              }
            default:
              mode_ = OWM_SLEEP; //all other commands do nothing
          }
        }
        break;
      case OWM_MATCH_ROM:
        if (p == ((_owid[ByteP_]&bitp_) == bitp_)) { //Compare with ID Buffer
          bitp_ = (bitp_ << 1);
          if (!bitp_) {
            ByteP_++;
            bitp_ = 1;
            if (ByteP_ >= 8) {
              mode_ = OWM_READ_COMMAND; //same? get next command
              _OW_alarm=0;
              _OW_rc=1;
              cbuf_ = 0;
              rxBuffer_=&cbuf_;
              ByteP_=0;
              _OW_crc8=0;
              _OW_crc16=0;
              //mode_ = OWM_SLEEP; //all other commands do nothing // odzkouset a pozdeji smazat
              //onCommand_(cbuf_); // odzkouset a pozdeji smazat
              break;
            }
          }
        } else {
          mode_ = OWM_SLEEP;
        }
        break;
      case OWM_WRITE: //  write to host
        RESET_LOW;
        bitp_ = (bitp_ << 1);
        if (!bitp_) {
          ByteP_++;
          bitp_ = 1;
          //if (ByteP_ == 2) txCallback_();
          if (ByteP_ >= txBufferLength_) {
            wmode_=1;
            mode_ = OWM_SLEEP;
            txCallback_();
//            
            break;
          };
        }
        actbit_ = (bitp_ & txBuffer_[ByteP_]) == bitp_;
        wmode_ = actbit_; //prepare for send firs bit
        DEBUFG_IMPULS
        break;
    }
  }
  if (mode_ == OWM_SLEEP) {
    DIS_TIMER;
  }
  if (mode_ != OWM_PRESENCE)  {
    SET_TIMER(OWT_MIN_RESET - OWT_READLINE);
    EN_OWINT;
  }
}


void noOpCallback_() {}

uint8_t OneWireSlave::crc8(const uint8_t* data, uint8_t numBytes)
{
  uint8_t crc = 0;
  while (numBytes--) {
    uint8_t inByte = *data++;
    for (short i = 8; i; i--) {
      uint8_t mix = (crc ^ inByte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inByte >>= 1;
    }
  }
  return crc;
}

void OneWireSlave::begin(void (*onCommand)(uint8_t), uint8_t* owId) {
  CONF_DEBUFG_IMPULS
  _owid = owId;
  _owid[7] = crc8(_owid,7);
  onCommand_ = onCommand;
  mode_ = OWM_SLEEP;
  wmode_ = OWW_NO_WRITE;
  _OW_crc_mode=0;
  CONF_PORT
  CONF_TIMER
  DIS_TIMER;
  SET_FALLING
  EN_OWINT  
  sei();
}

void OneWireSlave::read(uint8_t* buffer, uint8_t numBytes, void (*complete)()) { //  read from master
  //cli();
  OneWireSlave::beginReceiveBytes_(buffer, numBytes, complete);
  //sei();
}

void OneWireSlave::beginReceiveBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)()) {
  rxBuffer_ = buffer;
  rxBufferLength_ = numBytes;
  rxCallback_ = complete;
  mode_ = OWM_READ;
  ByteP_ = 0;
  rxBuffer_[0] = 0;
}

void OneWireSlave::write(uint8_t* buffer, uint8_t numBytes, void (*complete)()) { //write to master
  //cli(); // povoleni a nasledne zakazani s nejakeho podivneho duvodu okamzite vyvolani preruseni i kdyz k tomu jinak neni duvod (navic zakazani preruseni nic nereseni kdyz se nasledne okamzite zase povoli, funkce totiz jen nastrtuje zapis a okamzite konci)
  OneWireSlave::beginWriteBytes_(buffer, numBytes, complete == 0 ? noOpCallback_ : complete);
  //sei();
}

void OneWireSlave::beginWriteBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)()) {
  mode_ = OWM_WRITE;
  txBuffer_ = buffer;
  txBufferLength_ = numBytes;
  txCallback_ = complete;
  ByteP_ = 0;
  bitp_ = 1;
  actbit_ = (bitp_ & txBuffer_[0]) == bitp_;
  wmode_ = actbit_; //prepare for send firs bit
  DEBUFG_IMPULS
  DEBUFG_IMPULS
  DEBUFG_IMPULS
}

