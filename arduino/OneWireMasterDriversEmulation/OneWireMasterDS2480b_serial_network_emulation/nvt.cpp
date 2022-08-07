#include "nvt.h"
 
#ifdef DEBUG_NVT
  char s_[30];
#endif
byte mod_=DATA;
byte nvt_buf_[MAX_NVT_BUF];
byte nvt_buf_size_ = 0;
int counter_=0;

void print_nvt_buf(){
  byte i=nvt_buf_size_;
  for (i; i>0; i--){
    Serial.print(nvt_buf_[i], HEX);
  }
  Serial.println();
}
 
bool nvt_COM_PORT_OPTIONS(){
        //global mod_, nvt_buf_, nvt_buf_size_, client_
        #ifdef DEBUG_NVT
          sprintf(s_,"%x COM_PORT_OPTIONS COMMAND", nvt_buf_[2]);
          Serial.print(s_);
        #endif
        //print nvt_buf_
        //if nvt_buf_[3]==CAS_SET_BAUDRATE: udelej neco
        nvt_buf_[3]=nvt_buf_[3]+100; // v odpovedi by tento parametr mnel mit vzdy o 100 vice (dle dokumentace rfc 2217) j        print("Answer for uknown SB command: "),
        #ifdef DEBUG_NVT
          print_nvt_buf();
        #endif
        //client_.write(nvt_buf_,nvt_buf_size_);
        mod_=DATA;
        nvt_buf_size_=0;
        return false;
}

bool unknown_SB(){
        //global mod_, nvt_buf_, nvt_buf_size_, client_
        #ifdef DEBUG_NVT
          sprintf(s_,"%x UNKNOWN SE COMMAND", nvt_buf_[2]);
          Serial.print(s_);
          print_nvt_buf();
          Serial.print(F("Answer for uknown SB command: "));
          print_nvt_buf();
        #endif
        //client_.write(nvt_buf_,nvt_buf_size_);
        mod_=DATA;
        nvt_buf_size_=0;
        return false;
}

bool positive_WILL_DO(){ // vraceni pozitivni odpovedi
        //global mod_, nvt_buf_, nvt_buf_size_, client_
        #ifdef DEBUG_NVT
          sprintf(s_,"%x 3B COMMAND POSITIVE ANSWER", nvt_buf_[2]);
          Serial.print(s_);
          print_nvt_buf();
        #endif
        // odpoved bude pozitivni -> zamena will za do a do za will
        if (nvt_buf_[1] == WILL) nvt_buf_[1]=DO;
        if (nvt_buf_[1] == DO) nvt_buf_[1]=WILL;
        #ifdef DEBUG_NVT
          Serial.print(F("Answer for uknown 3B command: ")),
          print_nvt_buf();
        #endif
        //client_.write(nvt_buf_,nvt_buf_size_);
        mod_=DATA;
        nvt_buf_size_=0;
        return false;
}

bool unknown_WILL_WONT_DO_DONT(){ // protoze jde o nezname fukce tj. zde nepodporovanou funkcionalitu tak vratim zapornou odpoved
        //global mod_, nvt_buf_, nvt_buf_size_, client_
        #ifdef DEBUG_NVT
          sprintf(s_,"%x UNKNOWN  3B COMMAND ", nvt_buf_[2]);
          Serial.print(s_);
          print_nvt_buf();
        #endif
        // pro DONT a WONT nevracim zadnou odpoved
        if ( nvt_buf_[1]==DONT ||  nvt_buf_[1]== WONT ) {
                mod_=DATA;
                nvt_buf_size_=0;
                return 0;
        }
        // odpoved bude ale yaporna -> zamena will za dont a do za wont
        if ( nvt_buf_[1]==WILL ) nvt_buf_[1]=DONT;
        if ( nvt_buf_[1]==DO )   nvt_buf_[1]=WONT;
        #ifdef DEBUG_NVT
          Serial.print(F("Answer for uknown 3B command: "));
          print_nvt_buf();
        #endif
        //client_.write(nvt_buf_,nvt_buf_size_);
        mod_=DATA;
        nvt_buf_size_=0;
        return false;
}

// Network Virtual Terminal rfc 2217
// vrati true pokud jde o data a false pokud jde o oknofuracni pokyny nvt (rfc 2217) terminalu
// oddelujici ynameni od dat je byte 0xFF za nimz nasleduje konfigurace s dolece kosatzmi moznostmi
// 1 2 a vice baidovzch povelu pro rizeni serioveho portu (rychlosti, start, stop bity atd... .
// pokud se ma v datech poslat hodnota 0xFF tak se musi poslat 2x za sebou.
bool nvt(byte data){
        if ( nvt_buf_size_ > MAX_NVT_BUF ) { // buffer je dostatecne nadimenyovam pro stracovani jednoho rikazu, pokud se zaplni je neco spatne
          // reset celho zariceni
          mod_=DATA;
          nvt_buf_size_=0;
        }
        nvt_buf_[nvt_buf_size_++]=data;
        switch(mod_){
        case DATA:
                if ( data==IAC ){
                        #ifdef DEBUG_NVT
                          Serial.print(F("IAC"));
                        #endif
                        mod_=IAC;
                        return false;
                }
                else {
                         nvt_buf_size_=0;
                         return true;
                }
        case IAC:
                switch(data) {
                    case IAC:
                        mod_=DATA;
                        nvt_buf_size_=0; // v prvnim byte je lozeno 0xFF
                        return true;
                    case SB:
                        #ifdef DEBUG_NVT
                          Serial.print(F("SB"));
                        #endif
                        mod_=SB;
                        return false;
                    case WILL:
                        #ifdef DEBUG_NVT
                          Serial.print(F("WILL"));
                        #endif
                        mod_=WILL;
                        return false;
                    case WONT:
                        #ifdef DEBUG_NVT
                          Serial.print(F("WONT"));
                        #endif
                        mod_=WONT;
                        return false;
                    case DO:
                        #ifdef DEBUG_NVT
                          Serial.print(F("DO"));
                        #endif
                        mod_=DO;
                        return false;
                    case DONT:
                        #ifdef DEBUG_NVT
                          Serial.print(F("DONT"));
                        #endif
                        mod_=DONT;
                        return false;
                    // server basic (only 2 byte nvt commands)
                    case BRK:
                        #ifdef DEBUG_NVT
                          Serial.print(F("BRK"));
                        #endif
                        big_reset(); //  do big reset device
                        mod_=DATA;
                        return false; 
                    //optionaly add pupport for more basic commands
                    #ifdef DEBUG_NVT
                      sprintf(s_,"%x UNKNOWN 2B COMMAND", data);
                      Serial.print(s_);
                    #endif
                    mod_=DATA; //unknown command
                    return false;
                }
        case SB:
                if (data==IAC){
                        #ifdef DEBUG_NVT
                          Serial.print(F("IAC->SE"));
                        #endif
                        mod_=SE;
                        return false;
                }
                else {
                        #ifdef DEBUG_NVT
                          Serial.print(data, HEX);
                        #endif
                        return false;
                }
        case SE:
                if (data==SE) {
                        #ifdef DEBUG_NVT
                          Serial.print(F("SE"));
                        #endif
                        mod_=DATA;
                        // seznam podporovanych SB prikazu
                        //if nvt_buf_[2]==prikaz: return func_prikaz
                        if ( nvt_buf_[2]==COM_PORT_OPTIONS) return nvt_COM_PORT_OPTIONS();
                        return unknown_SB();
                }
                else {
                        mod_=SB;
                        return false;
                }
        case WILL :
                // serve 3 bytes nvt commands
                if ( data==COM_PORT_OPTIONS ) return positive_WILL_DO();
                return unknown_WILL_WONT_DO_DONT();
        case WONT :
                // serve 3 bytes nvt commands
                return unknown_WILL_WONT_DO_DONT();
        case DO :
                // serve 3 bytes nvt commands
                if ( data==ECHO ) return positive_WILL_DO();
                if ( data==SUPPRES_GO_AHEAD ) return positive_WILL_DO();
                if ( data==COM_PORT_OPTIONS ) return positive_WILL_DO();
                return unknown_WILL_WONT_DO_DONT();
        case DONT :
                // serve 3 bytes nvt commands
                return unknown_WILL_WONT_DO_DONT();
        }
}
