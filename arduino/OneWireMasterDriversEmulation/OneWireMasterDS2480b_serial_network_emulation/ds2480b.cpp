#include "ds2480b.h"
 
/*
 * funkcni emulace integrovaneho obvodu DS2480B (master driver for 1-Wire bus control by serial bus)
 * 
 * emulace virtualne emlementuje temer veschny fce chipu. Ten umoznuje i konfiguraci jednotlivych delek ridicich impulsu,
 * diky pouziti knihovny <OneWire.h>  to nelze v reale aplikovat (delky signalu jsou napevn nastveny v teto knihovne, pro bezne vyuziti to nejak nevadi.
 * 1-Wire sbernice se ovlada s jedineho pinu na ktery je potreba pridat odpor 2,2 KOhm na +5V.
 * Bohuzel oproti oreginalnimu cipu nedisponije arduino funkci active pullup takze, tento radic bez pridavne elektroniky nelze pousit na delsi sbernice.
 * 
 * 
 * SEARCH princip
 * vyledavani se da delat bud pres jedno bitove operace (ele to je neefektovni protoze kvuli jednomu bit s rom adresy se musi provest tri jednobitove prikazy (2x cteni a 1x zapis) a kazdy prikaz je 1Byte poslany pres seriovou sbernici tj. 3x8=24bitu kvuli vyhledani jednoho bitu s 1-wire adresy) 
 * proto je efektivnejsi vyuyiti acceleratoru kdtry radic ds2480b nabizi
 * ten funguje tak, ze kazdemu bitu z 1-wire adresy se priradi jeste jeden bit, kde radic zapisuje vysledek vyhledavani pro dany bit a to tak, ze:
 * pokud na sbernici exituje vice zarizeni, ktera maji tento jit nastavena rozdilne (tzv. discrepancy) tak nastavi 1 a do rom adresy nastavi 0 (vychozi smer vetveni) (jnak receno nastavi zde poznamku, ze v tomto miste dochazi k vetveni adresy)
 * pokud jsou na sbernici jen zarizeni ktera maji tento bit nastaveny stejne tak to rozeznavaci algoritmus zjisti a vysledek zapise do patricneho pitu RO adresy.
 * 
 * ve vysledku pak mame nalezenon jednu ROM adresu a k nim masku urcujici na kterych bitovych pozicich se adresa muze vetvit (s tim, ze vychozi vetveni zacina 0)
 * vyhodnoceni vetveni tak je na nadrazenem programu, je pravidlem prochazet vetveni odspod tj. odseknout zaverecnou cast adresy (tj. vyplnit nulami) 
 * az po prvi nalezene vetveni a protoze nulova vetev uz prosla pri prvnim vyhledani, tak na dane misto umistit jednicku a nechat to rojit znovu.
 * Pri pristim vyhodnoceni toto misto vetveni jiz ignorujeme, protze kromne priznaku vetveni (discrepancy) jiz ma v ROM adrese na tomto miste 1
 * coz znaci, ze uz jsme prosli obe verianty vetveni (0 i 1), takze hledame dalsi misto vetveni.
 * A kdyz uz neni nalezeno zadne misto pro vyreseni vetveni, tak je konec vyhledavni.
 * ted prakticky
 * pres comand mode se nsatvi mod search accelerator
 * pak se prepne do datoveho modu
 * nasledne se posle 16 bytu tj. 2x64 bitu (ROM adresa s maskou vetveni, pri prvnim ruchodu same nuly)
 * pak se precte 16 bytu podpovet z radice
 * ulozi se ziskana adresa
 * najde se prvni nevyresene vetveni odmaze se cast adresy ktera se vetvi az do mista nalezenho vetveni v miste vetveni se zameni v rom adrese 0 za 1
 * cyklus se opakuje (opet se posle 16 bytu do radice) dokud nejsou vyresene vsechny vetve
 * 
 */

OneWire  ds(PIN_OW);  // on pin 3 (a 4.7K resistor is necessary)


// konfoguracni tabulky flexibilni rychlosti
uint16_t pdsrc_[]={1500,220,165,137,110,83,70,55}; // Pulldown Slave Rate Control (neco s delkou sbernice a jak rychle lze shodit stav na 0V, ale vubec jsem nepochopil k cemu se to tu pouziva, logicke by bylo k prodlouzeni intervalu, ale to s dokumentace nevyplyva) !nepouzivam [V/us]
uint16_t ppd_[]= {32,64,128,256,512,1024,2048,0}; // delky programovaciho puslu 12V [us]
uint16_t spud_[]={32,64,128,256,512,1024,2048,0}; // delky strong pullup duration 12V (jinak receno delky power poulsu 5V) [ms]
uint8_t  wilt_[]={8,9,10,11,12,13,14,15}; // delky Write-1 Low Time (delka uvodniho pulsu, pri cteni nebo zapisu jednicky) !nepouzivam, protoze v pouzite knihovne to nejde nastavit je tam natvrdo 10us pro zapis 1 a 3us pro cteni
uint8_t dso_w0rt_[]={3,4,5,6,7,8,8,10}; // delky Data Sample Offset and Write 0 Recorvery Time (pri cteni to je doma po ktere se samlije, po zapisu 0 je to odpocinkova doba) ! nepouzivam  v pouzite knihovne to nejde nastavit je tam natvrdo pri cteni 10us a pri zapisu odpocinkova doba 5us
uint8_t load_[]={18,21,24,27,30,33,36,39}; // nedocetl jsem se na co to je ??? !nepouzivam
uint16_t rbr_[]={9600, 19200, 57600, 115200, 9600, 19200, 57600, 115200}; // rychlost serioveho portu [kbps]

// DS2480B mapomerne slouzitou strukturu nastaveni hodnot, kterou se tu snazim implementovat i kdyz vetsinu hodnot vybec nepouziju
// nasteveni totiz zavisi od toho jaka byla v comand prikazu vybrana rychlost (regulerni, flexibilni a overspeed) 
// aby to nebylo jednoduche tak exitujou vychozi rychlosti pro kazdou rychlostni skupinu, u flexibilni rychlosti lze rychlosti konfigurovat dle tabulky(poli) vyse 
// a navic u regulerni i overspeed rychlosti se nektere hodnoty sdili s flexibilnim nastvenim
// vyhozi nastaveni indexu (odkazujicih do tebulek vyse) pro flexibilni rychlost
byte pdsrc_flex_; // 1500 x0.01V/us
byte ppd_flex_; // 512 us
byte spud_flex_; // 524ms
byte wilt_flex_; // 8us
byte dso_w0rt_flex_; // 3us
byte load_flex_; // ? x0.1mA
byte rbr_flex_; // 9600 kbps

byte  mode_;
bool  check_;
bool  search_accel_;
byte  pulse_response_; // 0= no pulse, other values means response value
bool  power_;

// deklarec promnenych pro aktulani rychlost
uint16_t pdsrc_actual_=pdsrc_[pdsrc_flex_];
uint16_t ppd_actual_=ppd_[ppd_flex_];
uint16_t spud_actual_=spud_[spud_flex_];
uint8_t wilt_actual_=wilt_[wilt_flex_];
uint8_t dso_actual_=dso_w0rt_[dso_w0rt_flex_];
uint8_t w0rt_actual_=dso_w0rt_[dso_w0rt_flex_];
uint8_t load_actual_=load_[load_flex_];
uint16_t rbr_actual_=rbr_[rbr_flex_];
uint16_t rbr_last_=rbr_[rbr_flex_];



void Serialwrite(byte data){
  #ifdef DEBUG_DS2480B
    client_.print(F("<-: "));
    client_.println(data,HEX);
  #endif
  #ifndef DEBUG_DS2480B
    //client_.print("OPDOVED:");
    //client_.println(data,HEX);
    client_.write(&data,1);
    if (data==0xFF) { // 0xFF se musi poslat 2x za sebou vis. RFC 2217
      //client_.print("OPDOVED:");
      //client_.println(data,HEX);
      client_.write(&data,1);
    }
  #endif
}


// set device to defualt state
void big_reset(){
  pinMode(PIN_12V, OUTPUT); // defaul off 12V programing signal
  digitalWrite(PIN_12V, LOW);
  
  pdsrc_flex_=0; // 1500 x0.01V/us
  ppd_flex_=4; // 512 us
  spud_flex_=0; // 524ms
  wilt_flex_=0; // 8us
  dso_w0rt_flex_=0; // 3us
  load_flex_=0; // ? x0.1mA
  rbr_flex_=0; // 9600 kbps  

  mode_=COMAND_MODE;
  check_=false;
  search_accel_=false;
  pulse_response_=0; // 0= no pulse, other values means response value
  power_ = false;

  //Serial.end(); // for net varinat serial communication not have importance
  //Serial.begin(9600); // for net varinat serial communication not have importance
  #ifdef DEBUG_DS2480B
    client_.println(F("BIG RESET"));
  #endif
}


void ds2480b_setup() {
  // put your setup code here, to run once:
  big_reset();
}

void pulseOFF(){
  if (pulse_response_!=0) { // is pulse
    digitalWrite(PIN_12V, LOW); // of 12V programing siglnal
    ds.depower(); // of 5V strong pullup signal
    Serialwrite(pulse_response_);
    pulse_response_=0; // pulse stop flag
  }
}

void pulseON(byte voltage, byte response){
  pulse_response_=response; // save response value to golbal variable
  if ( voltage==OW_PULSE_12V) { // 12V programing pulse
    digitalWrite(PIN_12V, HIGH);
    if (spud_actual_!=0) { // not infinite durapion
      delay(spud_actual_); // durtion in ms
      pulseOFF(); // and stop pulse
    }
    if (mode_==DATA_MODE && spud_actual_==0) { // prevetion no eneble never ending duration
      delay(spud_[6]); // set max durtion in ms from configuration table
      pulseOFF(); // and stop pulse      
    }
  }
  else { // 5V strong pullup signal
    pinMode(PIN_OW, OUTPUT);
    digitalWrite(PIN_OW, HIGH);    
    if (ppd_actual_!=0) { // not infinite durapion
      delay(ppd_actual_); // duration in us
      pulseOFF(); // and stop pulse
    }
    if (mode_==DATA_MODE && ppd_actual_==0) { // prevetion no eneble never ending duration
      delay(ppd_[6]); // set max durtion in ms from configuration table
      pulseOFF(); // and stop pulse      
    }
  }
  //PS: infinite durapion must be terminated by command 0xF1 (function owStopPulse() )
}




/* 
 * set actual speed values from communication cammands send here as papramter data 
 * command may set set diferent class speed (regular, flexible, overhead )
 * and every class speed have own setting
*/
void owSetSpeed(byte data){
  byte speeed;
  switch(data & CMD_SPEED_MASK){
    case REGULAR1_SPEED:
    case REGULAR2_SPEED:
      #ifdef DEBUG_DS2480B
        client_.print(F(" speed=regular"));
      #endif
      pdsrc_actual_=1500; // 1500 x0.01V/us
      ppd_actual_=ppd_[ppd_flex_]; // us (share with flexible speed)
      spud_actual_=spud_[spud_flex_]; //ms (share with flexible speed)
      wilt_actual_=8; // 8 us
      dso_actual_=3; // 3us
      w0rt_actual_=3; // 3us
      load_actual_=39; // x0.1mA
      rbr_actual_=rbr_[rbr_flex_]; // kbps (share with flexible speed)
      break;
    case FLEXIBLE_SPEED: // this speed is configureabled by CONFIGURATION COMMANDS who set indexies into configurations tables
      #ifdef DEBUG_DS2480B
        client_.print(F(" speed=flexible"));
      #endif
      pdsrc_actual_=pdsrc_[pdsrc_flex_];
      ppd_actual_=ppd_[ppd_flex_];
      spud_actual_=spud_[spud_flex_];
      wilt_actual_=wilt_[wilt_flex_];
      dso_actual_=dso_w0rt_[dso_w0rt_flex_];
      w0rt_actual_=dso_w0rt_[dso_w0rt_flex_];
      load_actual_=load_[load_flex_];
      rbr_actual_=rbr_[rbr_flex_];
      break;
    case OVERDRIVE_SPEED:
      #ifdef DEBUG_DS2480B
        client_.print(F(" speed=override"));
      #endif
      pdsrc_actual_=1500; // 1500 x0.01V/us
      ppd_actual_=ppd_[ppd_flex_]; // us (share with flexible speed)
      spud_actual_=spud_[spud_flex_]; //ms (share with flexible speed)
      wilt_actual_=1; // 8 us
      dso_actual_=1; // 3us
      w0rt_actual_=3; // 3us
      load_actual_=39; // x0.1mA
      rbr_actual_=rbr_[rbr_flex_]; // kbps (share with flexible speed)
      break;
  }
}

void cmdWriteConf(byte *p_save, byte data){
  byte value=(data & CMD_CONF_WRITE_VALUE_MASK) >> 1; // prepare new value by select from data and rotate one bit to right
  if ( (p_save==&rbr_flex_) && (value!=rbr_actual_) ) { // if change serial speed
    //Serial.end(); // for net varinat serial communication not have importance
    //Serial.begin(rbr_[value]); // for net varinat serial communication not have importance
  }
  *p_save=value;
  #ifdef DEBUG_DS2480B
    client_.print(F(" config WRITE="));
    if (p_save==&pdsrc_flex_)     client_.print(F("   CMD_CONF_PDSRC_W"));
    if (p_save==&ppd_flex_)       client_.print(F("   CMD_CONF_PDD_W")); 
    if (p_save==&spud_flex_)      client_.print(F("   CMD_CONF_SPUD_W"));
    if (p_save==&wilt_flex_)      client_.print(F("   CMD_CONF_WILT_W"));
    if (p_save==&dso_w0rt_flex_)  client_.print(F("   CMD_CONF_DSO_W0RT_W"));
    if (p_save==&load_flex_)      client_.print(F("   CMD_CONF_LOAD_W"));
    if (p_save==&rbr_flex_)       client_.print(F("   CMD_CONF_RBR_W"));
    client_.print(" new_value=");
    client_.print(value);
    client_.print(" means=>");
    if (p_save==&pdsrc_flex_) client_.println(pdsrc_[value]); // pdsrc[value]==pdsrc[pdsrc_flex_]
    if (p_save==&ppd_flex_) client_.println(ppd_[value]); 
    if (p_save==&spud_flex_) client_.println(spud_[value]);
    if (p_save==&wilt_flex_) client_.println(wilt_[value]);
    if (p_save==&dso_w0rt_flex_) client_.println(dso_w0rt_[value]);
    if (p_save==&load_flex_) client_.println(load_[value]);
    if (p_save==&rbr_flex_) client_.println(rbr_[value]);
  #endif
  Serialwrite(data & 0b01111110);
}

void cmdReadConf(byte *p_load, byte data){
  byte parametr_code= data >> 1; // prepare new value by select from data and rotate one bit to right
  byte value=*p_load;
  #ifdef DEBUG_DS2480B
    client_.print(F(" config READ="));
    if (p_load==&pdsrc_flex_)     client_.print(F("   CMD_CONF_PDSRC_R"));
    if (p_load==&ppd_flex_)       client_.print(F("   CMD_CONF_PDD_R")); 
    if (p_load==&spud_flex_)      client_.print(F("   CMD_CONF_SPUD_R"));
    if (p_load==&wilt_flex_)      client_.print(F("   CMD_CONF_WILT_R"));
    if (p_load==&dso_w0rt_flex_)  client_.print(F("   CMD_CONF_DSO_W0RT_R"));
    if (p_load==&load_flex_)      client_.print(F("   CMD_CONF_LOAD_R"));
    if (p_load==&rbr_flex_)       client_.print(F("   CMD_CONF_RBR_R"));
    client_.print(" actual_value=");
    client_.print(value);
    client_.print(" means=>");
    if (p_load==&pdsrc_flex_) client_.println(pdsrc_[value]); // pdsrc[value]==pdsrc[pdsrc_flex_]
    if (p_load==&ppd_flex_) client_.println(ppd_[value]); 
    if (p_load==&spud_flex_) client_.println(spud_[value]);
    if (p_load==&wilt_flex_) client_.println(wilt_[value]);
    if (p_load==&dso_w0rt_flex_) client_.println(dso_w0rt_[value]);
    if (p_load==&load_flex_) client_.println(load_[value]);
    if (p_load==&rbr_flex_) client_.println(rbr_[value]);
  #endif
  Serialwrite(data & 0b01110000 | (value<<1));
}


void cmdBit(byte data){
  #ifdef DEBUG_DS2480B
    client_.print(F("   CMD_SINGLE_BIT"));
  #endif
  owSetSpeed(data); // set speed by sended data
  byte bit_val=(data & OW_BIT_DATA_MASK);
  if ( bit_val==0 ) { //write bit=0
    ds.write_bit(0);
    Serialwrite((data & OW_BIT_RESPONSE_MASK) | 0b00000000 );
  }
  else { //read bit
   if (ds.read_bit()==1)  Serialwrite( (data & OW_BIT_RESPONSE_MASK) | 0b00000011 );
   else                 Serialwrite( (data & OW_BIT_RESPONSE_MASK) | 0b00000000 );
  }
  if ( (data & OW_BIT_PULSE_MASK)==OW_BIT_PULSE_MASK ) { // enable strong pullup 5V
    pulseON( OW_PULSE_5V , (data & OW_PULSE_RESPONSE_MASK) | CMD_PULSE );
  }
}

void cmdSearchAcceleratorControl(byte data){
  #ifdef DEBUG_DS2480B
    client_.print(F("   CMD_SEARCH_ACCELERATOR_CONTROL "));
  #endif
  owSetSpeed(data); // set speed by sended data
  if ( (data & OW_SEARCH_ACCEL_MASK) == OW_SEARCH_ACCEL_ON ) search_accel_=true;
  if ( (data & OW_SEARCH_ACCEL_MASK) == OW_SEARCH_ACCEL_OFF ) search_accel_=false;
  #ifdef DEBUG_DS2480B
    if (search_accel_==true) client_.println(F(" = ON"));
    else client_.println(F(" = OFF"));
  #endif
  // this command have not response byte
}

void cmdReset(byte data){
  #ifdef DEBUG_DS2480B
    client_.print(F("   CMD_RESET"));
  #endif   
  owSetSpeed(data); // set speed by sended data
  #ifdef DEBUG_DS2480B
    client_.println();
  #endif   
  if (ds.reset()==1) { // test prasence pulse from sleve device/s (ds. library distinguish only exist or no exists presence pulse, but oreginal DS2480 distinguish add more alarming presence pulse and 1-wire sorted bus)
    Serialwrite(OW_RESET_ANSWER | PRESENCE);
  }
  else {
    Serialwrite(OW_RESET_ANSWER | NO_PRESENCE);
  }
}

void cmdPulse(byte data){
  #ifdef DEBUG_DS2480B
    client_.print(F("   CMD_PULSE"));
  #endif   
  if ( (data & OW_PULSE_MASK) != OW_PULSE_MASK ) return; // test if it realy pusle command (test more then last tree bits)
  if ( (data & OW_PULSE_ARM_MASK) == OW_PULSE_ARM_MASK )  power_=true;
  else                                                    power_=false;
  #ifdef DEBUG_DS2480B
    if (power_==true) client_.println(F(" =ARM"));
    else              client_.println(F(" =DISARM"));
  #endif   
  pulseON( (data & OW_PULSE_VOLT_MASK) , data );
}

void cmdStopPulse(){
  #ifdef DEBUG_DS2480B
    client_.print(F("   CMD_STOP_PULSE"));
  #endif   
  pulseOFF();
  // this command have not response byte
}


byte owByte(byte data){
  byte ret;
  byte pulse_response;
  if (data==0xFF) ret=ds.read();
  else {
    ds.write(data, power_);
    ret=data;
  }
  // send result
  Serialwrite(ret);
  // pripadny puls
  if (power_==true) { // puls aktivni
    if ( (ret && 0b10000000) !=0) pulse_response=0xf6; // last bit is 1 => by documentattion  pulse resmonse is 0xF6
    else                          pulse_response=0x76; // last bit is 0 => by documentattion  pulse resmonse is 0x76
    pulseON( OW_PULSE_5V , pulse_response );
  }
  return ret;
}

byte owSearchByte(byte data){
  // vsichi poslou prvni bit - prectu ho
  // vsichni poslou invertovany prvni bit - prectu ho
  // | bit0 | bit1 | vyznam
  // |   0  |  0   | na sbernici je vice zarizeni a na tomto miste maji ruzne hodnoty (konflikt - more alternatives)
  // |   0  |  1   | na sbernici je jedno nebo vice zarizeni a na tomto miste maji 0 (no alternative)
  // |   1  |  0   | na sbernici je jedno nebo vice zarizeni a na tomto miste maji 1 (no alternative)
  // |   1  |  1   | na sernici neni zadne zarizeni ktere by reagovalo (error)
  //
  // pak se zapise bit a dale odpovidaji jen ta zarizeni ktera maji prvni bit stejny jako prave zaslany (pokud neni alternativa poslu je jasne co poslu, 
  // v pripade konfilktu o tom rozhoduje nadrazeny program, kery mi to posle v datech a ja si to pomoci "maskR" odchutim
  // a tak dokala dokud se neprojdou vsechny bity (pri zaple akceleraci vyhledavani je rom adresa posilana postupne, pricemz v kazdem byte je stridave ulozena adresa jejiz osah se pouzije
  // v pripade konfliktu a prosto pro ulozeni masky konfliktu (aby nadrezeny program vedel kde se adresa vetvi)
  // ukazka rozlozeni bytu na adresu "r" a masku konfliktu "d"
  // | 7  | 6  | 5  | 4  | 3  | 2  | 1  | 0  |
  // | r3 | d3 | r2 | d2 | r1 | d1 | r0 | d0 |  (do 1B se tedy vmestnaji jen 4b ROM adresy tak misto 8B ROM adrey se musi poslat 16B a take 16B se naspet precte)
  byte b0,b1,b2,d,r;
  byte ret=data;
  byte maskD=0b00000001;
  byte maskR=0b00000010;
  for (byte maskD=0b00000001; maskD!=0; maskD=(maskD<<2) ){
    b0=ds.read_bit();
    b1=ds.read_bit();
    if (b0==0 && b1==0) { // conflict (b2 chosen by the host)
      if ( (data&maskR)==0 )  b2=0;
      else                    b2=1;
      d=1;
      r=b2;
    }
    if (b0!=b1) { // no conflict (there is not alternative)
      b2=b0;
      d=0;
      r=b2;
    }
    if (b0==1 && b1==1) { // error (there is no response)
      b2=1;
      d=1;
      r=b2;
    }
    if (r==1) ret=ret|maskR;
    else      ret=ret&(~maskR);
    if (d==1) ret=ret|maskD;
    else      ret=ret&(~maskD);
    #ifdef DEBUG_DS2480B
      client_.print(F(" b0="));
      client_.print(b0);
      client_.print(F(" b1="));
      client_.print(b1);
      client_.print(F("  r="));
      client_.print(r);
      client_.print(F(" d="));
      client_.print(d);
      client_.print(F(" maskD="));
      client_.print(maskD, BIN);
      client_.print(F(" maskR="));
      client_.println(maskR, BIN);
    #endif   

    maskR=(maskR<<2);
    ds.write_bit(r);
  }
  Serialwrite(ret);
  return ret;
}



void do_command_mode(byte data){
  mode_=COMAND_MODE; // byla volana tato fukce, tim je dany command mode
  if (data==DATA_MODE) { // kontrola prepinace 0xE1 na data mod
      mode_=DATA_MODE;
      return;
  }
  if (data==0) big_reset();
  if (data==CMD_STOP_PULSE) cmdStopPulse();
  
  switch(data & CMD_COMMUNICATION_MASK){
    case CMD_SINGLE_BIT:
      cmdBit(data & OW_BIT_DATA_MASK);
      return;
    case CMD_SEARCH_ACCELERATOR_CONTROL:
      cmdSearchAcceleratorControl(data);
      return;
    case CMD_RESET:
      cmdReset(data);
      return;
    case CMD_PULSE:
      cmdPulse(data);
      return;
  }
  // CONFIGURATION COMMANDS CODES
  switch(data & CMD_CONFIGURATION_MASK){
    case CMD_CONF_PDSRC_W:
      cmdWriteConf(&pdsrc_flex_, data);
      break;
    case CMD_CONF_PDD_W:
      cmdWriteConf(&ppd_flex_, data);
      break;
    case CMD_CONF_SPUD_W:
      cmdWriteConf(&spud_flex_, data);
      break;
    case CMD_CONF_WILT_W:
      cmdWriteConf(&wilt_flex_, data);
      break;
    case CMD_CONF_DSO_W0RT_W:
      cmdWriteConf(&dso_w0rt_flex_, data);
      break;
    case CMD_CONF_LOAD_W:
      cmdWriteConf(&load_flex_, data);
      break;
    case CMD_CONF_RBR_W:
      cmdWriteConf(&rbr_flex_, data);
      break;
    case CMD_CONF_R:
      //client_.println("read cmd");
      switch(data & CMD_CONF_READ_PARAM_MASK ){
        case CMD_CONF_PDSRC_R:
          cmdReadConf(&pdsrc_flex_, data);
          break;
        case CMD_CONF_PDD_R:
          cmdReadConf(&ppd_flex_, data);
          break;
        case CMD_CONF_SPUD_R:
          cmdReadConf(&spud_flex_, data);
          break;
        case CMD_CONF_WILT_R:
          cmdReadConf(&wilt_flex_, data);
          break;
        case CMD_CONF_DSO_W0RT_R:
          cmdReadConf(&dso_w0rt_flex_, data);
          break;
        case CMD_CONF_LOAD_R:
          cmdReadConf(&load_flex_, data);
          break;
        case CMD_CONF_RBR_R:
          cmdReadConf(&rbr_flex_, data);
          break;
      }
  }
}


void do_data_mode(byte data, bool enable_check){
  mode_=DATA_MODE; // byla volana tato fukce, tim je dany data mod
  if (data==COMAND_MODE && enable_check==ENABLE_CHECK) { // command mode or data
      mode_=CHECK_MODE;
      return;
  }
  if (search_accel_==false) owByte(data);
  else { // search_accel==true
    owSearchByte(data);
  }
}

void do_stream(byte data){
  if (mode_==CHECK_MODE) {
    if (data==COMAND_MODE)  do_data_mode( data, DISABLE_CHECK);
    else                    do_command_mode(data);
    return;
  }
  if (mode_==COMAND_MODE) {
     do_command_mode(data);
     return;
  }
  if (mode_==DATA_MODE) {
    do_data_mode( data, ENABLE_CHECK);
    return;
  }
}
  

//-- jen pro debug pokusy -------------------------------------------
#ifdef DEBUG_DS2480B
  void hex_numbr_msg_to_bin_streem(byte msg[], size_t size){ // whole line with hex number convert to binary nyber by one call
      uint8_t value;
      char buf[3];
      if (size!=4) {
        client_.print(F("Input have "));
        client_.print(size);
        client_.println(F(" chars, but must have two char representated byte value in hex format"));
      }
      memset(buf,0,3); // null buffer
      memcpy ( buf, msg, 2 );
      value = (uint8_t)strtol(buf, NULL, 16); 
      client_.print(F("->: "));
      client_.println(value, HEX);
      return do_stream( value );
  }
#endif



/*
void loop() {
  #ifdef DEBUG_DS2480B
    do_stream( read_serial_bin_hex_number() );
  #endif
  #ifndef DEBUG_DS2480B
    // put your main code here, to run repeatedly:
    byte data;
    if ( client_.readBytes(&data, 1)==1 ) {
      do_stream(data);
   }
  #endif
}
*/
