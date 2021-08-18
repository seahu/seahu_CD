/*
 * 1-Wire librrary for comunicatin to slave 1-Wire multi sensor/actor devices with family code 0xCD 
 * based on OneWireSlave0xCD arduino library.
 * WHAT IS 1-Wire uni measurment structure?
 * 1-wire uni measuring structure is open ource additions over 1-Wire comunication
 * for easy create 1-wire sensors who have automaticaly support into arduino (this librrary), OneWireFileSystem  and Linux kernel
 * enable create one or mluti sensors slave devices.
 * 
 * Every 1-wire device based on OneWireSlave0xCD arduino library contain one or more section with input sensor or output device (DA, pwm, RGB, ...)
 *  Every section have own control register, actual data, min and max value for alarm, description registers and register for store use note.
 *  Control register defined direct orintation of section, bit resolutin, and alarm options.
 *  The devices have also two global registry number of section and device description.
 *  This library contain function for easy read and write from/to those registers.
 *    
 *    sestion 0
 *    ---------
 *              CONTROL register
*/
#ifndef w1_uni_measurning_h
#define w1_uni_measurning_h

#include "Arduino.h"
#include <OneWire.h>

//#define OneWireMaster0xCDDebug // enable debug

// union representating more types of actual value
typedef union {
    bool            Bool;
    uint8_t         u8bit;
    unsigned long   u32bit;
    long            s32bit;
  } val;

class OneWireMaster0xCD
{
  

 

public:
    /*  
     *   INI
     */
    //OneWireUni(int pin);
    //OneWireUni(OneWire *ow, int pin);
    OneWireMaster0xCD(OneWire *ow);

    void setAddr(uint8_t addrres[]);
    void setSection(uint8_t section);
    void setOw(OneWire& ow);
    
    /*
     * GETNUMBER OF MEASURNING SECTIONS
     */
    uint8_t get_number_of_sections();

    /*
     * SELECT MEASURNING SECTION
     */
    bool select_sections(uint8_t number);

    /*
     * GET STATUS BYTE
     * result is returned and stored into global value - controlByte
     * return 0 means error
     */
    uint8_t get_status_byte();
    uint8_t get_status_byte(uint8_t section);

    /*
     * SET INTO STATUS BYTE
     * if no comunication  store new value into global value - controlByte
     * return true of false
    */
    uint8_t set_status_byte(uint8_t B);
    uint8_t set_status_byte(uint8_t B, uint8_t section);

    /*
     * PRINT STATUS BYTE
     */
    void print_status_byte();

    /*
     * IS ready for reead new value?
     */
    bool is_ready();
    
    /*
     * MEASURE REQUIREST ACTUAL SECTION
     */
    void start_measure();
    void start_measure(uint8_t section);

    /*
     * MEASURE REQUIREST ALL SECTIONS
     */
    void start_measure_all();

    
    /*
     * GET NEW MEASURED VALUE
     */
    bool get_measured_value(val *value);
    bool get_measured_value(val *value, uint8_t section);

    /*
     * get/set ACTUAL_VALUE from actual section
     */
    bool get_actual_value(val *value);
    bool get_actual_value(val *value, uint8_t section);
    
    bool set_actual_value(val *value);
    bool set_actual_value(val *value, uint8_t section);
    bool set_actual_value(uint32_t value, uint8_t section);
    bool set_actual_value(int32_t value, uint8_t section);
    bool set_actual_value(uint16_t value, uint8_t section);
    bool set_actual_value(int16_t value, uint8_t section);
    bool set_actual_value(uint8_t value, uint8_t section);
    bool set_actual_value(int8_t value, uint8_t section);
    bool set_actual_value(bool value, uint8_t section);

    /*
     * print value by type defined into control register for debug purpouse
     */
    void print_value(val *value, char head[]);

    /*
     * get/set MIN_ALARM_VALUE from actual section
     */
    bool get_min_alarm_value(val *value);
    bool get_min_alarm_value(val *value, uint8_t section);
    
    bool set_min_alarm_value(val *value);
    bool set_min_alarm_value(val *value, uint8_t section);
    
    /*
     * get/set MAX_ALARM_VALUE from actual section
     */
    bool get_max_alarm_value(val *value);
    bool get_max_alarm_value(val *value, uint8_t section);
    
    bool set_max_alarm_value(val *value);
    bool set_max_alarm_value(val *value, uint8_t section);

    /*
     * get/set secttion user note (!len of buf must be min 64 !)
     */
    bool get_note(char buf[]);
    bool set_note(char buf[]);
    bool get_note(char buf[], uint8_t section);
    bool set_note(char buf[], uint8_t section);
    
    /*
     * get section description (!len of buf must be min 32 !)
     */
    bool get_description(char buf[]);
    bool get_description(char buf[], uint8_t section);

   /*
    * get device description (!len of buf must be min 64 !)
    */    
    bool get_device_description(char buf[]);

//  private:
    uint8_t controlByte=0;
    //OneWire  ds;
    OneWire  *p_ow;
    uint8_t addr[8]; // 1-wire device addres
    uint8_t section; // actual section number
    
    /*
    * READ sekvencion over 1-Wire for device with family code 0xCD
    * ------------------------------------------------------------
    *  cmd:      command code
    *  *section: pointer to section number, if NULL sections is not used
    *  *buf:     pointer to data buffer will be return
    *  len:      size of buffer
    *  Delay:    is time to process 1B data sumDelay=Delay*len
    *  return:   true or false
    */
    bool read_uni(uint8_t cmd, uint8_t *section , void *buf, uint8_t len, uint8_t Delay);

    /*
    * WRITE sekvencion over 1-Wire for device with family code 0xCD
    * ------------------------------------------------------------
    *  cmd:      command code
    *  *section: pointer to section number, if NULL sections is not used
    *  *buf:     pointer to data buffer
    *  len:      size of buffer
    *  Delay:    is time to process 1B data sumDelay=Delay*len
    *  return:   true or false
    */
    bool write_uni(uint8_t cmd, uint8_t *section, void *buf, uint8_t len, uint8_t Delay);

    /*
     * GET/SET  VALUE over 1-Wire
     * get/set into/from value (used for actual, min and max value)
     * cmd must be code for command read/write actualValue, min_alarm_value or max_alarm_value
     */
    bool get_value(val *value, uint8_t cmd);
    bool set_value(val *value, uint8_t cmd, uint8_t Delay);
    
};

#endif
