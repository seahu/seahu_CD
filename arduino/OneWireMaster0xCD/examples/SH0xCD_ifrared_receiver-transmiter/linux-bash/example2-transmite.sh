#!/bin/bash

# Example howto use 1-Wire slave device "SEAHU Infrared receiver/transmiter v1 C2021 by Ing. Ondrej Lycka""
# This example show how to transmite binary datat in hex representation to IR signals.
# Script use bash under linux with OWFS (One Wire File System - www.owfs.org)
#
# Exmaple description:
# simply transmite buttons codes by 1-Wire bus to simalate push buttons on IR remote controler
# --------------------------------------------------------------------------------------------
#Type device: 1-Wire
# Family code : 0xCD
# Device description: SEAHU Infrared receiver/transmiter v1 C2021 by Ing. Ondrej Lycka
# Number sections: 6
#      section0 :  string buffer of binary data in hexadecimal display (for next data receive or transmit must be previously data deleted by writting binary zero here)
#      section1 :  number of retransmissions
#      section2 :  transmition mode 0=38KHz 1=36KHz
#      section3 :  led indication IR activity 1=enable 0-disable
#      section4 :  control green led 0=off, 1=on
#      section5 :  led idetification of 1-Wire device 1=on 0-off
#      section6 :  always 0 write to 1 to acutal_value start save all  values from all sections to EEPROM
#
# more at:
# https://github.com/seahu/seahu_CD


# First you need run OWFS.
# OWFS mount virtual file system usually into directory /mnt/1wire and represented every 1-Wire slave device as directory and its registries as files.
DEVICE="CD.210417105611" # find device directory and copy here its address
DEVICE_FULL_PATH="/mnt/1wire/uncached/$DEVICE"

# you can get your own  button codes using scanning with script example1-receivre.sh
button_on="E66F3EA6410000AAAA2A0080AA00"
button_off="E5713BAB410000AAAA280082AA00"

echo "Transmite IT signal by 1-Wire bus."
echo "Stop program ctrl+c"
#config parameters
echo "3" > "$DEVICE_FULL_PATH/section1/actual_value" # number of transmite repeats (just one but 3 or more is better in disturbed space)
echo "0" > "$DEVICE_FULL_PATH/section2/actual_value" # 0=transmite on 38KHz 1=transmite on 36KHz

#repeadly transmite signals simulates remote control on/off buttons
while true
    do
    #echo $(wait_to__new_signal_by_alarm_status) # for wait to date you can use this function
    printf '\000' > "$DEVICE_FULL_PATH/section0/user_note" # null data to enable next
    echo "$button_on" > "$DEVICE_FULL_PATH/section0/user_note" # button on (note: after sent new value you must leave some time for slave device may transmite IR signal, after this time you can null data)
    echo "sent button on"
    sleep 1
    printf '\000' > "$DEVICE_FULL_PATH/section0/user_note" # null data to enable next
    echo "$button_off" > "$DEVICE_FULL_PATH/section0/user_note" # button off
    echo "sent button off"
    sleep 1
done
