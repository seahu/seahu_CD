#!/bin/bash

# Example howto use 1-Wire slave device "SEAHU Infrared receiver/transmiter v1 C2021 by Ing. Ondrej Lycka""
# This example show how to receive IR signals. Every detected IR signals will be show on separate line as binary data in hex representation.
# Script use bash under linux with OWFS (One Wire File System - www.owfs.org)
#
# Exmaple description:
# simply recivre IR remote buttons codes by 1-wire bus
# -----------------------------------------------------
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
#      section6 :  always 0  write to 1 to acutal_value start save all  values from all sections to EEPROM
#
# more at:
# https://github.com/seahu/seahu_CD


# First you need run OWFS.
# OWFS mount virtual file system usually into directory /mnt/1wire and represented every 1-Wire slave device as directory and its registries as files.
DEVICE="CD.210417105611" # find device directory and copy here its address
DEVICE_FULL_PATH="/mnt/1wire/uncached/$DEVICE"

# suitable if on bus is more devices and more devices need serve fast response to alarm status
function wait_to_new_signal_by_alarm_status {
    printf '\000' > "$DEVICE_FULL_PATH/section0/user_note" #prevention clear actual data to allow get new
    while true
	do
	for filename in "/mnt/1wire/alarm"/*; do
	    if [ $filename == "/mnt/1wire/alarm/$DEVICE" ] ; then
		return
	    fi
	done
    done
}

# suitable for bus with less count devices who need fast response
function wait_to_new_signal_by_repeate_read_data {
    while true
	do
	new=$(< "$DEVICE_FULL_PATH/section0/user_note" tr -d \\0)
	if [ "$new" != "" ] ; then  break; fi
    done
}

echo "RECEVRED IT signal by 1-Wire bus."
echo "Stop program ctrl+c"
while true
    do
    #echo $(wait_to__new_signal_by_alarm_status) # for wait to date you can use this function
    wait_to_new_signal_by_repeate_read_data          # or this function
    new=$(< "$DEVICE_FULL_PATH/section0/user_note" tr -d \\0) # tr -d \\0 because new bash print warning if optput contain byes with null
    echo "$new"
    printf '\000' > "$DEVICE_FULL_PATH/section0/user_note" # null data to enable next
done
