#!/bin/bash

# Example howto use 1-Wire slave device "SEAHU 1-Wire to 2xRGB + 1xPWM + 1xMotion sensor C2021 by Ing. On"
#    script use bash in linux with OWFS (One Wire File System - www.owfs.org)
#
# more at:
# https://github.com/seahu/seahu_CD
#
# Exmaple description:
# set repeated fluent change between two colors of led strip (sone way can be control RGB2 or WHITE led strip)
# ------------------------------------------------------------------------------------------------------------------
#Type device: 1-Wire
# Family code : 0xCD
# Device description: SEAHU 1-Wire to 2xRGB + 1xPWM + 1xMotion sensor C2021 by Ing. Ondrej Lycka
# Number sections: 14
#      section0 :  for frist  RGB led strip - actually color (as RGB number in binary 32bit format in range 0-16777215 (value=Blue+256*Green+65536*Red) 
#      section1 :  for frist  RGB led strip - duration in [ms] of change, if >0 duration is time of fluent change form actual to new RGB value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
#      section2 :  for frist  RGB led strip - number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
#      section3 :  for second RGB led strip - actually color (as RGB number in binary 32bit format in range 0-16777215 (value=Blue+256*Green+65536*Red) 
#      section4 :  for second RGB led strip - duration in [ms] of change, if >0 duration is time of fluent change form actual to new RGB value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
#      section5 :  for second RGB led strip - number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
#      section6 :  for white  PWM led strip - actually PWM value (as number in binary 32bit format in range 0-255) 
#      section7 :  for white  PWM led strip - duration in [ms] of change, if >0 duration is time of fluent change form actual to new PWM value if <0 duration is time of flash beatween actual and new value, 0= immediate color change
#      section8 :  for white  PWM led strip - number of repeats flueant or flash changes from actual to new RGB value 0=no repeat, -1=repeat to infinity, >0 number of repeats 
#      section9 :  for motion sensor         - sensor status (only for read) 1=motion sensor detect activity, 0=no detec activity
#      section10:  for motion sensor         - 0=no action, 1=if motion sensor detec activity set PWM white led strip to max value (ligh ON)
#      section11:  for motion sensor         - Time in ms in range 0-3600. The time to hold on light after end activity.
#      section12:  for led indicator         - Led indication of device led 1=on 0=off
#      section13:  for all                   - 1=save actual values as default values (into EEPROM.) (not for MCU ATmega8-no have enough ROM memory)


# First you need run OWFS.
# OWFS mount virtual file system usually into directory /mnt/1wire and represented every 1-Wire slave device as directory and its registries as files.
DEVICE="/mnt/1wire/uncached/CD.210209113616" # find device directory and copy here its address

# Then you can control it.

R=0
G=0
B=0

function color {
    R=$1
    G=$2
    B=$3
    color=$((R*65536+G*256+B))
    echo -n $color
}

echo "0" > $DEVICE/section1/actual_value  # for start set immediately colors without delay
echo $(color 0 0 0) > $DEVICE/section0/actual_value # set frist color to BLACK
echo $(color 255 0 0) > $DEVICE/section0/actual_value # set secound color to RED
echo "500" > $DEVICE/section1/actual_value  # set 500ms time of fluent duration change between two colors
echo "100" > $DEVICE/section2/actual_value  # set 100 cycles of repeat, 0=no repeat, -1=repeat to infinity
