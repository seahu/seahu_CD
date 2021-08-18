#!/bin/bash

# Example howto use 1-Wire slave device "SEAHU 1-Wire stepter motor driver C2020 by Ing. Ondrej Lycka"
# This example show how to drive stepetr motor.
# Script use bash under linux with OWFS (One Wire File System - www.owfs.org)
#
# Exmaple description:
# EXAMPLE HOW-TO USE STEPTER MOTOR WITH SEAHU 1-Wire MOTOR DRIVER
# ---------------------------------------------------------------
# Type device: 1-Wire
# Family code : 0xCD
# Device description: SEAHU 1-Wire stepter motor driver C2020 by Ing. Ondrej Lycka
# Number sections: 15
#      section0 :  MoveRelative       - movving relative distance in steps
#                                        -2147483646 steps => never ending move to left
#                                        +2147483646 steps => never ending move to right
#      section1 :  MoveAbsolute       - movving absolute distance in steps
#      section2 :  Speed              - moving speed in steps/sec
#      section3 :  Accel              - moving acceleration in steps/(sec*sec)
#      section4 :  Decel              - moving deceleration in steps/(sec*sec)
#      section5 :  AbsoluteCoordinate - actual absolute coordinate
#      section6 :  TimeToDisableMotor - delay in us to turns off the engine after inactivity
#      section7 :  ConfigFlags        - configure bits falgs
#                                          1.bit (+1) - 0= normal motor direction, 1= inverse motor direction
#                                          2.-4.bit   - select step table used for stepter motor 0=8 steps, 1=8 inverse steps, 2=4 steps, 3=4 inverse steps
#                                              432 1      432 1     432 1     432 1  bit.
#                                             =000 0     =001 0    =010 0    =011 0
#                                             (+0)       (+2)      (+4)      (+5)
#                                             A  B       A  B      A  B      A  B
#                                             11 10      00 01     11 10     00 01
#                                             11 00      00 11     11 01     00 10
#                                             11 01      00 10     10 11     01 00
#                                             10 01      01 10     01 11     10 00
#                                             10 11      01 00
#                                             00 11      11 00
#                                             01 11      10 00
#                                             01 10      10 01
#                                             
#                                          5.bit (+16) - 0= disable switch detect route begin, 1= enable switch detect route begin
#                                          6.bit (+32) - 0= disable switch detect route end, 1= enable switch detect route end
#                                          7.bit (+64) - 0= disable detect route begin by motor current, 1= enable detect route begin by motor current
#                                          8.bit (+128)- 0= disable detect route end by motor current, 1= enable detect route end by motor current
#                                          9.bit (+256)- 0= disable software control begin of route, 1=enable software control begin of route with value in min_alarm_value from section AbsoluteCoordinate
#                                          10.bit (+512)- 0= disable software control end of route, 1=enable software control end of route with value in max_alarm_value from section AbsoluteCoordinate
#                                          11.bit (+1024)- 0= do nothing, 1=start proces to autodetec boundary (autodec route begin and end, begin will be 0 in absolute coordinate)
#                                          12.bit (+2048)- 0=do nothing, 1=start proces to autodetec speed (optimal will be stred into section2 and maximal will be stored into section9)
#                                          13.bit (+4096)- 0=do nothing, 1=start proces to autodetec acceleration (optimal will be stred into section3 and section4)
#                                          etc. inverse motor direction and enable detec begin and end by motor current = 1+8+16
#      section8  :  ForceHalt          - 0=do nothing, 1=halt moving
#      section9  :  Current            - actual current
#      section10 :  MaxSteps           - max. number of step in one direction 0=no limit
#      section11 :  MaxAmp             - max. ampher enabled to motor consume, more stop motor
#      section12 :  MaxSpeed           - max. speed of motor detected by AutoCalibrateSpeedAndAccel (only for read)
#
#      section13 :  led idetification  - identify of 1-Wire device 1=on 0-off
#      section14 :  Save               - 0=do nothing, 1=save actual section values
#
# more at:
# https:#github.com/seahu/seahu_CD


# First you need run OWFS.
# OWFS mount virtual file system usually into directory /mnt/1wire and represented every 1-Wire slave device as directory and its registries as files.
DEVICE="CD.210812003507" # find device directory and copy here its address
DEVICE_FULL_PATH="/mnt/1wire/uncached/$DEVICE"
ALARM_FULL_PATH="/mnt/1wire/alarm"


# configure constatnats
REVERSE_DIRECTION=1
STEPS_TABLE=14
ENABLE_PIN_BEGENIN=16
ENABLE_PIN_END=32
ENABLE_AMP_BEGENIN=64
ENABLE_AMP_END=128
ENABLE_SW_BEGENIN=256
ENABLE_SW_END=512
START_AUTODETECT_BOUNDARY=1024
START_AUTODETECT_SPEED=2048
START_AUTODETECT_ACCEL=4096

# defaul config and motor values - can set this values by oneself
DEFAULT_DIRECTION=0
DEFAULT_STEP_TABLE=0
DEFAULT_MAX_STEPS=40000
DEFAULT_SPEED=1000
DEFAULT_ACCEL=4000
DEFAULT_DECEL=4000
DEFAULT_MAX_AMP=20
DEFAULT_CONFIG=$(( DEFAULT_DIRECTION | DEFAULT_STEP_TABLE*2 )) # use definitions from previous section - //configure constants 

# special vales for never ending values (for relative moving)
NEVER_ENDING_LEFT_MOVING=-2147483646
NEVER_ENDING_RIGHT_MOVING=2147483646

# define section number
MOVE_RELATIVE="section0"
MOVE_ABSOLUTE="section1"
SPEED_="section2"
ACCEL_="section3"
DECEL_="section4"
ABSOLUTE_COORDINATE="section5"
TIME_TO_DISABLE_MOTOR="section6"
CONFIG_FLAGS="section7"
FORCE_HALT="section8"
MAX_SPEED="section0"
MAX_STEPS="section10"
MAX_AMP="section11"
CURRENT="section12"
INFO_LED="section13"
SAVE="section14"

# steps move to one key push on this example
STEPS=500

function wait_to_end_moving {
	while :
		do
			ls "$ALARM_FULL_PATH" | grep "$DEVICE"
			if [ $? -eq 0 ]; then
				break
			fi
		done
}

function setup {
  echo $DEFAULT_MAX_STEPS > "$DEVICE_FULL_PATH/$MAX_STEPS/actual_value" # set max steps in one direction
  echo $DEFAULT_SPEED > "$DEVICE_FULL_PATH/$MAX_STEPS/actual_value" # set speed
  echo $DEFAULT_ACCEL > "$DEVICE_FULL_PATH/$ACCEL_/actual_value" # set acceleration
  echo $DEFAULT_DECEL > "$DEVICE_FULL_PATH/$DECEL_/actual_value" # set deceleration
  echo $DEFAULT_MAX_AMP > "$DEVICE_FULL_PATH/$MAX_AMP/actual_value" # set max curent consume (it is relaive no ampers, if you plan use to mus be set by own tests)
  echo $DEFAULT_CONFIG > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # set new config
  echo "1" > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/enable_min_alarm" # set enable alarm on end of relative moving
  echo "1" > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/enable_max_alarm" # set enable alarm on end of relative moving
  ls "$ALARM_FULL_PATH" | grep "$DEVICE" # reset previously alamr
}

#setup default values
setup

#do relative moving
#move to left
echo $((-1*$STEPS)) > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination left direciton
wait_to_end_moving
#move to right
echo $STEPS > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination right direcition

