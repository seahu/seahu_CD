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
DEFAULT_CONFIG=$(( DEFAULT_DIRECTION | DEFAULT_STEP_TABLE*2 | ENABLE_SW_BEGENIN | ENABLE_AMP_END )) # use definitions from previous section - //configure constants 

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

function print_help {
  echo "HELP"
  echo "----"
  echo "h - this help."
  echo "i - List actual setting."
  echo "b - Start autodetect boundary by motor cuurent consume."
  echo "d - Start autodetect speed."
  echo "a - Start autodetect acceleration."
  echo "A - Switch enable/disable alarm on end of relative moving."
  echo "o - Set origin for absolute coordination."
  echo "z - Set beginin trace (in absolute coordinaton)."
  echo "x - Set end trace (in absolute coordinaton)."
  echo "c - Switch Enable/Disable sowtware trace boundary."
  echo "e - Switch Disable(halt)/Enable motor."
  echo "f - Store actual absolute coordination."
  echo "g - Go to stored absolute coordination."
  echo "n - Relative left move steps."
  echo "m - Relative right move steps."
  echo "N - Never ending left move."
  echo "M - Relative right move steps."
  echo "l - Switch ON/OFF indication led."
  echo "s - Save actual section values."
  echo "[CTRL+C] - to stop!"
  echo "note: More setting in source by edit 'defaul config and motor values' define values."
  echo "--"
}

function flag_comparation { #auxliary function for hlep print status of flags in next function
  if [ $(($1 & $2)) -eq 0 ]
  then
     echo "N" 
  else 
     echo "Y"
  fi
}

function print_actual_info {
  echo "List actual setting"
  echo "-------------------"
  echo "Config:"
  value=$(cat "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value") #get config flags of stepeter motor settings
  echo "Reverse direction:"  $(flag_comparation $value $REVERSE_DIRECTION)
  echo "Steps table:"  $(($(($value & $STEPS_TABLE)) >> 1))
  echo "Enable begenin by PIN:"  $(flag_comparation $value $ENABLE_PIN_BEGENIN)
  echo "Enable end by PIN:"  $(flag_comparation $value $ENABLE_PIN_END)
  echo "Enable begenin by current:"  $(flag_comparation $value $ENABLE_AMP_BEGENIN)
  echo "Enable end by current:"  $(flag_comparation $value $ENABLE_AMP_END)
  echo "Enable begenin by software from min_lalarm_value in section ABSOLUTE_COORDINATE:"  $(flag_comparation $value $ENABLE_SW_BEGENIN)
  echo "Enable end by software from max_lalarm_value in section ABSOLUTE_COORDINATE:"  $(flag_comparation $value $ENABLE_SW_END)
  echo "Min alarm value:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/min_alarm_value")
  echo "Max alarm value:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/max_alarm_value")
  stat=$(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/control_byte") #get control byte for ABSOLUTE_COORDINATE section for check if is set alarm for this section
  echo "Is set alarm for absolute min_alarm_value:" $(flag_comparation $stat 2#01000000)
  echo "Is set alarm for absolute max_alarm_value:" $(flag_comparation $stat 2#10000000)
  stat=$(cat "$DEVICE_FULL_PATH/$MOVE_RELATIVE/control_byte") #get control byte for MOVE_RELATIVE section for check if is set alarm for this section
  echo "Is set alarm for end relative moving:" $(flag_comparation $stat 2#01100000)
  echo "Actual absolute coorinate:"  $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure")
  echo "Speed:"  $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/actual_value")
  echo "Accel:"  $(cat "$DEVICE_FULL_PATH/$ACCEL_/actual_value")
  echo "Decel:"  $(cat "$DEVICE_FULL_PATH/$DECEL_/actual_value")
  echo "Time to disable motor after n activity:"  $(cat "$DEVICE_FULL_PATH/$TIME_TO_DISABLE_MOTOR/actual_value")
  echo "Max steps in one moving:"  $(cat "$DEVICE_FULL_PATH/$MAX_STEPS/actual_value")
  echo "Max speed discoverd by autocalibration speed proces:"  $(cat "$DEVICE_FULL_PATH/$MAX_SPEED/actual_value")
  echo "Max current for stop motor:"  $(cat "$DEVICE_FULL_PATH/$MAX_SPEED/actual_value")
  echo "Actual current:"  $(cat "$DEVICE_FULL_PATH/$CURRENT/actual_value")
  echo "Status of info led:"  $(cat "$DEVICE_FULL_PATH/$INFO_LED/actual_value")
}


function setup {
  echo $DEFAULT_MAX_STEPS > "$DEVICE_FULL_PATH/$MAX_STEPS/actual_value" # set max steps in one direction
  echo $DEFAULT_SPEED > "$DEVICE_FULL_PATH/$MAX_STEPS/actual_value" # set speed
  echo $DEFAULT_ACCEL > "$DEVICE_FULL_PATH/$ACCEL_/actual_value" # set acceleration
  echo $DEFAULT_DECEL > "$DEVICE_FULL_PATH/$DECEL_/actual_value" # set deceleration
  echo $DEFAULT_MAX_AMP > "$DEVICE_FULL_PATH/$MAX_AMP/actual_value" # set max curent consume (it is relaive no ampers, if you plan use to mus be set by own tests)
  print_help
}

while :
do
  read -p "command:" cmd
  case "$cmd" in
    "h")  print_help
          ;;
    "i")  print_actual_info
          ;;
    "b")  echo "Start autodetect boundary by motor cuurent consume."
          echo $((DEFAULT_CONFIG | START_AUTODETECT_BOUNDARY)) > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # start proces to find bounadary by amp
          ;;
    "d")  echo "Start autodetect speed."
          echo $((DEFAULT_CONFIG | ENABLE_AMP_BEGENIN | ENABLE_AMP_END | START_AUTODETECT_SPEED,CONFIG_FLAGS)) > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # start proces to autodetec max speed
          ;;
    "a")  echo "Start autodetect acceleration."
          echo $((DEFAULT_CONFIG | ENABLE_AMP_BEGENIN | ENABLE_AMP_END | START_AUTODETECT_ACCEL,CONFIG_FLAGS)) > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # start proces to autodetec max acceleration
          ;;
    "o")  echo "Set origin for absolute coordination:"
          cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure" > /dev/null # for coretly write new value must first read actual value (because device use shadow register who actualice by reading, otherway new value will be compared with no actual value)
          echo "0" > "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/actual_value" # set zero to absolute coordinate
          echo "OK"
          ;;
    "z")  echo "Set beginin trace (in absolute coordinaton):"
          value=$(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure") # get actual coordination
          echo "$value" > "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/min_alarm_value" # set begenin of trace
          echo "New value:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/min_alarm_value") # for check read stored begenin trace
          ;;
    "x")  echo "Set end trace (in absolute coordinaton):"
          value=$(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure") # get actual coordination
          echo "$value" > "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/max_alarm_value" # set end of trace
          echo "New value:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/max_alarm_value") # for check read stored begenin trace
          ;;
    "c")  echo "Switch Enable/Disable sowtware trace boundary:"
          value=$(cat "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value") # get actual falags config
          if [ $((value & $((ENABLE_SW_BEGENIN | ENABLE_SW_END)))) -eq $((ENABLE_SW_BEGENIN | ENABLE_SW_END)) ]
            then # actualy is enable -> disable
               value=$((value & $(( 0xFFFF ^ ENABLE_SW_BEGENIN)))) #prepare new value for disable sowtware check begenin
               value=$((value & $(( 0xFFFF ^ ENABLE_SW_END)))) #prepare new value for disable sowtware check begenin
               echo "$value" > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # set new config
               echo "Disable"
            else # actualy is disable -> enable
               value=$((value | ENABLE_SW_BEGENIN)) #prepare new value for disable sowtware check begenin
               value=$((value | ENABLE_SW_END)) #prepare new value for disable sowtware check begenin
               echo "$value" > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # set new config
               echo "Enable"
          fi
          ;;
    "e")  echo "Switch Disable(halt)/Enable motor:"
          value=$(cat "$DEVICE_FULL_PATH/$FORCE_HALT/actual_value") # get actual falags config
          if [ $value -eq 1 ]
            then # actualy is disable -> 0=enable
               value=0
               echo "Enable"
            else # actualy is enable -> 1=disable
               value=1
               echo "Disable"
          fi
          echo "$value" > "$DEVICE_FULL_PATH/$FORCE_HALT/actual_value" # set new config
          ;;
    "f")  echo -n "Store actual absolute coordination:"
          m_absolute_coordination=$(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure") # get actual coordination
          echo m_absolute_coordination
          ;;
    "g")  echo -n "Go to stored absolute coordination: $m_absolute_coordination" "  "
          echo "actual coordination:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure") # catualize actual coordination
          echo "$m_absolute_coordination" > "$DEVICE_FULL_PATH/$MOVE_ABSOLUTE/actual_value" # set new absolute coordination
          ;;
    "n")  echo -n "Relative left move steps:"  $((-1*$STEPS))
          echo $((-1*$STEPS)) > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "m")  echo -n "Relative right move steps:"  $STEPS
          echo $STEPS > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "N")  echo -n "Never ending left move:"
          echo $NEVER_ENDING_LEFT_MOVING > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "M")  echo -n "Never ending right move:"
          echo $NEVER_ENDING_RIGHT_MOVING > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "A")  echo "Switch enable/disable alarm on end of relative moving:"
          value=$(cat "$DEVICE_FULL_PATH/$MOVE_RELATIVE/"enable_min_alarm ) # get actual flag
          if [ $value -eq 1 ]
            then # actualy is ALARM IS ENABLED -> 0=OFF
               value=0
               echo "switch ALARM DISABLE"
            else # actualy is ALARM IS DISABLED -> 1=ON
               value=1
               echo "switch ALARM ENABLED"
          fi
          echo "$value" > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/enable_min_alarm" # set new value
          echo "$value" > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/enable_max_alarm" # set new value
          ;;
    "s")  echo -n "Save actual section values:"
          echo 1 > "$DEVICE_FULL_PATH/$SAVE/actual_value" # set new relative coordination
          ;;
    "l")  echo "Switch ON/OFF indication led:"
          value=$(cat "$DEVICE_FULL_PATH/$INFO_LED/actual_value") # get actual falags config
          if [ $value -eq 1 ]
            then # actualy is ON -> 0=OFF
               value=0
               echo "switch OFF"
            else # actualy is OFF -> 1=ON
               value=1
               echo "switch ON"
          fi
          echo "$value" > "$DEVICE_FULL_PATH/$INFO_LED/actual_value" # set new value
          ;;
  esac
  
done


