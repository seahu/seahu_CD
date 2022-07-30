#!/bin/bash

# Example howto use 1-Wire slave device "SEAHU SH025b 1-Wire unipolar stepter motor driver C2021         "
# This example show how to drive stepetr motor.
# Script use bash under linux with OWFS (One Wire File System - www.owfs.org)
#
# Exmaple description:
# EXAMPLE HOW-TO USE STEPTER MOTOR WITH SEAHU 1-Wire MOTOR DRIVER
# ---------------------------------------------------------------
# Type device: 1-Wire
# Family code : 0xCD
# Device description: SEAHU SH025b 1-Wire unipolar stepter motor driver C2021         
# Number sections: 9
#      section0 :  MoveRelative       - movving relative distance in steps
#                                        -2147483646 steps => never ending move to left
#                                        +2147483646 steps => never ending move to right
#      section1 :  MoveAbsolute       - movving absolute distance in steps
#      section2 :  Speed              - moving speed in steps/sec
#      section3 :  AbsoluteCoordinate - actual absolute coordinate
#      section4 :  TimeToDisableMotor - delay in us to turns off the engine after inactivity
#      section5 :  ConfigFlags        - configure bits falgs
#                                          1.bit (+1) - 0= normal motor direction, 1= inverse motor direction
#                                          2.-4.bit   - select step table used for stepter motor:
#                                            (+0) = one phase at a time, siples less torque less electric consume (32 steps per ratio and after gearbox 32x64=2048 steps per ratio)
#                                            (+2) = two hases at a time, more torque butmore eletric consume (32 steps per ratio and after gearbox 32x64=2048 steps per ratio)
#                                            (+4) = two or one phase, two times more  precise (smaller step) (64 steps per ratio and after gearbox 64x64=4096 steps per ratio)
#                                              4321       4321      4321 bit.
#                                             =000X      =001X     =010X
#                                             (+0)       (+2)      (+4)
#                                             A  B       A  B      A  B
#                                             00 01      00 11     00 01
#                                             00 10      01 10     00 11
#                                             01 00      11 00     00 10
#                                             10 00      10 01     01 10
#                                                                  01 00
#                                                                  11 00
#                                                                  10 00
#                                                                  10 01
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
#      section6  :  ForceHalt          - 0=do nothing, 1=halt moving
#
#      section7  :  led idetification  - identify of 1-Wire device 1=on 0-off (on board indication led share same pin as motr pin B1, therefore must be off to enable corect funtion of motor and can flash during motor move)
#      section8  :  Save               - 0=do nothing, 1=save actual section values
#
# more at:
# https:#github.com/seahu/seahu_CD


# First you need run OWFS.
# OWFS mount virtual file system usually into directory /mnt/1wire and represented every 1-Wire slave device as directory and its registries as files.




DEVICE="CD.211219232600" # find device directory and copy here its address
DEVICE_FULL_PATH="/mnt/1wire/uncached/$DEVICE"


# configure constatnats
REVERSE_DIRECTION=1
STEPS_TABLE=4

# defaul config and motor values - can set this values by oneself
DEFAULT_DIRECTION=0
DEFAULT_STEP_TABLE=4
DEFAULT_SPEED=50
DEFAULT_CONFIG=$(( DEFAULT_DIRECTION | DEFAULT_STEP_TABLE )) # use definitions from previous section - //configure constants 

# special vales for never ending values (for relative moving)
NEVER_ENDING_LEFT_MOVING=-2147483646
NEVER_ENDING_RIGHT_MOVING=2147483646

# define section number
MOVE_RELATIVE="section0"
MOVE_ABSOLUTE="section1"
SPEED_="section2"
ABSOLUTE_COORDINATE="section3"
TIME_TO_DISABLE_MOTOR="section4"
CONFIG_FLAGS="section5"
FORCE_HALT="section6"
INFO_LED="section7"
SAVE="section8"

# steps move to one key push on this example
STEPS=500

function print_help {
  echo "HELP"
  echo "----"
  echo "h - this help."
  echo "i - List actual setting."
  echo "A - Switch enable/disable alarm on end of relative moving."
  echo "o - Set origin for absolute coordination."
  echo "c - Switch Enable/Disable sowtware trace boundary."
  echo "e - Switch Disable(halt)/Enable motor."
  echo "f - Store actual absolute coordination."
  echo "g - Go to stored absolute coordination."
  echo "n - Relative left move steps."
  echo "m - Relative right move steps."
  echo "N - Never ending left move."
  echo "M - Never ending right move."
  echo "z - Decrease speed -100."
  echo "x - Decrease speed -10."
  echo "v - Incerase speed +10."
  echo "b - Incerase speed +100."
  echo "p - Change step table."
  echo "t - Switch ON/OFF hlat motor status."
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
  echo "Min alarm value:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/min_alarm_value")
  echo "Max alarm value:" $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/max_alarm_value")
  stat=$(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/control_byte") #get control byte for ABSOLUTE_COORDINATE section for check if is set alarm for this section
  echo "Is set alarm for absolute min_alarm_value:" $(flag_comparation $stat 2#01000000)
  echo "Is set alarm for absolute max_alarm_value:" $(flag_comparation $stat 2#10000000)
  stat=$(cat "$DEVICE_FULL_PATH/$MOVE_RELATIVE/control_byte") #get control byte for MOVE_RELATIVE section for check if is set alarm for this section
  echo "Is set alarm for end relative moving:" $(flag_comparation $stat 2#01100000)
  echo "Actual absolute coorinate:"  $(cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure")
  echo "Speed:"  $(cat "$DEVICE_FULL_PATH/$SPEED_/actual_value")
  echo "Time to disable motor after n activity:"  $(cat "$DEVICE_FULL_PATH/$TIME_TO_DISABLE_MOTOR/actual_value")
  echo "Status of info led:"  $(cat "$DEVICE_FULL_PATH/$INFO_LED/actual_value")
  echo "Status of halt motor status:" $(cat "$DEVICE_FULL_PATH/$FORCE_HALT/actual_value")
}


function setup {
  echo $DEFAULT_SPEED > "$DEVICE_FULL_PATH/$SPEED/actual_value" # set speed
  print_help
}

print_help
while :
do
  read -p "command:" cmd
  case "$cmd" in
    "h")  print_help
          ;;
    "i")  print_actual_info
          ;;
    "o")  echo "Set origin for absolute coordination:"
          cat "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/measure" > /dev/null # for coretly write new value must first read actual value (because device use shadow register who actualice by reading, otherway new value will be compared with no actual value)
          echo "0" > "$DEVICE_FULL_PATH/$ABSOLUTE_COORDINATE/actual_value" # set zero to absolute coordinate
          echo "OK"
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
    "n")  echo  "Relative left move steps:"  $((-1*$STEPS))
          echo $((-1*$STEPS)) > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "m")  echo  "Relative right move steps:"  $STEPS
          echo $STEPS > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "N")  echo  "Never ending left move:"
          echo $NEVER_ENDING_LEFT_MOVING > "$DEVICE_FULL_PATH/$MOVE_RELATIVE/actual_value" # set new relative coordination
          ;;
    "M")  echo  "Never ending right move:"
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
    "s")  echo  "Save actual section values:"
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
    "t")  echo -n "Switch ON/OFF hlat status motor:"
          value=$(cat "$DEVICE_FULL_PATH/$FORCE_HALT/actual_value") # get actual valu of FORCE_HALT
          if [ $value -eq 1 ]
            then # actualy is ON -> 0=OFF
               value=0
               echo "switch OFF"
            else # actualy is OFF -> 1=ON
               value=1
               echo "switch ON"
          fi
          echo "$value" > "$DEVICE_FULL_PATH//$FORCE_HALT/actual_value" # set new value
          ;;
    "r")  echo -n "Reverse motor direction ON/OFF:"
          value=$(cat "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value") # get actual valu of FORCE_HALT
          val=$(( $value & 1 ))
          if [ $val -eq 1 ]
            then # actualy is ON -> 0=OFF
               value=$(( $value & ~1 ))
               echo "Reverse motor direction OFF"
            else # actualy is OFF -> 1=ON
               value=$(( $value | 1 ))
               echo "Reverse motor direction ON"
          fi
          echo "$value" > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # set new value
          ;;
    "p")  echo -n "Change step table to:"
          value=$(cat "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value") # get actual valu of FORCE_HALT
          val=$(( $value >> 1 ))
          val=$(( $val & 7 ))
          val=$(( $val + 1 ))
          if [ $val -ge 3 ]
            then # device has only three types of step table
               val=0;
          fi
          echo "$val"
          value=$(( $value & ~15 )) # clear space for new value
          val=$(( $val << 1 )) # move nuber step table to one bit left
          value=$(( $value | $val )) # complete new value
          echo "$value" > "$DEVICE_FULL_PATH/$CONFIG_FLAGS/actual_value" # set new value
          ;;
    "z")  echo -n "SPEED: speed -100="
          value=$(cat "$DEVICE_FULL_PATH/$SPEED_/actual_value") # get actual valu of FORCE_HALT
          value=$(( $value - 100 ))
          if [ $value -lt 0 ]
            then # speed can not be less than zero
               value=0
          fi
          echo "$value" > "$DEVICE_FULL_PATH/$SPEED_/actual_value" # set new value
          echo "$value"
          ;;
    "x")  echo -n "SPEED: speed -10="
          value=$(cat "$DEVICE_FULL_PATH/$SPEED_/actual_value") # get actual valu of FORCE_HALT
          value=$(( $value - 10 ))
          if [ $value -lt 0 ]
            then # speed can not be less than zero
               value=0
          fi
          echo "$value" > "$DEVICE_FULL_PATH/$SPEED_/actual_value" # set new value
          echo "$value"
          ;;
    "v")  echo -n "SPEED: speed +10="
          value=$(cat "$DEVICE_FULL_PATH/$SPEED_/actual_value") # get actual valu of FORCE_HALT
          value=$(( $value + 10 ))
          echo "$value" > "$DEVICE_FULL_PATH/$SPEED_/actual_value" # set new value
          echo "$value"
          ;;
    "b")  echo -n "SPEED: speed +100="
          value=$(cat "$DEVICE_FULL_PATH/$SPEED_/actual_value") # get actual valu of FORCE_HALT
          value=$(( $value + 100 ))
          echo "$value" > "$DEVICE_FULL_PATH/$SPEED_/actual_value" # set new value
          echo "$value"
          ;;
  esac
  
done


