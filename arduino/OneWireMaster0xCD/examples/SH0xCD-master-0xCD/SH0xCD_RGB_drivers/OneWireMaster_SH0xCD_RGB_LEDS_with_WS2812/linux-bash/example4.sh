#!/bin/bash

# Example howto use 1-Wire slave device "SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds"
#    script use bash in linux with OWFS (One Wire File System - www.owfs.org)
#
# more at:
# https://github.com/seahu/seahu_CD
#
# Exmaple description:
# GRADUALLY FILL THE PIXELS WITH CHANGE COLOR ON RGB LED STRIP with WS2812 leds by 1-Wire bus with device "SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds"
# ----------------------------------------------------------------------------------------------------------------------------------------------
# Type device: 1-Wire
# Family code : 0xCD
# Device description: SEAHU 1-Wire controler for RGB LED STRIP with WS2812 leds
# Number sections: 6
#    section0 :  set number of pixels (every LED STIP contain diferent number of pixels, here must be set its number)
#    section1 :  actually color (as RGB number) apply if change color or position
#    section2 :  pixel position (0=all pixels)
#    section3 :  rotation abs(value)=shift rototion, value<0 => left rototion, value>0 => right rotation, value=0 => no rotation (immediately after set will be du and  reset to 0)
#    section4 :  control identification led 0=off, 1=on
#    section5 :  for development always 0. Write to 1 to acutal_value start save all  values from all sections. (for me after restart, device load default values from another project and switch on beeper)


# First you need run OWFS.
# OWFS mount virtual file system usually into directory /mnt/1wire and represented every 1-Wire slave device as directory and its registries as files.
DEVICE="/mnt/1wire/uncached/CD.201019001355" # find device directory and copy here its address

# Then you can control it.
R=0
G=50
B=10



mumPixels=60
add=30
wait=0.1

echo "$numPixels" > $DEVICE/section0/actual_value  # set numer pixels
echo "0" > $DEVICE/section2/actual_value   # select all pixels
echo "0" > $DEVICE/section1/actual_value   # select color

function color {
    R=$1
    G=$2
    B=$3
    color=$((R*65536+G*256+B))
    echo -n $color
}

color=$(color 50 50 50)

echo "1" > $DEVICE/section2/actual_value   # select  pixel
echo "$color" > $DEVICE/section1/actual_value   # select color
for i in $(seq 1 60);
do
    R=$(($R+add))
    color=$(color $R $G $B)
    echo "$i" > $DEVICE/section2/actual_value   # select  pixel
    echo "$color" > $DEVICE/section1/actual_value   # select color
    #sleep $wait
done
