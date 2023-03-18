# Plugin for Domoticz (one of best open source home automation system)

This project add support fro 1-Wire devices with family code 0xCD.
This devices is totally universally can be thermometer, humidity meter, switch, dimmer, .. 
plus may by contain more elements in one devices.

Features:
- plug and use - auto add new decvices
- auto rename devices by values stored into 1-Wire device
- auto assing device types by device description
- more devices in one device

Requirents:
- Domoticz for 1-Wire devices use OWFS (one wire file system), but orginal
  owfs do not support 1-Wire devices with familz code 0xCD 
  (the author OWFS has been ignoring this patch for at least two years).
  Therefore you need modified OWFS from: https://github.com/seahu/owfs

Install:
 - Copy directorz seahu_CD into direcory domoticz/plugins and restart domoticz.
 - From menu Setup->Hardware select "1-Wire familz code 0xCD (seahu) plugin and corect set OWFS Path