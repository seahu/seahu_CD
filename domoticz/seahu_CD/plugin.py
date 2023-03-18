# Python Plugin for 1-Wire devices with family code 0xCD
#
# Author: Ondrej Lycka
#

# main problem this plugin is map 1-Wire 0xCD devices who may contain one from four datatypes values (bool, int32, unsigned int32, nad text) to totaly confused 
# data types used in domoticz whose use diferent metoth to store data etc, in nValue , svalue, Color and his combinations
#
# Schma to data flowing:
#
# set ( domoticz->hw )
# -------------------
# 
# onCommand(DeviceID, Unit, Command, Level, Color)
#           called if change switch
#           Commnad - (no documented) by my research may contain for devices type:
#               switch - "on","off"
#               custom sensor - (no call)
#           level - ?
#           color - 
#
# onDeviceModified(DeviceID, Unit)
#           called if change custom sensor, new value must read from unit.nValue, unit.sValue, unit.Color
#
# get ( hw->domoticz )
# --------------------
# direct write into unit
# Devices[DeviceID].Units[Unit].nValue = nValue (number)
# Devices[DeviceID].Units[Unit].sValue = sValue (string)
# Devices[DeviceID].Units[Unit].Color = Color  ({ m:0..4 t:0..255, r:0..255, g:0..255, b:0..255, cw:0..255, ww:0..255 })
#

"""
<plugin key="SeahuPlug" name="1-Wire family code 0xCD (seahu) plugin" author="Ondrej Lycka" version="1.0.0" wikilink="http://www.domoticz.com/wiki/plugins/plugin.html" externallink="https://github.com/seahu/seahu_CD">
    <description>
        <h2>1-Wire multi sensor/actor slaves devices with family code 0xCD</h2><br/>
        <p>
            This plugin add support for 1-wire devices with family code 0xCD who use arduino open source multi sensors/actors library.
            This devices can contain more sensors, actors or setings options e.g. temparature, humidity,
            presure, switch, dimmer, RGB, RGBW, counter, ...  or setting options for example hystersion at termostat, ...
            Thanks this aruino librray is very easy cerate own 1-Wire devices with  different types sensors and actors.
        </p>
        <p>
            Every device has own 1-Wire address and every sensor, actor or setting otions inside the device have own section.
            This 1-wire devices contain additional informatin as device description, sections description,
            user notes for sections, section data type, actual value section,.. Some of this infromtion is used for generate name or description its representation in domoticz.
        </p>
        <p>
            More at https://github.com/seahu/seahu_CD
        </p>
        <p>
            This plugin use linux 1-Wire owfs (one wire file system), but official owfs not support 1-Wire devices with this family code.
            Owfs with support this device is aviable at https://github.com/seahu/owfs.
        </p>
        <p>
            PS: in original owfs is pull request with support this devices since 2020, but autor ingnore this reguest.
        </p>
        <h3>Section may contain next types value:</h3>
        <ul style="list-style-type:square">
            <li>32B binary or 32B text -- maped as domoticz text</li>
            <li>bool                   -- maped as domoticz switch On/Off</li>
            <li>un/signed 32bit        -- maped as domoticz custom sensor or more specify by section description</li>
        </ul>
        <h3>Currently, for the section data type signed/unsigned 32bit , the following device types are distinguished according to the section description: </h3>
        <ul style="list-style-type:square">
            <li>Section description -- domoticz type</li>
            <li>RGB -- Color Switch RGB</li>
            <li> RGBW -- Color Switch RGBW</li>
            <li>Temperature -> Temp</li>
            <li>Procentage -> General Percentage</li>
            <li>Distance -> General Distance</li>
            <li>Counter -> Counter</li>
            <li>Volt -> General Voltage</li>
            <li>Current -> General Ampera (1 phase)</li>
            <li>Termostat -> Thermostat Setpoint</li>
        </ul>
        <p>
            The plugin create for every sections separate domoticz device. Plus for more transparenty crete one domoticz device
            with textual descritption of whole device.
        </p>
        <p>
            For creating domoticz device is used section user_note for domoticz device name. If is empty for domoticz device name is used section descrtiotin.
            Raname domoticz name is stored into section user_note.
        </p>
        <p>
            In domoticz device list ID=1-wire address and Unit=section+1 (because Unit=0 is not allowed, section0=Unit 1, section1=Unit 2,..)..
            Plus extra Unit 100 with text description whole device.
        </p>
        <h3>Features</h3>
        <ul style="list-style-type:square">
            <li>Auto add new devices</li>
            <li>Multi sensors and actors in one 1-Wire device</li>
            <li>Store domoticz device name direct into 1-Wire device</li>
            <li>Spoort 1-wire alarm (for faster response)</li>
        </ul>
    </description>
    <params>
        <param field="Address" label="OWFS Path" width="200px" required="true" default="/mnt/1wire"/>
        <param field="Mode1" label="Sensor period (s) integer number" width="200px" required="true" default="10"/>
        <param field="Mode2" label="Alarm pool period (s) float number" width="200px" required="true" default="0.2"/>
        <param field="Mode3" label="New devices search period (s) integer number" width="200px" required="true" default="180"/>
        <param field="Mode6" label="Debug (0 or 1)" width="150px" >
            <options>
                <option label="None" value="0"  default="true" />
                <option label="Basic" value="2"/>
                <option label="Debugging" value="10"/>
            </options>
        </param>
    </params>
</plugin>
"""
import os
import DomoticzEx as Domoticz
import queue
import sys
import time
import threading
import json

OWFS_path="mnt/1wire"
Sensor_period=10
Alarm_period=0.2
New_devices_period=180
lock=threading.Lock() #protect critical part of code for write/read new values
QueueTimeOut=0.1

class SeahuPlugin:
    enabled = False
    last_sensor_period=0
    last_alarm_period=0
    last_new_devices_period=0

    def __init__(self):
        self.messageQueue = queue.Queue()
        self.messageThread = threading.Thread(name="QueueThread", target=SeahuPlugin.handleMessage, args=(self,))

    def handleMessage(self):
        # main function in this main loop of separate theader is control alarm
        # and handle exit thedaer during corect ending this  plugin
        try:
            Domoticz.Log("Entering message handler")
            while True:
                try:
                    Message = self.messageQueue.get(block=True, timeout=QueueTimeOut)
                    #Message = self.messageQueue.get(block=True, timeout=5) 
                    Domoticz.Debug("Tu jsem")
                    print("tu jsem")
                    if Message is None:
                        Domoticz.Debug("Exiting message handler")
                        self.messageQueue.task_done()
                        break
                    # this is place for to catch content of message etc. if (Message["Type"] == "Log"): do somthing
                    self.messageQueue.task_done()
                except queue.Empty: # messageQueue.get timeout -> return queue.Empty exception
                    #Log(10, "Queue is empty.")
                    now=time.time()
                    if ( (now-self.last_sensor_period) > Sensor_period ):           #---- check sensors
                        Log(10, "Check sensors.")
                        for DeviceID in Devices:
                            for Unit in Devices[DeviceID].Units:
                                if Unit==100 : continue # unit 100 is only device description no real section
                                get_value_from_device(DeviceID, Unit)
                        self.last_sensor_period=time.time()
                    if ( (now-self.last_alarm_period) > Alarm_period ):             #---- check alarm
                        Log(10, "Check alarm.")
                        check_alarm_device()
                        self.last_alarm_period=time.time()
                    if ( (now-self.last_new_devices_period) > New_devices_period ): #---- check new devices
                        Log(10, "New devices.")
                        #check_new_devices()
                        self.last_new_devices_period=time.time()

        except Exception as err:
            Domoticz.Error("handleMessage: "+str(err))

    def onStart(self):
        global OWFS_path
        global Sensor_period
        global Alarm_period
        global New_devices_period

        Log(10, "onStart called")
        OWFS_path=Parameters["Address"]
        Sensor_period=int(Parameters["Mode1"])
        Alarm_period=float(Parameters["Mode2"])
        New_devices_period=int(Parameters["Mode3"])
        # calculate optimal queue timeout from the shortes period
        QueueTimeOut=New_devices_period
        if QueueTimeOut>Sensor_period : QueueTimeOut=Sensor_period
        if QueueTimeOut>Alarm_period  : QueueTimeOut=Alarm_period
        QueueTimeOut=QueueTimeOut/2

        if Parameters["Mode6"] == "11":
            Domoticz.Debugging(int(Parameters["Mode6"]))
            DumpConfigToLog()
            Domoticz.Debugging(1)
            Domoticz.Log("Debugger started, use 'telnet 0.0.0.0 4444' to connect")
            import rpdb
            rpdb.set_trace()
        check_new_devices()
        self.messageThread.start()
        #Domoticz.Heartbeat(1) # heardbeat 
        Log(10, "do log")
        DumpConfigToLog()

    def onStop(self):
        Log(10, "onStop called")
        # Not needed in an actual plugin
        for thread in threading.enumerate():
            if (thread.name != threading.current_thread().name):
                Domoticz.Log("'"+thread.name+"' is running, it must be shutdown otherwise Domoticz will abort on plugin exit.")

        # signal queue thread to exit
        self.messageQueue.put(None)
        Domoticz.Log("Clearing message queue...")
        self.messageQueue.join()

        # Wait until queue thread has exited
        Domoticz.Log("Threads still active: "+str(threading.active_count())+", should be 1.")
        while (threading.active_count() > 1):
            for thread in threading.enumerate():
                if (thread.name != threading.current_thread().name):
                    Domoticz.Log("'"+thread.name+"' is still running, waiting otherwise Domoticz will abort on plugin exit.")
            time.sleep(1.0)

    def onConnect(self, Connection, Status, Description):
        Log(10, "onConnect called")

    def onMessage(self, Connection, Data):
        Log(10,"onMessage called")

    def onCommand(self, DeviceID, Unit, Command, Level, Color):
        Log(10, "onCommand called for Device " + str(DeviceID) + " Unit " + str(Unit) + ": Parameter '" + str(Command) + "', Level: " + str(Level) + ", Color:" + str(Color) )
        Command = Command.strip()
        # I suppport only comands: "On", "Off", "Set Color"
        action, sep, params = Command.partition(' ')
        Log(10, "action="+action)
        Log(10, "sep="+sep)
        Log(10, "parmas="+params)
        if (action=="On"):
            sValue=action
            nValue=1
            Color=""
        elif (action=="Off"):
            sValue=action
            nValue=0
        elif (action=="Set"):
            if (params=="Color"):
                Log(10, "params:Color")
                sValue="On"
                nValue=1
                Color=Color
            elif (params=="Level"):
                Log(10, "params:Level")
                sValue=str(Level)
                nValue=1
                Color=Color
            else:
                Log(10, "UNKNOWN parmas:"+params)
                return
        else:
            Log(10, "UNKNOWN action:"+action)
            return


        set_value_to_device(nValue, sValue, Color, DeviceID, Unit)

    def onNotification(self, Name, Subject, Text, Status, Priority, Sound, ImageFile):
        Log(10, "Notification: " + Name + "," + Subject + "," + Text + "," + Status + "," + str(Priority) + "," + Sound + "," + ImageFile)
        Log(10, "Unit onDeviceModified for "+str(self.Name))

    def onDeviceModified(self, DeviceID, Unit):
        Log(10, "Modified (name, value): DeviceID"+ str(DeviceID)+" Unit:"+str(Unit))
        Log(10, "Check name: DeviceID"+ str(DeviceID)+" Unit:"+str(Unit))
        Name=Devices[DeviceID].Units[Unit].Name
        user_note=clear_text(get_user_note(DeviceID, Unit))
        if Name[:32] != user_note :
            set_user_note(Name, DeviceID, Unit)
            Log(10, "DeviceID"+ str(DeviceID)+" Unit:"+str(Unit)+" Name:"+Name+" - CHANGE NAME")

        nValue=Devices[DeviceID].Units[Unit].nValue # "Custom Sensor" use onlu sValue
        sValue=Devices[DeviceID].Units[Unit].sValue # "Custom Sensor" use onlu sValue
        Color=Devices[DeviceID].Units[Unit].Color # "Custom Sensor" use onlu sValue
        Log(10, "Send actual value to DeviceID"+ str(DeviceID)+" Unit:"+str(Unit)+" sValue:"+str(sValue))
        set_value_to_device(nValue, sValue, Color, DeviceID, Unit)

    def onDisconnect(self, Connection):
        Log(10, "onDisconnect called")

    def onHeartbeat(self):
        return
        Log(10, "onHeartbeat called")
        Log(10, str(Devices))
        for DeviceID in Devices:
            for Unit in Devices[DeviceID].Units:
                get_value_from_device(DeviceID, Unit)


global _plugin
_plugin = SeahuPlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onConnect(Connection, Status, Description):
    global _plugin
    _plugin.onConnect(Connection, Status, Description)

def onMessage(Connection, Data):
    global _plugin
    _plugin.onMessage(Connection, Data)

def onCommand(DeviceID, Unit, Command, Level, Color):
    global _plugin
    _plugin.onCommand(DeviceID, Unit, Command, Level, Color)

def onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile):
    global _plugin
    _plugin.onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile)

def onDeviceModified(DeviceID, Unit):
    global _plugin
    _plugin.onDeviceModified(DeviceID, Unit)

def onDisconnect(Connection):
    global _plugin
    _plugin.onDisconnect(Connection)    

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

# Generic helper functions
def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Log(10, "'" + x + "':'" + str(Parameters[x]) + "'")
    Log(10, "Device count: " + str(len(Devices)))
    for DeviceName in Devices:
        Device = Devices[DeviceName]
        Log(10, "Device ID:       '" + str(Device.DeviceID) + "'")
        Log(10, "--->Unit Count:      '" + str(len(Device.Units)) + "'")
        for UnitNo in Device.Units:
            Unit = Device.Units[UnitNo]
            Log(10, "--->Unit:           " + str(UnitNo))
            Log(10, "--->Unit Name:     '" + Unit.Name + "'")
            Log(10, "--->Unit nValue:    " + str(Unit.nValue))
            Log(10, "--->Unit sValue:   '" + Unit.sValue + "'")
            Log(10, "--->Unit LastLevel: " + str(Unit.LastLevel))
    return

def pokus():
   Log(10, "POKUS")
   return

def Log(level, text):
    if level <= int(Parameters["Mode6"]):
        Domoticz.Log(text)

def clear_text(text):
    new_text=""
    for char in text:
        if ord(char)==0 : break
        new_text=new_text+char
    return new_text

def read_from_owfs(path):
    file=OWFS_path+"/"+path
    Log(10, "Read from file:"+ file)
    with open(file, 'r',encoding="utf-8", errors='ignore') as f: val=f.read()
    return val

def write_to_owfs(text, path):
    Log(10, "Write to file:"+ path + " Value:" + text)
    file=OWFS_path+"/"+path
    Log(10, "Write to file:"+ file + " Value:" + text)
    f=open(file, 'w')
    f.write(text)
    f.close()


def get_control_byte(DeviceID, Unit):
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" - GET CONTROL BYTE")
    path = str(DeviceID)+"/section"+str(Unit-1)+"/control_byte";
    control_byte=int(read_from_owfs(path))
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" control_byte:"+str(control_byte))
    return control_byte

def get_actual_value(DeviceID, Unit, alarm=False):
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" - GET ACTUAL VALUE")
    if alarm==True :
        path = "uncached/"+str(DeviceID)+"/section"+str(Unit-1)+"/actual_value";
    else :
        path = str(DeviceID)+"/section"+str(Unit-1)+"/actual_value";
    actual_value=int(read_from_owfs(path))

    # hook for OWFS and biger nuber (OWFS use only signed integer 32b) this convert signed int32 to unsigned int32 
    control_byte=get_control_byte(DeviceID, Unit)
    Log(10, "Section contol_byte fro OWFS hook: " + str(control_byte))
    SeahuType=control_byte & 0b00011000
    if SeahuType==0b00010000 : #unsign int -> apply hook
        if actual_value<0:
            Log(10, "Apply OWFS hook for convert signed int32 to unsined int32 actaul value: " + str(control_byte))
            actual_value=actual_value+4294967296
    #end hook

    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" actual_value:"+str(actual_value))
    return actual_value

def get_user_note(DeviceID, Unit, alarm=False):
    Log(10, "DeviceID "+ str(DeviceID)+" section:"+str(Unit-1)+" -GET USER NOTE")
    if alarm==True :
        path = "uncached/"+str(DeviceID)+"/section"+str(Unit-1)+"/user_note";
    else :
        path = str(DeviceID)+"/section"+str(Unit-1)+"/user_note";
    user_note=read_from_owfs(path)
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" user_note("+ str(len(user_note))+"):"+user_note)
    return user_note

def get_device_description(DeviceID):
    Log(10, "DeviceID:"+ str(DeviceID)+" - GET DEVICE DESCRIPTION")
    path = str(DeviceID)+"/device_description";
    description = clear_text( read_from_owfs(path) )
    Log(10, "DeviceID:"+ str(DeviceID)+" Device_description("+str(len(description))+"):"+description)
    return description

def get_description(DeviceID, Unit):
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" - GET_DESCRIPTION")
    path = str(DeviceID)+"/section"+str(Unit-1)+"/description";
    description = clear_text( read_from_owfs(path) )
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" Description("+ str(len(description))+"):"+description)
    return description

def get_section_alarm_status(DeviceID, Unit):
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" - GET_ALARM_STATUS")
    path = "uncached/"+str(DeviceID)+"/section"+str(Unit-1)+"/status_alarm";
    status_alarm = int( read_from_owfs(path) )
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" status_alarm:"+ str(status_alarm))
    return status_alarm


def set_actual_value(Value, DeviceID, Unit):
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" new_value:"+str(Value)+" - SET ACTUAL VALUE")
    path = str(DeviceID)+"/section"+str(Unit-1)+"/actual_value";
    # hook for OWFS and biger nuber (OWFS use only signed integer 32b) this convert usigned int32 to signed int32 
    if Value>2147483647: 
            control_byte=get_control_byte(DeviceID, Unit)
            Log(10, "Section contol_byte fro OWFS hook: " + str(control_byte))
            SeahuType=control_byte & 0b00011000
            if SeahuType==0b00010000 : #unsign int -> apply hook
                Log(10, "Apply OWFS hook for convert unsigned int32 to signed int32 actaul value: " + str(control_byte))
                Value=Value-4294967296 
    # hook end
    write_to_owfs(str(Value), path)

def set_user_note(Value, DeviceID, Unit):
    Log(10, "DeviceID:"+ str(DeviceID)+" section:"+str(Unit-1)+" new_value:"+str(Value)[:32]+" - SET USER NOTE")
    path = str(DeviceID)+"/section"+str(Unit-1)+"/user_note";
    write_to_owfs(str(Value)[:32], path) # user_note has max 32 chars

def store_device_values(nValue, sValue, Color, DeviceID, Unit):
    change=0
    Color=Color.replace("'", "\"")
    if nValue!=None:
        if Devices[DeviceID].Units[Unit].nValue != nValue :
            Log(10, "STORE nValue:"+str(nValue))
            Devices[DeviceID].Units[Unit].nValue = nValue
            change=1
    if not(sValue=="" or sValue==None):
        if Devices[DeviceID].Units[Unit].sValue != sValue :
            Log(10, "STORE sValue:"+sValue)
            Devices[DeviceID].Units[Unit].sValue = sValue
            change=1
    if not(Color=="" or Color==None):
        if Devices[DeviceID].Units[Unit].Color != Color :
            Log(10, "STORE Color:"+Color)
            Devices[DeviceID].Units[Unit].Color = Color
            change=1
    if change==1 :
        Devices[DeviceID].Units[Unit].Update(Log=True)
        Devices[DeviceID].Units[Unit].Refresh()
        action="UPDATE"
    else :
        Devices[DeviceID].Units[Unit].Touch()
        action="TOUCH"
    Log(10, "DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" nValue:"+str(nValue)+" sValue"+sValue+" - ACTION "+action)


def set_value_to_device(nValue, sValue, Color, DeviceID, Unit):
    lock.acquire()
    Log(10, "DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" nValue:"+str(nValue)+" sValue:"+str(sValue)+" Color:"+str(Color)+" - SET VALUE TO DEVICE" )
    Type=Devices[DeviceID].Units[Unit].Type
    SubType=Devices[DeviceID].Units[Unit].SubType
    try:
        # --- TEXT ---
        if Type==243 and SubType==19 : # TypeName=Text
            set_user_note(sValue, DeviceID, Unit)
        # --- ON/OFF ---
        elif Type==244 and SubType==73: # TypeName=On/Off
            TypeName="On/Off"
            sValue=sValue.capitalize()
            if (sValue=="On" or sValue=="1" ) :
                nValue=1
            elif (sValue=="Off" or sValue=="0" ):
                nValue=0
            else:
                Domoticz.Log("Dimmer Value ("+sValue+") is not On/Off or 1/0 -> udate dimmer is ignored !")
                return
            set_actual_value(nValue, DeviceID, Unit)
        # --- CUSTOM SENSOR ---
        elif Type==243 and SubType==31: # TypeName=Custom
            set_actual_value(int(sValue), DeviceID, Unit)
        # --- COLOR ---
        elif Type==241 : # Color switch
            Log(10, "Color:"+str(Color))
            if sValue=="Off" or nValue==0: 
                set_actual_value(0, DeviceID, Unit) # for action sValue==Off only trun off RGB leds and do not change color in domoticz
            elif sValue=="On" or nValue>0: 
                if Color=="" or Color==None:
                    Color=Devices[DeviceID].Units[Unit].Color # if not enterd new value then fetch last value stored in domoticz db
                    if Color=="": 
                        if SubType==2 : # RGB
                            # place for change format store new value
                            Color="{\"m\":3, \"t\":255, \"r\":255, \"g\":255, \"b\":255, \"cw\":255, \"ww\":255}"
                        elif SubType==6 : # RGBW
                            # place for chnage format store new value
                            Color="{\"m\":4, \"t\":255, \"r\":255, \"g\":255, \"b\":255, \"cw\":255, \"ww\":255}"
                        else:
                            Domoticz.Log("Color switch SubType ("+str(SubType)+") is not 2 or 6 -> udate Color Switch is ignored !")
                            return
                    Log(10, "Color from db:"+str(Color))
                # device is "On"
                # prepare rgb
                Color = json.loads(Color)
                Log(10, "Color[b]:"+str(Color["b"]))
                r=int(Color["r"])
                g=int(Color["g"])
                b=int(Color["b"])
                ww=int(Color["ww"])
                cw=int(Color["cw"])
                w=int((ww+cw)/2)
                if SubType==2 : # RGB
                    colorValue = ( r << 16 ) + ( g << 8 ) + b
                if SubType==6 : # RGBW
                    colorValue = ( w << 24 ) + ( r << 16 ) + ( g << 8 ) + b
                set_actual_value(colorValue, DeviceID, Unit)
        # -- TEMPERATURE --
        elif Type==80 and SubType==5:
            set_actual_value(int(sValue), DeviceID, Unit)
        # -- HUMIDITY --
        elif Type==81 and SubType==1:
            set_actual_value(int(sValue), DeviceID, Unit)
        # -- PERCENTAGE --
        elif Type==243 and SubType==6:
            set_actual_value(int(sValue), DeviceID, Unit)
        # -- DISTANCE --
        elif Type==243 and SubType==27:
            set_actual_value(nValue, DeviceID, Unit)
        # -- COUNTER --
        elif Type==113 and SubType==0:
            set_actual_value(int(sValue), DeviceID, Unit)
        elif Type==243 and SubType==8:
            set_actual_value(int(sValue), DeviceID, Unit)
        # -- AMPERE --
        elif Type==243 and SubType==23:
            set_actual_value(int(sValue), DeviceID, Unit)
        # -- THERMOSTAT --
        elif Type==242 and SubType==1:
            set_actual_value(int(sValue), DeviceID, Unit)

        store_device_values(nValue, sValue, str(Color), DeviceID, Unit)
        Log(10, "DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" Type:"+str(Type)+" SubType:"+str(SubType)+" is set to nValue:"+str(nValue)+" sValue:"+sValue+" Color:"+str(Color))

    except:
        Domoticz.Log("DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" nValue:"+str(nValue)+" sValue:"+str(sValue)+" nValue:"+str(nValue)+" Color:"+str(Color)+" - ERROR TO SET VALUE" )
    lock.release()

def get_value_from_device(DeviceID, Unit, alarm=False ):
    lock.acquire()
    Log(10, "DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" - GET VALUE FROM DEVICE" )
    Type=Devices[DeviceID].Units[Unit].Type
    SubType=Devices[DeviceID].Units[Unit].SubType
    try:
        # --- TEXT ---
        if Type==243 and SubType==19 : # TypeName=Text
            sValue=get_user_note(DeviceID, Unit, alarm)
            nValue = len(sValue)
            Color = ""
        # --- ON/OFF ---
        elif Type==244 and SubType==73: # TypeName=ON/Off
            nValue=get_actual_value(DeviceID, Unit, alarm)
            if   (nValue==0) : sValue="Off"
            elif (nValue==1) : sValue="On"
            else:
                Domoticz.Log("Dimmer nValue ("+str(Value)+") is not 1/0 -> udate dimmer is ignored !")
                return
            Color = ""
        # --- CUSTOM SENSOR ---
        elif Type==243 and SubType==31: # TypeName=Custom
            nValue=get_actual_value(DeviceID, Unit, alarm)
            sValue = str(nValue)
            Color = ""
        # --- COLOR ---
        elif Type==241 : # Color switch
            rgbwValue=get_actual_value(DeviceID, Unit, alarm)
            if int(rgbwValue)==0: 
                # dicece is "Off" do not change color in domoticz db
                nValue=0;
                sValue="Off";
                Color=Devices[DeviceID].Units[Unit].Color
            else:
                # device is "On"
                nValue=1
                sValue="On"
                #prepare color values
                w=(int(rgbwValue) & 0xFF000000 ) >> 24
                r=(int(rgbwValue) & 0x00FF0000 ) >> 16
                g=(int(rgbwValue) & 0x0000FF00 ) >> 8
                b=(int(rgbwValue) & 0x000000FF )
                t=255 # max color temperature
                if SubType==2 : # RGB
                    # place for change format store new value
                    Color={"m":3, "t":t, "r":r, "g":g, "b":b, "cw":w, "ww":w}
                elif SubType==6 : # RGBW
                    # place for chnage format store new value
                    Color={"m":4, "t":t, "r":r, "g":g, "b":b, "cw":w, "ww":w}
                else:
                    Domoticz.Log("Color switch SubType ("+str(SubType)+") is not 2 or 6 -> udate Color Switch is ignored !")
                    return
        # -- TEMPERATURE --
        elif Type==80 and SubType==5:
            nValue = get_actual_value(DeviceID, Unit, alarm)
            sValue = str(nValue)
            Color = ""
        # -- HUMIDITY --
        elif Type==81 and SubType==1:
            nValue = get_actual_value(DeviceID, Unit, alarm)
            sValue = str(nValue)
            Color = ""
        # -- PERCENTAGE --
        elif Type==243 and SubType==6:
            nValue = get_actual_value(DeviceID, Unit, alarm)
            sValue = str(nValue)
            Color = ""
        # -- DISTANCE --
        elif Type==243 and SubType==27:
            nValue = get_actual_value(DeviceID, Unit, alarm)
            sValue = ""
            Color = ""
        # -- COUNTER --
        elif Type==113 and SubType==0:
            nValue=0
            sValue = str(get_actual_value(DeviceID, Unit, alarm))
            Color = ""
        # -- VOLTAGE --
        elif Type==243 and SubType==8:
            nValue = get_actual_value(DeviceID, Unit, alarm)
            sValue = str(nValue)
            Color = ""
        # -- AMPERE --
        elif Type==243 and SubType==23:
            nValue = get_actual_value(DeviceID, Unit, alarm)
            sValue = str(nValue)
            Color = ""
        # -- THERMOSTAT --
        elif Type==242 and SubType==1:
            Value = get_actual_value(DeviceID, Unit, alarm)
            nValue = 0
            sValue = str(Value)
            Color = ""

        Log(10, "STORE VALUE to DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" Type:"+str(Type)+" SubType:"+str(SubType)+" nValue:"+str(nValue)+" sValue"+sValue+" Color"+str(Color))
        store_device_values(nValue, sValue, str(Color), DeviceID, Unit)
        Log(10, "GET ACTUAL VALUE for DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" Type:"+str(Type)+" SubType:"+str(SubType)+"nValue:"+str(nValue)+" sValue"+sValue+" Color"+str(Color))

    except:
        Domoticz.Log("DeviceID:"+str(DeviceID)+" Unit:"+str(Unit)+" - ERROR GET VALUE" )
    lock.release()

def check_alarm_device():
    try:
        listDir=os.listdir(OWFS_path+"/alarm")
    except:
        Log(10, "Directory '"+OWFS_path+"' does not exist.")
        return

    for file in listDir:
        if file[0:2]!="CD" : continue # is not family CD 1-wire device
        DeviceID=file
        if DeviceID not in Devices : continue # device is not registered in domoticz
        for section in os.listdir(OWFS_path+'/bus.1/'+DeviceID):
            if section[0:7]=="section" :
                section_number=section[7:]
                Domoticz.Log("Sectin: "+ section_number)
                Unit=int(section_number)+1
                if Unit in Devices[DeviceID].Units:
                    if get_section_alarm_status(DeviceID, Unit)==1: #is alarm -> read new actual value
                        Log(10, "Alarm on DeiceID:"+DeviceID+" Unit:"+str(Unit))
                        get_value_from_device(DeviceID, Unit, True)


def check_new_devices():
    Log(10, "Check new devices.")
    for file in os.listdir(OWFS_path+"/bus.1"):
        if file=="alarm" : continue
        if file=="interaface" : continue
        if file[0:2]!="CD" : continue # is not family CD 1-wire device
        # file is family CD 1-wire device -> check his setions and his exist in domotic
        DeviceID=file
        Log(10, "Device: " + DeviceID)
        if DeviceID not in Devices:
            Description=get_device_description(DeviceID)
            Domoticz.Unit(Name=Description, Unit=100, TypeName="Text", DeviceID=DeviceID, Description=Description).Create()
            Devices[DeviceID].Units[100].sValue = Description
            Devices[DeviceID].Units[100].Update(Log=True)
            for section in os.listdir('/mnt/1wire/bus.1/'+file):
                if section[0:7]!="section" : continue
                section_number=section[7:]
                Domoticz.Log("Sectin: "+ section_number)
                Unit=int(section_number)+1
                if Unit not in Devices[DeviceID].Units:
                    #description
                    Description=get_description(DeviceID, Unit)
                    # get unit and multipler from description, example of  description: Temperature bottom_boiler [x.1 C] {-300 1500}
                    unit=None 
                    multipler=None
                    begin_unit=Description.find('[')
                    end_unit=Description.find(']')
                    if begin_unit!=-1 and end_unit!=-1: # in Description exist ext "[ somthing ]"
                        unit_desc=Description[begin_unit+1:end_unit].split(' ')
                        if len==1: # etc. [Kwh]
                            unit=unit_desc[0]
                            multipler=1
                        elif len==2: # etc. [x0.1 Kwh]
                            unit=unit_desc[1]
                            multiler=float(unit_desc[0][1:])
                    #user_note
                    user_note=clear_text(get_user_note(DeviceID, Unit))

                    # I must say: domoticz devices type table is totaly chaos for me
                    # after log time study domoticz device table I assing my four types values (string, bool, unsigned int32, signed int32)
                    # into 3 types domoticz devices text, switch and Custom sensor
                    # After that, custom sensors I try more classified by section description
                    #
                    # Basic classification device type:
                    #----------------------------------
                    # type (basic type by control byte register of section)
                    # basic types are: string, bool, unsigned_int32, int32,
                    # equal domoticz types table: 
                    # ---------------------------
                    # seahu type     | Type          | Subtype            | TypeName
                    #                | ID  | name    | ID | name          |
                    #----------------+-----+---------+----+---------------+----------
                    # string(32B)    | 243 | General | 19 | Text          | Text    | nValue=(len) sValue=text
                    # bool           | 244 | Ligh/SW | 73 | Switch        |         | nValue:0/1 sValue:Off/On
                    # unsigned int32 | 243 | General | 31 | Custom Sensor | Custom  | nValue:int sValue: string_int Options: {'Custom': '1;<axisUnits>'} 
                    # int32          | 243 | General | 31 | Custom Sensor | Custom  | nValue:int sValue: string_int Options: {'Custom': '1;<axisUnits>'} 
                    #
                    # basic types expectly "Custom sensor" can be extended by information from description of section
                    # section description shoud be contain stanadar description how more specifies usage of section can be used fro more presisely classifi device into domoticz type device
                    # more information about stanadard format section description in documentation at: https://github.com/seahu/seahu_CD/tree/master/Documentation
                    #
                    # more classification custom sensors:
                    #------------------------------------
                    # seahu type     | Type               | Subtype            | TypeName     | Switchtype   | format to store value        | note
                    # description    | ID  | name         | ID | name          |              | ID | name    |                              |
                    #----------------+-----+--------------+----+---------------+--------------+----+---------+------------------------------+-------
                    # Temperature    | 80  | Temp         | 5  | LaCrosse TX3  | Temperature  |    |         | nValue:? sValue:?            |
                    # Humidity       | 81  | Humidity     | 1  | LaCrosse TX3  | Humidity     |    |         | nValue:? sValue:?            |
                    # Presure        | 243 | General      | 9  | Pressure      | Pressure     |    |         | nValue:? sValue:?            |
                    # Lighting       | 246 | Lux          | 1  | Lux           | Illumination |    |         | nValue:0 sValue:float        |
                    # UV             | 87  | UV           | 1  |               | UV           |    |         | nValue:0 sValue:"<UV>;<Temp> | temp="0"
                    # Counter        | 113 | Counter      | 0  |               |              | 3  | Counter | nValue:? sValue:?            |
                    # RGB            | 241 | Color switch | 2  | RGB           |              |    |         | nValue:? sValue:?            |
                    # RGBW           | 241 | Color switch | 6  | RGBWZ         |              |    |         | nValue:? sValue:?            |
                    # Watt           | 248 | Usage        | 1  | Electric      | Usage        |    |         | nValue:? sValue:?            |

                    # get control_byte
                    control_byte=get_control_byte(DeviceID, Unit)
                    Log(10, "Section contol_byte: " + str(control_byte))
                    SeahuType=control_byte & 0b00011000
                    if      SeahuType==0b00000000 : 
                        Type=243
                        Subtype=19
                        Switchtype=0
                        TypeName="Text"
                    elif SeahuType==0b00001000 :
                        Type=244
                        Subtype=73
                        Switchtype=0
                        TypeName=""
                    elif SeahuType==0b00010000 :
                        Type=243
                        Subtype=31
                        Switchtype=0
                        TypeName="Custom"
                    elif SeahuType==0b00011000 :
                        Type=243
                        Subtype=31
                        Switchtype=0
                        TypeName="Custom"

                    if Type==243 : # try more clasificate device by section description
                        # -- COLOR ---
                        if Description[:4]=="RGB ":
                            Type=241
                            Subtype=2
                            Switchtype=7
                            TypeName=""
                            Log(10, "Type: RGB")
                        elif Description[:5]=="RGBW ":
                            Type=241
                            Subtype=6
                            Switchtype=7
                            TypeName=""
                            Log(10, "Type: RGBW")
                        # -- TEMPERATURE --
                        elif Description[:11]=="Temperature":
                            Type=80
                            Subtype=5
                            Switchtype=0
                            TypeName="Temperature"
                        # -- HUMIDITY --
                        elif Description[:11]=="Humidity":
                            Type=81
                            Subtype=1
                            Switchtype=0
                            TypeName="Temperature"
                        # -- PERCENTAGE --
                        elif Description[:10]=="Percentage":
                            Type=243
                            Subtype=6
                            Switchtype=0
                            TypeName="Percentage"
                        # -- DISTANCE --
                        elif Description[:8]=="Distance":
                            Type=243
                            Subtype=27
                            Switchtype=0
                            TypeName="Distance"
                        # -- COUNTER --
                        elif Description[:7]=="Counter":
                            Type=113
                            Subtype=0
                            Switchtype=3
                            TypeName=""
                        # -- VOLTAGE --
                        elif Description[:4]=="Volt":
                            Type=243
                            Subtype=8
                            Switchtype=0
                            TypeName="Voltage"
                        # -- AMPERE --
                        elif Description[:7]=="Current":
                            Type=243
                            Subtype=23
                            Switchtype=0
                            TypeName="Current (Single)"
                        # -- THERMOSTAT --
                        elif Description[:9]=="Termostat":
                            Type=242
                            Subtype=1
                            Switchtype=0
                            TypeName="Set Point"

                    #Name
                    if TypeName=="Text": Name=section+" - "+Description
                    else :
                        if (user_note!=""): Name=user_note
                        else :              Name=section+" - "+Description
                    Log(10, "Sectin name: "+ Name)

                    Log(10, "Create new section:"+ section +" Description: "+Description+"TypeName: "+TypeName+" Type: "+str(Type)+" Subtype: "+str(Subtype)+" Switchtype: "+str(Switchtype))
                    # prepare Options
                    Options={"control_byte":control_byte}
                    if (unit!=None): Options['Custom']='1;'+unit
                    if (multipler!=None): Options['multipler']=multipler
                    try:
                        Domoticz.Unit(
                                    Name=Name,
                                    Unit=int(section_number)+1,
                                    Type=Type,
                                    Subtype=Subtype,
                                    Switchtype=Switchtype,
                                    DeviceID=file,
                                    Description=Description,
                                    Options=Options
                                    ).Create()
                    except:
                        Log(10, "ERROR create new section:"+ section +" Description: "+Description)
                        continue
                    #get actual value
                    get_value_from_device(DeviceID, Unit)

