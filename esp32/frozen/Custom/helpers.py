#!/usr/bin/env python
"""helpers.py: PowerPilot python helpers POC"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

from globs import *
from machine import RTC, WDT, Pin
import gc
from pycom import *
import pycom
import machine
import time
from struct import pack,unpack
import binascii
import _thread


nvs_lock = _thread.allocate_lock()

def get_change_percent(current, previous):
    try:
        return (abs(current - previous) / previous) * 100.0
    except ZeroDivisionError:
        return 0

def packageDataInJSON(data2send, msgtype):
    jsonData = '{'
    jsonData +='"m":"' + msgtype + '",'
    jsonData += data2send
    jsonData += '}'
    return jsonData


def getNumberOfPhase(devtype):
    numberOfPhase = 1
    if devtype == C11_SPM93 or devtype == C11_SPM32 or devtype == C11_SPM33:
        numberOfPhase = 3
    return numberOfPhase

def packageDataInBinary(msgtype, timeslot, data2send, length,B3=0x00):
    myarray=bytearray(length+BINARY_PROTOCOL_HEADER)

    #default values for headers
    myarray[1]=VERSION_SOFTWARE  # reserved B2 , use the software version here
    myarray[2]=B3 

    # copy the time slot
    ts = pack('I',int(timeslot)) # unsingned int (4B)
    for i in range(0,4):
        myarray[3+i] = ts[i]

    # get the right mesg type / version
    if msgtype == MESSAGE_TYPE_INSTANTANEOUS_DATA:
        myarray[0]=0x00

    elif msgtype == MESSAGE_TYPE_METERING_DATA:
        myarray[0]=0x04

    elif msgtype == MESSAGE_TYPE_COMPUTED_DATA:
        myarray[0]=0x08
    
    elif msgtype == MESSAGE_TYPE_ALARM_LI:
        myarray[0]=0x80
    
    elif msgtype == MESSAGE_TYPE_ALARM_VOLTAGE:
        myarray[0]=0x84

    elif msgtype == MESSAGE_TYPE_ALARM_POWER_FAIL:
        myarray[0]=0x88
    
    elif msgtype == MESSAGE_TYPE_C2D_GET_MODBUS_RESPONSE:
        myarray[0]=0x90
    
    elif msgtype == MESSAGE_TYPE_D2C_GET_RESPONSE:
        myarray[0]=0x50
    
    elif msgtype == MESSAGE_TYPE_TIME_REQUEST:
        myarray[0]=0x10
        mac = machine.unique_id()
        #overwrite empty timeslot with final 4 bytes of Mac address
        myarray[3]=mac[2]
        myarray[4]=mac[3]
        myarray[5]=mac[4]
        myarray[6]=mac[5]
    else:
        return None # not supported

    # copy all bytes
    for i in range (0,length):
        myarray[BINARY_PROTOCOL_HEADER+i]=data2send[i]

    return myarray

def unpackDataFromBinary(msg):
    length = len(msg)

    #4400001207051714187A(RAAAEgcFFxQYeg==)  time synch
    #34000002690401000000(NAAAAmkEAQAAAA==)  reboot
    #34000001690401000000  relay command ON
    #34000001690400000000  relay command OFF
    #34000000690401000000  upgrade
    #3400000369041e000000  set radio offset  to 30 (1e))
    #34000004690470820300  set HVT to  230V (70820300))  90d00300 =250V ,, 48dc0300 = 253V , 207V = 98280300     unpack('h',binascii.unhexlify('7fff'))[0] , binascii.hexlify(pack('h',800))
    #3400000a6904b8f30300
    #34000005690470820300  set LVT  to 230V
    #34000006690414000000 (NAAABmkEFAAAAA==)  set CT ratio to 20
    #34000006690410270000 set CT ratio to 10000
    #34000007690402000000 set Device type to SPM93
    #34000007690403000000 set Device type to SPM32
    #34000007690404000000 set Device type to SPM33
    #34000007690404000000 set Device type to SPM33
    #34000008690400000000 (NAAACGkEAAAAAA==) set antenna to Internal
    #34000008690401000000(NAAACGkEAQAAAA==) set antenna to External
    #3400000969040a000000 set inst frequency to 10 min

    #3400000C690400160000 Load control relay 0 address 22 set to 0
    #3400000C690400160100 Load control relay 0 address 22 set to 1
    #3400000C690401160000 Load control relay 1 address 22 set to 0
    #3400000C690401160100 Load control relay 1 address 22 set to 1

    #70000016000027  Get Relay statuses  on SPM32/SPM33 address 22 (register 40039)

    #3000000100000000(MAAAAQAAAAA=)  get INST type 0
    #3000000101000000 (MAAAAQEAAAA=) get INST type 1
    #3000000200000000 (MAAAAgAAAAA=)  get Extended

    #70000016000001  Get modbus register 40001
    #74000016C800010500  Set modbus register 40201 to 5  (CT ratio for M11/SPM91)
    #74000016CA00010500  Set modbus register 40203 to 5  (CT ratio for SPM32)
    #74000016CA00013E00 (dAAAFsoAAT4A) Set modbus register 40203 to 60  (CT ratio for SPM32)
    #74000016C900010500  Set modbus register 40202 to 5  (CT ratio for SPM33) 
    #74000016A10F010500  Set modbus register 44002 to 5  (CT ratio for SPM93)
    #74000016A10F01a000  Set modbus register 44002 to 160  (CT ratio for SPM93)

    #700000162a2301

    #34000004690470820300  (NAAABGkEcIIDAA==)
    #34000005690462000000  (NAAABWkEYgAAAA==)

    if length < 3 :
        return None
    
    msgtype = msg[0]
    if msgtype == 0x44 and length == 10:  # time response
        B2 = msg[1]
        B3 = msg[2]
        YY = msg[3]
        MM = msg[4]
        DD = msg[5]
        hh = msg[6]
        mm = msg[7]
        ss = msg[8]
        TZ = msg[9]

        return [MESSAGE_TYPE_TIME_RESPONSE,int(YY),int(MM),int(DD),int(hh),int(mm),int(ss),TZ]
    
    elif msgtype == 0x34 and length == 10:  # MESSAGE_TYPE_C2D_SET_REQUEST
        B2 = msg[1]
        B3 = msg[2]
        ID = msg[3]  # name (ID) of parameters (0-254)
        TYPE = msg[4]  # type  =>  only support signed int  i.e. must be 'i' or 105
        LEN = msg[5] # length of the value => only support 4 for now (4B)
        val = unpack('i',msg[6:10])[0] # value  => currently only support signed int (4B)
    
        return [MESSAGE_TYPE_C2D_SET_REQUEST,ID,val]

    elif msgtype == 0x30 and length == 8:  # MESSAGE_TYPE_C2D_GET_REQUEST
        B2 = msg[1]
        B3 = msg[2]  
        ID = msg[3]  # name (ID) of parameters (0-254)
        # retrieve an another parameter as uint32 (time stamp for instance)
        PARAM = unpack('I',msg[4:8])[0]
    
        return [MESSAGE_TYPE_C2D_GET_REQUEST,ID,PARAM]
    
    elif msgtype == 0x70 and length == 7:  # MESSAGE_TYPE_C2D_GET_MODBUS_REQUEST
        B2 = msg[1]
        B3 = msg[2]
        MD_ADDRESS =  msg[3]
        REGISTER_START_ADDRESS = unpack('h',msg[4:6])[0]+40001
        NUM_OF_REGISTER = msg[6]
   
        return [MESSAGE_TYPE_C2D_GET_MODBUS_REQUEST,MD_ADDRESS,(REGISTER_START_ADDRESS,NUM_OF_REGISTER)]
    
    elif msgtype == 0x74 and length > 7:  # MESSAGE_TYPE_C2D_SET_MODBUS_REQUEST
        B2 = msg[1]
        B3 = msg[2]
        MD_ADDRESS =  msg[3]
        REGISTER_START_ADDRESS = unpack('h',msg[4:6])[0]+40001
        NUM_OF_REGISTER = msg[6]
   
        return [MESSAGE_TYPE_C2D_SET_MODBUS_REQUEST,MD_ADDRESS,(REGISTER_START_ADDRESS,NUM_OF_REGISTER,msg[7:7+NUM_OF_REGISTER*2])]
    
    return None



def initConfigurationVariable(name, initvalue):
    with nvs_lock:
        data = pycom.nvs_get(name)
        if data == None :
            print('DEBUG setting NVS {} to {}'.format(name,initvalue))
            pycom.nvs_set(name, initvalue)

def getConfigurationVariable(name):
    with nvs_lock:
        return pycom.nvs_get(name)

def setConfigurationVariable(name, value):
    with nvs_lock:
        pycom.nvs_set(name, value)


def buildPFAlarmMessage(timestamp):
    if timestamp == None:
        timestamp = 0
    currentAlarmLoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_ALARM_POWER_FAIL, timestamp, None,0)
    return [currentAlarmLoraMessageToSend,timestamp,BINARY_PROTOCOL_HEADER]

def buildLIAlarmMessage(timestamp,li,phase):
    if timestamp == None:
        timestamp = 0
    
    currentAlarmLoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_ALARM_LI, timestamp, pack('HB',int(li),int(phase)),3)
    return [currentAlarmLoraMessageToSend,timestamp,3+BINARY_PROTOCOL_HEADER]

def buildVoltageAlarmMessage(timestamp,voltage,alarmtype,phase):
    if timestamp == None:
        timestamp = 0
    myarray=bytearray(4)
    myarray= pack('BBH', alarmtype, phase , voltage)

    currentAlarmLoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_ALARM_VOLTAGE, timestamp, myarray,4)
    return [currentAlarmLoraMessageToSend,timestamp,4+BINARY_PROTOCOL_HEADER]

def buildTimeRequestMessage():
    LoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_TIME_REQUEST, 0, None,0)
    return [LoraMessageToSend,0,BINARY_PROTOCOL_HEADER]

def buildComputedMessage(unixLocalTime,LastestLiValue,rssi,snr,compute,numberOfphase):
    # rssi is alwasy negative range is 0 -> -170 dBm so invert and use a single byte for packing
    instfreq=getConfigurationVariable(NVS_INST_DATA_FREQ)
    antenna=getConfigurationVariable(NVS_ANTENNA_CONFIGURATION)
    swct=getConfigurationVariable(NVS_CT_RATIO)
    devtype=getConfigurationVariable(NVS_DEVICE_TYPE)
    memfree = UINT16_CAPPED(int(gc.mem_free())/1000)
    data=pack('BbBBBBHH',-int(rssi),int(snr),int(instfreq),int(devtype),int(antenna),int(0),int(swct),int(memfree))
    msglength=10
    for phase in range(0,numberOfphase):
        data += pack('HBBHHBBB', UINT16_CAPPED(int(LastestLiValue[phase])),
                       INT8_CAPPED(int(compute[phase][MAX_VOLTAGE])-230),
                       INT8_CAPPED(int(compute[phase][MIN_VOLTAGE])-230),
                       UINT16_CAPPED(int(compute[phase][MAX_CURRRENT])),
                       UINT16_CAPPED(int(compute[phase][PSOH])),
                       UINT8_CAPPED(int(compute[phase][BALANCE])),
                       UINT8_CAPPED(int(compute[phase][MAX_THDV])),
                       UINT8_CAPPED(int(compute[phase][PSOH2])))
        msglength += 11
    LILoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_COMPUTED_DATA,unixLocalTime, data,msglength)# create current LoRA Inst message
    return [LILoraMessageToSend,unixLocalTime,BINARY_PROTOCOL_HEADER+msglength]

def buildGetModbusResponseMessage(unixLocalTime,deviceType,address,startAddress,nbOfRegisters,databuff):
    buff= bytearray(5+nbOfRegisters*2)
    buff=pack('BBHB',int(deviceType),int(address),int(startAddress)-40001,int(nbOfRegisters))
    buff+=databuff
    MessageToSend = packageDataInBinary(MESSAGE_TYPE_C2D_GET_MODBUS_RESPONSE,unixLocalTime, buff,5+nbOfRegisters*2)# create current LoRA Inst message
    return [MessageToSend,unixLocalTime,BINARY_PROTOCOL_HEADER+5+nbOfRegisters*2]

def buildInstantaneousMessage(unixLocalTime,data,length,type=0):
    #first 2 digits  defines the type of inst request ( up to 4 types)
    B3= type & 0x03
    currentLoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_INSTANTANEOUS_DATA,unixLocalTime, data,length,B3)
    return [currentLoraMessageToSend,unixLocalTime,length+BINARY_PROTOCOL_HEADER] 

def buildMeteringMessage(unixLocalTime,data,length):
    currentHHLoraMessageToSend = packageDataInBinary(MESSAGE_TYPE_METERING_DATA, unixLocalTime, data, length)   # create current LoRA Inst message
    return [currentHHLoraMessageToSend,unixLocalTime,length+BINARY_PROTOCOL_HEADER]

def buildGetResponseMessage(unixLocalTime,id,value):
    msglength=5
    buff=pack('Bi',int(id),int(value))
    MessageToSend = packageDataInBinary(MESSAGE_TYPE_D2C_GET_RESPONSE, unixLocalTime, buff, msglength)   # create current LoRA Inst message
    return [MessageToSend,unixLocalTime,msglength+BINARY_PROTOCOL_HEADER]

def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

def getRandom():
    rand = machine.rng() / RNG_MAX
    return rand


def stopTasks():
    global main_stop_flag, lora_stop_flag, wifi_stop_flag, li_stop_flag, mb_stop_flag,event_stop_flag
    main_stop_flag.acquire(0)
    lora_stop_flag.acquire(0)
    wifi_stop_flag.acquire(0)
    li_stop_flag.acquire(0)
    mb_stop_flag.acquire(0)
    event_stop_flag.acquire(0)

def stopTasksButLora():
    global main_stop_flag, wifi_stop_flag, li_stop_flag, mb_stop_flag,event_stop_flag
    main_stop_flag.acquire(0)  
    wifi_stop_flag.acquire(0)
    li_stop_flag.acquire(0)
    mb_stop_flag.acquire(0)
    event_stop_flag.acquire(0)

# lock the PF sem if new PF state
def _powerFail_cb(arg):
    if power_fail_semaphore.locked():
        power_fail_semaphore.release()

def debouncedPFSignal(power_fail):
    #lets read the signal over 10ms period and confirm it is a valid true state
    ctr= 10
    state = 1
    while (ctr > 0):
        if power_fail.value() == 0:
            state = 0
            break
        ctr -= 1
        time.sleep_ms(1)
    return state

# return the state of the Power Fail input
def hardwareInit(switch):

    #init
    gc.disable()
    gc.enable()

    power_fail = Pin(POWER_FAIL_PIN, mode=Pin.IN, pull=Pin.PULL_UP)
    power_fail.callback(Pin.IRQ_RISING, _powerFail_cb)

    pf_state = debouncedPFSignal(power_fail)

    display = Pin('P8', mode=Pin.OUT)

    if pf_state == False:
        display.value(1)

    LED_OUT = Pin(LED_OUT_PIN, mode=Pin.OUT)
    LED_OUT.value(0)

    SWITCH_VDD = Pin('P20', mode=Pin.OUT) # old H/W version
    SWITCH_VCTL  = Pin('P19', mode=Pin.OUT)

    SWITCH_VDD.value(1)
    SWITCH_VCTL.value(switch)

    power_fail_semaphore.acquire(0) # locked the sem 

    return pf_state

# OTA Upgrade via Wifi

def upgradeFirmware(uri):
    pass


def getDynamicPowerGain(v,i):
	power_total = int(v*i) # i and v are always positive
	if power_total > MAXINT16 and power_total <= MAXINT16 * 10:
		return 10
	elif power_total > MAXINT16 * 10:
		return 100
	else :
		return 1	

def CAPPED(val, max, min):
	if val > max :
		return max
	elif val < min :
		return min
	else :
		return val
	
def UINT16_CAPPED(val) :
	return CAPPED(val,MAXUINT16, 0 )

def INT16_CAPPED(val) :
	return CAPPED(val,MAXINT16, MININT16 )

def UINT8_CAPPED(val) :
	return CAPPED(val,255, 0 )

def INT8_CAPPED(val) :
	return CAPPED(val,MAXINT8, MININT8 )