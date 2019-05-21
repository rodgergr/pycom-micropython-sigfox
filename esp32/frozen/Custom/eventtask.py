#!/usr/bin/env python
"""eventtask.py: PowerPilot python event POC"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

import uos
import machine
import time
from globs import *
from helpers import *
from helpers import upgradeFirmware
import _thread
from logging import Logger
import logging
from machine import RTC, Pin
from queuewrapper import QueueWrapper
import ujson
from struct import *
import pycom
from  modbus import getLatestInstMeasurments,getLoRaInstData,readRegisters,writeRegisters,writeLoad
import binascii
from loratask import getRSSI_SNR
from loopimpedancetask import getLatestLIValue
from computingtask import getComputedValues,resetComputedValue

#Helpers
def VoltageAlarmCheck(logger,currentState,unixLocalTime,deviceType):
    global voltageAlarmStateThresholds
    voltageAlarmHysteresis = 2 #2%
    numberOfPhase = getNumberOfPhase(deviceType)
    newState = [VOLTAGE_NORMAL,VOLTAGE_NORMAL,VOLTAGE_NORMAL]
    instdata = getLatestInstMeasurments(PHASE_ALL)
    for phase in range(0, numberOfPhase):  # RED, BLUE, WHITE
        instVolt = int(instdata[phase][INST_VOLTAGE] * 1000)

        if instVolt > voltageAlarmStateThresholds[ALARM_HIGH]:
            if instVolt > voltageAlarmStateThresholds[ALARM_VERY_HIGH]:
                newState[phase] = ALARM_VERY_HIGH
            else:
                newState[phase] = ALARM_HIGH
        elif instVolt < voltageAlarmStateThresholds[ALARM_LOW]:
            if instVolt < voltageAlarmStateThresholds[ALARM_VERY_LOW]:
                newState[phase] = ALARM_VERY_LOW
            else:
                newState[phase] = ALARM_LOW
        else:
            newState[phase] = VOLTAGE_NORMAL

        # If state would change
        if  newState[phase] != currentState[phase]:
            # If state is not currently VOLTAGE_NORMAL
            if(currentState[phase] != 0):
                currentStateThreshold = voltageAlarmStateThresholds[currentState[phase]]
                logger.debug("Current State Threshold {}".format(currentStateThreshold))
                percentFromThreshold = get_change_percent(currentStateThreshold,instVolt)
                logger.debug("percent from Threshold {}%".format(percentFromThreshold))
                # If hysteresis threshold is not met, and voltage is not 0 (percentage will be 0 if it is)
                if(percentFromThreshold < voltageAlarmHysteresis and instVolt != 0):
                    logger.debug("Change not great enough to meet hysteresis threshold of 2%")
                    newState[phase] = currentState[phase]
                    #Leave state unchanged for this phase
            else:
                logger.debug("Voltage is normal, hysteresis bypassed")
            # If state did change
            if  newState[phase] != currentState[phase]:
                logger.debug("Phase {} voltage state {} -> {}".format(phase,currentState[phase],newState[phase]))
                currentAlarmLoraMessageToSend = buildVoltageAlarmMessage(unixLocalTime, int(instVolt /10), int(newState[phase]), phase)
                # Add message to lora queue with debugging info
                def addLoraMessage(loraQueue,queueName,loraMessage):
                    res = loraQueue.put(loraMessage)
                    if res == loraQueue.SUCCESS:
                        logger.debug("Add msg to {}: ".format(queueName) + str(loraMessage))
                    else:
                        logger.error("{} is full".format(queueName))
                # "Very" alarms have a "Very" high priority
                if newState[phase] == 2 or newState[phase] == 4:
                    addLoraMessage(LoraQueueP3,"LQP3",currentAlarmLoraMessageToSend)
                else:
                    addLoraMessage(LoraQueueP2,"LQP2",currentAlarmLoraMessageToSend)

    return newState

def ProcessC2DMessage(logger,deviceType):
    global voltageAlarmStateThresholds
    LoraMessage = LoraQueueRx.get()
    numberOfPhase = getNumberOfPhase(deviceType)
    if LoraMessage != None:
        try:
            hexdata = binascii.hexlify(LoraMessage)
            logger.info("Received Msg: " + str(hexdata) + "  : Len:" + str(len(LoraMessage)))
            # TIME SYNC  = > Device is always using Mike time (NZST)
            # TODO refactor this
            msg = unpackDataFromBinary(LoraMessage)
            if (msg[C2D_REQUEST] == MESSAGE_TYPE_TIME_RESPONSE):
                logger.debug("Received Time Synch Msg")
                timeoffset = 0
                timezone = msg[7]
                if timezone == 122: # zulu time
                    timeoffset = 12*3600
                    
                if timezone == 109: # mike time
                    timeoffset = 0
                
                time.timezone(timeoffset)
                rtc=RTC()
                rtc.init(( int(msg[1])+2000, int(msg[2]), int(msg[3]),int(msg[4]), int(msg[5]), int(msg[6]), 0, 0))
                logger.debug(time.localtime())
                if time_set_flag.locked():
                    time_set_flag.release() # we know the time!

            if msg[C2D_REQUEST] == MESSAGE_TYPE_C2D_SET_REQUEST:
                logger.debug("Received Set request Msg Id:" + str(msg[C2D_SET_REQ_ID]) + "val:" + str(msg[C2D_SET_REQ_VAL]))
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_UPGRADE and msg[C2D_SET_REQ_VAL] == 1 :
                    upgradeFirmware(UPGRADE_URL) # start the updgrade
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] ==  C2D_PARAM_RELAY:
                    LED_OUT = Pin(LED_OUT_PIN)
                    LED_OUT.value(int(msg[C2D_SET_REQ_VAL]))
                    logger.debug("LED OUT state:" + str(msg[C2D_SET_REQ_VAL]))
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_RESET and int(msg[C2D_SET_REQ_VAL]) == 1 :
                        machine.reset()
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_RADIO_OFFSET :
                    #validate
                    radio_delay = int(msg[C2D_SET_REQ_VAL])
                    if radio_delay > 0 and radio_delay < 300 :
                        setConfigurationVariable(NVS_RADIO_DELAY,radio_delay)
                        logger.debug("New Radio delay :" + str(radio_delay))

                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_HV_ALARM  :
                    #validate HV alarm
                    hv_alarm = int(msg[C2D_SET_REQ_VAL])
                    if hv_alarm > 100000 and hv_alarm < 300000 :
                        setConfigurationVariable(NVS_HV_ALARM,hv_alarm)
                        voltageAlarmStateThresholds[ALARM_HIGH] = hv_alarm
                        logger.debug("New HV Alarm threshold :" + str(hv_alarm))

                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_VHV_ALARM  :
                    #validate VHV alarm
                    vhv_alarm = int(msg[C2D_SET_REQ_VAL])
                    if vhv_alarm > 100000 and vhv_alarm < 300000 :
                        setConfigurationVariable(NVS_VHV_ALARM,vhv_alarm)
                        voltageAlarmStateThresholds[ALARM_VERY_HIGH] = vhv_alarm
                        logger.debug("New VHV Alarm threshold :" + str(vhv_alarm))

                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_LV_ALARM  :
                    #validate LV alarm
                    lv_alarm = int(msg[C2D_SET_REQ_VAL])
                    if lv_alarm > 100000 and lv_alarm < 300000 :
                        setConfigurationVariable(NVS_LV_ALARM,lv_alarm)
                        voltageAlarmStateThresholds[ALARM_LOW] = lv_alarm
                        logger.debug("New LV Alarm threshold:" + str(lv_alarm))
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_VLV_ALARM  :
                    #validate VLV alarm
                    vlv_alarm = int(msg[C2D_SET_REQ_VAL])
                    if vlv_alarm > 100000 and vlv_alarm < 300000 :
                        setConfigurationVariable(NVS_VLV_ALARM,vlv_alarm)
                        voltageAlarmStateThresholds[ALARM_VERY_LOW] = vlv_alarm
                        logger.debug("New VLV Alarm threshold:" + str(vlv_alarm))
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_CT_RATIO  :
                    #validate
                    ct_ratio = int(msg[C2D_SET_REQ_VAL])
                    if ct_ratio >= 1 and ct_ratio <= 10000 :
                        setConfigurationVariable(NVS_CT_RATIO,ct_ratio)
                        logger.debug("New CT Ratio:" + str(ct_ratio))
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_DEV_TYPE  :
                    #validate
                    devType = int(msg[C2D_SET_REQ_VAL])
                    if devType >= M11 and devType <= C11_SPM33 :
                        setConfigurationVariable(NVS_DEVICE_TYPE,devType)
                        logger.debug("New Dev Type" + str(devType))
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_ANTENNA  :
                    #validate
                    antenna = int(msg[C2D_SET_REQ_VAL])
                    if antenna == 1 or antenna ==0 :
                        setConfigurationVariable(NVS_ANTENNA_CONFIGURATION,antenna)
                        logger.debug("New Antenna Setting: " + str(antenna))
                
                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_INST_FREQ  :
                    #validate
                    inst_freq = int(msg[C2D_SET_REQ_VAL])
                    if inst_freq >= 2 :
                        setConfigurationVariable(NVS_INST_DATA_FREQ,inst_freq)
                        logger.debug("New Inst Frequency Setting: " + str(inst_freq))

                if CD2_SET_PARAMETERS_LIST[msg[C2D_SET_REQ_ID]] == C2D_PARAM_LOAD :
                    #validateS
                    load_id = pack('i',int(msg[C2D_SET_REQ_VAL]))
                    load_number= load_id[LOAD_NUMBER]
                    load_address= load_id[LOAD_ADDRESS]
                    load_value= load_id[LOAD_VALUE]
                    if load_address > 0 :  # zero is an invalid mb address
                        writeLoad(load_address,deviceType,load_number, load_value)
                        logger.debug("Load control command for address " + str(load_address) + " number " + str(load_number) + " Value: " + str(load_value))                        

            if msg[C2D_REQUEST] == MESSAGE_TYPE_C2D_GET_REQUEST:
                logger.debug("Received Get request Msg Id:" + str(msg[C2D_GET_REQ_ID]))
                if CD2_GET_PARAMETERS_LIST[msg[C2D_GET_REQ_ID]] ==  C2D_PARAM_HHDATA:
                    timestamp = msg[C2D_GET_REQ_PARAM]
                    # see if we can found this HH message in our queue
                    for x in range(0, MAX_HH_QUEUE_ITEM):
                        msg = HHQueue.get() # get() does not erase the message in this queue
                        if (msg != None and msg[TIMESTAMP] == timestamp):
                            LoraQueueP3.put(msg) # re-send!
                            logger.debug("Add HH msg to LQP3 " +  str(binascii.hexlify((msg[MESSAGE]))))
                            break

                if CD2_GET_PARAMETERS_LIST[msg[C2D_GET_REQ_ID]] ==  C2D_PARAM_INST_DATA:
                    unixLocalTime= time.time() + time.timezone()
                    typereq = msg[C2D_GET_REQ_PARAM]
                    print(typereq)
                    mesg = getLoRaInstData(deviceType,int(typereq))
                    currentLoraMessageToSend = buildInstantaneousMessage(unixLocalTime, mesg[MESSAGE_DATA], mesg[MESSAGE_LENGTH],typereq)
                    LoraQueueImmediate.put(currentLoraMessageToSend)
                    logger.debug("Add inst msg to Immediately " +  str(binascii.hexlify((currentLoraMessageToSend[MESSAGE]))))
                
                if CD2_GET_PARAMETERS_LIST[msg[C2D_GET_REQ_ID]] ==  C2D_PARAM_PROCESSED_DATA:
                    unixLocalTime= time.time() + time.timezone()
                    rssi,snr = getRSSI_SNR()
                    compute=[0,0,0]
                    for phase in range (0 , numberOfPhase):
                        compute[phase] = getComputedValues(phase)
                    ProcessedLoraMessageToSend = buildComputedMessage(unixLocalTime, getLatestLIValue(),rssi,snr,compute,numberOfPhase)  # create current LoRA Inst message
                    LoraQueueImmediate.put(ProcessedLoraMessageToSend)  
                    logger.debug("Add Processed msg to Immediately " +  str(binascii.hexlify((ProcessedLoraMessageToSend[MESSAGE]))))
                
                if CD2_GET_PARAMETERS_LIST[msg[C2D_GET_REQ_ID]] ==  C2D_PARAM_LOAD:
                    unixLocalTime= time.time() + time.timezone()
                    load_id = pack('i',int(msg[C2D_GET_REQ_PARAM]))
                    load_number= load_id[LOAD_NUMBER]
                    load_address= load_id[LOAD_ADDRESS]
                    # assume SPM32 here as it is the only one we support for now
                    payload=readRegisters(load_address,deviceType,40039,1)  # register 40039 is the one used for this
                    val = 0
                    if ( payload  &  1 << load_number) ==  (1 << load_number) :
                        val = 1 

                    myintarray=bytearray(4)
                    myintarray[LOAD_NUMBER]=load_number
                    myintarray[LOAD_ADDRESS]=load_address
                    myintarray[LOAD_VALUE]=val
                    myintarray[LOAD_SPARE]=0

                    resVal= unpack('i',myintarray)

                    currentLoraMessageToSend = buildGetResponseMessage(unixLocalTime,CD2_GET_PARAMETERS_LIST.index(C2D_PARAM_LOAD),resVal)
                    LoraQueueImmediate.put(currentLoraMessageToSend)
                    logger.debug("Add get msg to Immediately " +  str(binascii.hexlify((currentLoraMessageToSend[MESSAGE]))))
        
            if msg[C2D_REQUEST] == MESSAGE_TYPE_C2D_GET_MODBUS_REQUEST:
                unixLocalTime= time.time() + time.timezone()
                logger.debug("Received Get Modbus request Address:" +  str(msg[C2D_GET_MD_REQ_ADDRESS]) + " " + str(msg[C2D_GET_REQ_PARAM]))
                #get the data
                payload=readRegisters(msg[C2D_GET_MD_REQ_ADDRESS],deviceType,msg[C2D_GET_REQ_PARAM][0],msg[C2D_GET_REQ_PARAM][1])
                #convert endianness
                for ii in range(0, len(payload)/2):
                    temp = payload[ii*2]
                    payload[ii*2] = payload[ii*2+1]
                    payload[ii*2+1] = temp
                #build the response
                logger.debug("MB data " + str(len(payload)) + "bytes : " + (str(binascii.hexlify((payload)))))
                msg=buildGetModbusResponseMessage(unixLocalTime,deviceType,msg[C2D_GET_MD_REQ_ADDRESS],msg[C2D_GET_REQ_PARAM][0],msg[C2D_GET_REQ_PARAM][1],payload)
                #add to queue
                LoraQueueImmediate.put(msg)
                logger.debug("Add Processed msg to Immediately " +  str(binascii.hexlify((msg[MESSAGE]))))
                return
            
            if msg[C2D_REQUEST] == MESSAGE_TYPE_C2D_SET_MODBUS_REQUEST:
                unixLocalTime= time.time() + time.timezone()
                logger.debug("Received Set Modbus request Address:" +  str(msg[C2D_GET_MD_REQ_ADDRESS]) + " " + str(msg[C2D_GET_REQ_PARAM]))
                buff=bytearray(len(msg[C2D_GET_REQ_PARAM][2]))
                #convert endianness
                for ii in range(0, len(msg[C2D_GET_REQ_PARAM][2])/2):
                    buff[ii*2] = msg[C2D_GET_REQ_PARAM][2][ii*2+1]
                    buff[ii*2+1] =  msg[C2D_GET_REQ_PARAM][2][ii*2]
                #set the data
                result=writeRegisters(msg[C2D_GET_MD_REQ_ADDRESS],deviceType,msg[C2D_GET_REQ_PARAM][0],msg[C2D_GET_REQ_PARAM][1],buff)
                logger.debug("Result: " + result)
                #TODO send a response                
                return

        except Exception as e:
            logger.error("Processing incoming message: " + str(e))
            pass       


def PowerFailCb(logger):
    power_fail = Pin(POWER_FAIL_PIN)
    if debouncedPFSignal(power_fail) == 1:
        #stopTasksButLora() # save power by leaving all non essential tasks ASAP
        pycom.nvs_set(POWER_FAIL_STATE, MODE_POWER_FAIL)
        unixLocalTime= time.time() + time.timezone()
        pycom.nvs_set(POWER_FAIL_TS, unixLocalTime)
        currentAlarmLoraMessageToSend = buildPFAlarmMessage(unixLocalTime)
        lora_queue_immediate_semaphore.acquire(0)
        rand=int(getRandom() * 2000) # 0-2000 value
        time.sleep_ms(rand) # just randow m wait of 3s
        LoraQueueImmediate.put(currentAlarmLoraMessageToSend)
        lora_queue_immediate_semaphore.acquire(1,2) # wait 2s max for msg to be sent by the Lora task
        time.sleep_ms(WAIT_TIME_BEFORE_SLEEP) # time for the LoRA MAC layer to send that message
        rand=int(getRandom() * 20) # 0-20 value
        machine.deepsleep((WAKEUP_DELAY+rand)*1000) # GO TO SLEEP NOW AND PRESERVE POWER

#Event & Alarm Task
def eventTask():
    global event_wdt_lock, event_stop_flag
    global voltageAlarmStateThresholds
    
    logger = Logger(name = 'EVENT ' + __version__,level=logging.DEBUG,filename=None)
    logger.debug('** Event Task started **')



    voltageAlarmState = [ALARM_VERY_LOW,ALARM_VERY_LOW,ALARM_VERY_LOW]

    voltageAlarmStateThresholds =	{
        1: int(getConfigurationVariable(NVS_HV_ALARM)),
        2: int(getConfigurationVariable(NVS_VHV_ALARM)),
        3: int(getConfigurationVariable(NVS_LV_ALARM)),
        4: int(getConfigurationVariable(NVS_VLV_ALARM)),
    }
    deviceType=int(getConfigurationVariable(NVS_DEVICE_TYPE))

    while not event_stop_flag.locked(): 

        unixLocalTime= time.time() + time.timezone()
        if event_wdt_lock.locked():
            event_wdt_lock.release()   # Tell the WDT Event is alive 

        counter = 10
        while counter > 0:
            counter = counter -1
            # -----------------        High priority Processing (within 100 ms) -------------------------
            if power_fail_semaphore.acquire(0) == True :
                PowerFailCb(logger)

            # other high priority tasks go here
            time.sleep_ms(100)

        # -------------------------------Low priority Processing (every 1s)      
        ProcessC2DMessage(logger,deviceType)   #  Process any message from the Cloud

        if time_set_flag.locked() == False :
            voltageAlarmState = VoltageAlarmCheck(logger,voltageAlarmState,unixLocalTime,deviceType) # check the Voltages

    logger.error('** Event Task ended **')