#!/usr/bin/env python
"""mainTask.py: PowerPilot python main POC"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2019"

import uos
import machine
import time
from globs import *
from helpers import *
import _thread
from logging import Logger
import logging
from machine import RTC, Pin
from  modbus import  getLoRaInstData, getHHData, getInstData,getLoRaHHData
from queuewrapper import QueueWrapper
import ujson
from struct import *
import pycom
from loratask import getRSSI_SNR
from loopimpedancetask import getLatestLIValue
from computingtask import getComputedValues,resetComputedValue

#Main Task
def mainTask():
    global main_wdt_lock, main_stop_flag

    LastestLiValue=0
    
    logger = Logger(name = 'MAIN ' + __version__,level=logging.DEBUG,filename=None)
    logger.debug('** Main Task started **')

    unixLocalTime= time.time() + time.timezone() 
    #time to get the time

    # Wait some  random time (to give a chance to some other device in the region to send the time request and we get the multicast back for free)
    waittime = int(getRandom() * 20)
    logger.debug('Waiting :' + str(waittime))
    time.sleep(waittime) 
    
    if time_set_flag.locked() == True:
        timeRequestMsg= buildTimeRequestMessage()
        LoraQueueP5.put(timeRequestMsg) 

    deviceType=int(getConfigurationVariable(NVS_DEVICE_TYPE))

    # how many phases should we read ?
    numberOfPhase = getNumberOfPhase(deviceType)

    # get the freq of inst messages
    INST_FREQ=getConfigurationVariable(NVS_INST_DATA_FREQ)

    typereq = 0 # used to inst message

    MQTTServer=None
    try:
        from deviceid import mqtt_connect_server
        MQTTServer = mqtt_connect_server
    except:
        pass

    main_stop_flag.release()
    while not main_stop_flag.locked():
        main_wdt_lock.release()   # Tell the WDT Main is alive 
        time.sleep(1)
        unixLocalTime= time.time() + time.timezone()
        timetuple = time.localtime()

        # **********************         Every 10s          ****************************************************
        if (timetuple[5] % 10 == 0 and MQTTServer != None) : 
            if time_set_flag.locked() == False:
                data2Send = packageDataInJSON(getInstData(numberOfPhase),MESSAGE_TYPE_INSTANTANEOUS_DATA)
                res= WifiQueueTx.put([MESSAGE_TYPE_INSTANTANEOUS_DATA_WIFI, data2Send])
                #logger.debug("Add Msg via Wfiqueue: " + data2Send)

        if (timetuple[5] != 0):
            continue 
        #-------------------------   runs once every minute past this point --------------------
       

        # **********************         Every 5 min          ****************************************************
        if (timetuple[4] % 5 == 0) :
            if time_set_flag.locked() == True :
                timeRequestMsg= buildTimeRequestMessage()
                res =  LoraQueueP4.put(timeRequestMsg) 
                logger.debug("Add Time Request to LQP4")   

        # **********************         Every 10 min          ****************************************************
        if (timetuple[4] % INST_FREQ == 0):
            # get the type of the request
            
            if deviceType == C11_SPM32 or deviceType == C11_SPM33:
                #alternate between 0 and 1 (2 types supported at present)
                if typereq == 0:
                    typereq = 1
                else:
                    typereq = 0
            mesg = getLoRaInstData(deviceType,typereq)
            currentLoraMessageToSend = buildInstantaneousMessage(unixLocalTime, mesg[MESSAGE_DATA], mesg[MESSAGE_LENGTH],typereq)

            res =  LoraQueueP4.put(currentLoraMessageToSend)  # we use P2 here as it is not as important as HH data
            if res == LoraQueueP3.SUCCESS:
                logger.debug("Add msg to LQP3 " + str(binascii.hexlify(currentLoraMessageToSend[MESSAGE])))
            else:
                logger.error("LQP3 is full")

        # **********************         Every 30 min          ****************************************************
        if (timetuple[4] % 30 == 0) :
            currentHHLoraMessageToSend = buildMeteringMessage( unixLocalTime, getLoRaHHData(1), 16)   # create current LoRA Inst message
            res = LoraQueueP4.put(currentHHLoraMessageToSend)
            HHQueue.put(currentHHLoraMessageToSend) # save it in case we need to resend later
            if res == LoraQueueP3.SUCCESS:
                logger.debug("Add msg to LQP3 + " + str(binascii.hexlify(currentHHLoraMessageToSend[MESSAGE])))
            else:
                logger.error("LQP3 is full")
    
        # **********************     Every 4 hours      ****************************************************
        if ( timetuple[3] % 4 == 0 and timetuple[4] == 15):
            rssi,snr = getRSSI_SNR()
            LastestLiValue = getLatestLIValue()
            compute=[0,0,0]
            for phase in range (0 , numberOfPhase):
                compute[phase] = getComputedValues(phase)
            LILoraMessageToSend = buildComputedMessage(unixLocalTime, LastestLiValue,rssi,snr,compute,numberOfPhase)  # create current LoRA Inst message
            res = LoraQueueP2.put(LILoraMessageToSend)
            if res == LoraQueueP3.SUCCESS:
                logger.debug("Add msg to LQP2 + " + str(binascii.hexlify(LILoraMessageToSend[MESSAGE])))
            else:
                logger.error("LQP12is full")
        
        # **********************     once a day @ 11.4 pm           ****************************************************
        if timetuple[4] == 44 and timetuple[3] == 23:
            timeRequestMsg= buildTimeRequestMessage() # create current LoRA Inst message
            res = LoraQueueP1.put(timeRequestMsg)
            if res == LoraQueueP1.SUCCESS:
                logger.debug("Add msg to LQP1 + " + str(binascii.hexlify(timeRequestMsg[MESSAGE])))
            else:
                logger.error("LQP1 is full")

         # **********************    reset computing values at the start of the day ****************** 
        if timetuple[4] == 16 and timetuple[3] == 0:
            resetComputedValue()
        
        

    logger.error('** Main Task ended **')