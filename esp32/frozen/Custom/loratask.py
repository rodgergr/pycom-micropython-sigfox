#!/usr/bin/env python
"""loratask.py: PowerPilot python LoRa"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2019"

import uos
import machine
import time
import network
from network import LoRa
try:
    from deviceid import app_eui,app_key,dev_eui
except:
    pass
    
from logging import Logger
import logging
import socket
from pycom import *
import pycom
import binascii
import _thread
from globs import *
from helpers import getConfigurationVariable

FARINTHEFUTURE=int(100000000000)

MAXQUEUE= 5
QueueDelay=[0,0,0,0,0]
OldestQueueMsg=[FARINTHEFUTURE,FARINTHEFUTURE,FARINTHEFUTURE,FARINTHEFUTURE,FARINTHEFUTURE]
FixedDelayPerQueue=[P1TIMEOUT,P2TIMEOUT,P3TIMEOUT,P4TIMEOUT,P5TIMEOUT]


rx_rssi=0
rx_snr=0

def lora_cb(lora):
    events = lora.events()
    if events & LoRa.RX_PACKET_EVENT:
        pass
    if events & LoRa.TX_PACKET_EVENT:
        pass

def lora_send_data(lora,  lorawan_socket, data2send, nbOfByteToSend, confirmed = False):
    try:
        lorawan_socket.setblocking(False)
        lorawan_socket.setsockopt(socket.SOL_LORA, socket.SO_CONFIRMED, confirmed)
        nbOfByteSent = lorawan_socket.send(data2send) 
        if nbOfByteToSend == nbOfByteSent:
            return SUCCESS
        else:
            return FAIL
    except Exception as e:
        return FAIL
    
def getRSSI_SNR():
    global rx_rssi,rx_snr
    with lora_rssi_read:
        return rx_rssi,rx_snr

# Processing of LoRA Downlinks
# this is called every 1s by the Lora task
def lora_rx_data(lora,  lorawan_socket, logger):
    global rx_rssi,rx_snr
  
    rawmsg = lorawan_socket.recv(64)
    if  rawmsg != b'':
        LoraQueueRx.put(rawmsg)
        stats = lora.stats()
        with lora_rssi_read:
            rx_rssi = stats[1]
            rx_snr = stats[2]
        logger.debug("RSSI=" + str(rx_rssi) + " SNR=" + str(rx_snr))

# Process message immediately
def processMsgImmediate(theQueue,  logger, lora,lorawan_socket) :
    LoraMessageToSend = theQueue.get()
    if LoraMessageToSend != None :
        res= lora_send_data(lora, lorawan_socket, LoraMessageToSend[MESSAGE],LoraMessageToSend[LENGTH],False)
        logger.debug("Sending " + str(res) + " with " + str(binascii.hexlify((LoraMessageToSend[MESSAGE]))))
        return True # done!
    else:
        return False

# message throttling to meet the 1% duty cycle
def processQOSQueue(theQueue, qIndex, unixLocalTime, logger, lora,lorawan_socket,RadioOffset) :
    global QueueDelay,OldestQueueMsg,FixedDelayPerQueue
    LoraMessageToSend = theQueue.get()
    if LoraMessageToSend != None :
        if  LoraMessageToSend[TIMESTAMP] <= OldestQueueMsg[qIndex-1] and unixLocalTime > (LoraMessageToSend[TIMESTAMP] + RadioOffset):
            if (unixLocalTime + int(RadioOffset * (6-qIndex) / 5 )) >= QueueDelay[qIndex-1] :
                result = lora_send_data(lora, lorawan_socket, LoraMessageToSend[MESSAGE],LoraMessageToSend[LENGTH],False)
                logger.debug("Sending msg from LQP" + str(qIndex) + ": " + str(binascii.hexlify(LoraMessageToSend[MESSAGE])))
                # re- adjust the delay for all queues
                if result  == SUCCESS:
                    for index in range(0,MAXQUEUE):
                        QueueDelay[index] = int(int(unixLocalTime) + int(FixedDelayPerQueue[index]))
                else:
                    logger.error("Failed to send from LQP" + str(qIndex-1))

                OldestQueueMsg[qIndex-1] = FARINTHEFUTURE  # done with this mesg
            else:
                OldestQueueMsg[qIndex-1] = int(LoraMessageToSend[TIMESTAMP])  # oldest mesg but not allow to send just yet
                theQueue.put(LoraMessageToSend) # back to the queue
        else:
            theQueue.put(LoraMessageToSend) # we have an older message in this queue so wait

def loraTask():
    global lora_wdt_lock, lora_stop_flag
    global QueueDelay

    logger = Logger(name = 'LORA ' + __version__,level=logging.DEBUG,filename=None)
    logger.debug('** LORA Task started **')

    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868, adr=True, public=True, device_class=LoRa.CLASS_C)  
    lorawan_socket = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lorawan_socket.setblocking(False)

    RadioOffset = getConfigurationVariable(NVS_RADIO_DELAY)
    # defautl is OTAA (old config file may not have this defined)
    JM=0
    try:
        from deviceid import lora_join_method
        JM=lora_join_method
    except :
        pass

    #add the multicasts
    try:
        from deviceid import multicasts
        for multicast_address in multicasts:
            multicast_devAddr= multicast_address[0]
            multicast_NwkSKey = multicast_address[1]
            multicast_AppSKey= multicast_address[2]
            lora.add_multicast(keys=(multicast_devAddr,multicast_NwkSKey,multicast_AppSKey))
    except:
        pass
    
    # define the reserved Downlink minutes if it exists
    DRM=[8]
    try:
        from deviceid import downlinks_reserved_minutes
        DRM = downlinks_reserved_minutes
    except :
        pass  

    
    if JM == OTAA_JOIN:
        logger.debug("Attempting OTAA Join Kotahi.net...using device EUI " + str(binascii.hexlify(dev_eui)))
        lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)
        counter = 0
        while not lora.has_joined():
            time.sleep(1)
            counter = counter + 1
            lora_wdt_lock.release() # tell the WDT that LoRa task is busy waiting to join
        logger.debug("Join successful after " + str(counter) + "s")            
    elif JM == ABP_JOIN:
        from deviceid import dev_addr,nwk_swkey,app_swkey
        logger.debug("Joined Kotahi.net using ABP with EUI " + str(binascii.hexlify(dev_eui)))
        lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))
        #time.sleep(3)
    # save the state in NVS
    #lora.nvram_save()
    
    if pycom.nvs_get(POWER_FAIL_STATE) == MODE_POWER_FAIL:
        # SPECIAL CASE FOR POWER FAIL AFTER DEEP SLEEP WAKE UP
        #lora.nvram_restore()
        processMsgImmediate(LoraQueueImmediate,  logger, lora,lorawan_socket ) # the PF message should already be in the queue            
        lora_queue_immediate_semaphore.release() # tell the main thread we are done
        time.sleep(10)
        #lora.nvram_save()
    
    # Lora loop starts here, processing the QOS queues 
    lora_stop_flag.release()
    
    while not lora_stop_flag.locked():
        
        # wait 1s while checking for any message to be sent immediately (e.g PF Alarm)
        counter = 10
        while counter > 0:
            if processMsgImmediate(LoraQueueImmediate,  logger, lora,lorawan_socket) == True:
                lora_queue_immediate_semaphore.release() # signal that the immediate msg has been sent
                time.sleep(1)
            counter = counter -1
            time.sleep_ms(100)

        # Process any downlinks
        lora_rx_data(lora, lorawan_socket,logger)

        lora_wdt_lock.release() # tell the WDT that LoRa task is doing just fine

        unixLocalTime= time.time() + time.timezone()

        timetuple = time.localtime()
        maskUplinks = False
        for minute in DRM:
            if (timetuple[4] % 10  == minute):
                maskUplinks = True
                break

        # process the uplinks using basic QOS Queues
        if maskUplinks == False:
            processQOSQueue(LoraQueueP5, 5, unixLocalTime, logger, lora,lorawan_socket,RadioOffset) 
            processQOSQueue(LoraQueueP4, 4, unixLocalTime, logger ,lora,lorawan_socket,RadioOffset)
            processQOSQueue(LoraQueueP3, 3, unixLocalTime, logger, lora,lorawan_socket,RadioOffset)
            processQOSQueue(LoraQueueP2, 2, unixLocalTime, logger, lora,lorawan_socket,RadioOffset)
            processQOSQueue(LoraQueueP1, 1, unixLocalTime, logger, lora,lorawan_socket,RadioOffset)
        
    logger.error('** LORA Task ended **')