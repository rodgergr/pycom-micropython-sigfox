#!/usr/bin/env python
"""loopimpedance.py: PowerPilot python LI"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2019"

import uos
import machine
import time
import network
from logging import Logger
import logging
from machine import Timer
from loopimpedance import LoopImpedance
from  modbus import getLatestInstMeasurments


from network import WLAN
from pycom import *
import queue
import binascii
from globs import *
from helpers import *
import _thread
from struct import *

liValue=[0,0,0]

def getLatestLIValue():
    global liValue
    with li_latest_value:
        return liValue

def _setLatestLIValue(newval,phase=0):
    global liValue
    with li_latest_value:
        liValue[phase]=newval

def _liValueHandler(ze,logger,phase):
    if ze == None:
        return

    if logger != None:
        logger.debug("New Value for LI: " + str(ze) + " mOhm phase " + str(phase))
    

def _liAlarmHandler(ze,logger,phase):
    if logger != None:
        logger.info("Alarm Loop impendance: " + str(ze) + " mOhm")
    if ze == None:
        return
    unixLocalTime= time.time() + time.timezone()
    #timetuple = time.localtime()
    #timeslot10M = int(timetuple[4] / 10) + timetuple[3] * 6     # 1 - 144  10 min timeslot in a day
    #timeslot600s = int((timetuple[5] + timetuple[4]*60 + timetuple[3] * 3600 - timeslot10M * 10 * 60) / 3 )     # 1-200    3s timeslot in the 10 m slot
    currentAlarmLoraMessageToSend = buildLIAlarmMessage (unixLocalTime, int(ze),phase)
    res = LoraQueueP5.put(currentAlarmLoraMessageToSend)
    if res == LoraQueueP5.SUCCESS:
        logger.debug("Add msg to LPQ5: " + str(currentAlarmLoraMessageToSend))
    else:
        logger.error("LQP5 is full")

class LiLoop:

    def __init__(self,callback=None,alarm=None,logger=None,numberOfPhase=1):
        self.seconds = 0
        self.__alarm = Timer.Alarm(self._seconds_handler, 1, periodic=True)
        self.numberOfPhase = numberOfPhase
        self.li=[0,0,0]
        self.li[PHASE_RED] = LoopImpedance(Imin=3000, Vmin=1000, ZEmin=100, Qmax=5.0, Tstable=3, ZEmatch=5.0, EventTableSize =20, Tstableperc = 1.00)
        if numberOfPhase == 3 :
            self.li[PHASE_WHITE] = LoopImpedance(Imin=3000, Vmin=1000, ZEmin=100, Qmax=5.0, Tstable=3, ZEmatch=5.0, EventTableSize =20, Tstableperc = 1.00)
            self.li[PHASE_BLUE] = LoopImpedance(Imin=3000, Vmin=1000, ZEmin=100, Qmax=5.0, Tstable=3, ZEmatch=5.0, EventTableSize =20, Tstableperc = 1.00)
        self.ze = [None,None,None]
        self.callback = callback
        self.alarm = alarm
        self._logger = logger

    def _seconds_handler(self, alarm):
        global liValue
        self.seconds += 1
        try:
            for phase in range(0,self.numberOfPhase):
                instData= getLatestInstMeasurments(PHASE_ALL)
                V = float(instData[phase][INST_VOLTAGE])
                I = float(instData[phase][INST_CURRENT])
                Q = float(instData[phase][INST_REACTIVEPOWER])
                if self.li[phase].addMeasurment(V,I,Q, self.seconds) == True:   # return true if a new meas has been computed
                    self._logger.debug("new val: " + str(self.li[phase].getZe()))
                    oldvalue = self.ze[phase]
                    self.ze[phase] = self.li[phase].getZe()
                    _setLatestLIValue(self.ze[phase],phase)
                    if self.callback != None:
                        self.callback(self.ze[phase],self._logger,phase)
                    if self.alarm != None and oldvalue != None and oldvalue * 5 <  self.ze[phase] :
                        self.alarm(self.ze[phase],self._logger,phase)
        except Exception as e:
            self._logger.error("LI second handler: " + str(e))
            pass


# run from a thread, continiously try to join any network in the ota_wifi list.
# _thread.start_new_thread(_liLoopThread)
def loopImpedanceTask():
    global li_wdt_lock, li_stop_flag
    logger = Logger(name = 'LI ' + __version__,level=logging.DEBUG,filename=None)
    logger.debug('** LI Task started **')
    deviceType=int(getConfigurationVariable(NVS_DEVICE_TYPE))
    numberOfPhase = getNumberOfPhase(deviceType)
    liLoop=LiLoop(_liValueHandler,_liAlarmHandler,logger,numberOfPhase) # start the Loop impedance process
    li_stop_flag.release()
    while not li_stop_flag.locked():
        li_wdt_lock.release()
        time.sleep(1)
        
    liLoop.__alarm.cancel()
    logger.error('** LI Task ended **')



