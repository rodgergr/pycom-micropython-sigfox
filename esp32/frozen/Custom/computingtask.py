#!/usr/bin/env python
"""computingtask.py: PowerPilot python Computing"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2019"

import uos
import machine
import time
import network
from logging import Logger
import logging
import _thread

from network import WLAN
from pycom import *
import binascii

from globs import *
from  modbus import getLatestInstMeasurments
from helpers import getConfigurationVariable,getNumberOfPhase

#Globals
MaxVoltage=[0,0,0]
MinVoltage= [300,300,300]
MaxCurrent=[0,0,0]
Energy=[0,0,0]
Balance=[100,0,0]
MaxTHDV=[0,0,0]
PSOH=[0,0,0]
PSOH2=[0,0,0]

def resetComputedValue():
    with processed_value:
        for i in range(0,3):
            MaxVoltage[i]=0
            MinVoltage[i]=300
            MaxCurrent[i]=0
            Energy[i]=0
            PSOH[i]=0
            PSOH2[i]=0

def processComputedperPhase(V,I,phase,Thdv):
    with processed_value:
        if MaxVoltage[phase] < V:
            MaxVoltage[phase] = V
        if MinVoltage[phase] > V:
            MinVoltage[phase] = V
        if MaxCurrent[phase] < I:
            MaxCurrent[phase] = I

        if int(V*1000) > VOLTAGE_EXCURSION_6_HIGH_THRESOLD:
            PSOH[phase] += 1
        elif int(V*1000) < VOLTAGE_EXCURSION_6_LOW_THRESOLD:
            PSOH[phase] += 1
        
        if int(V*1000) > VOLTAGE_EXCURSION_10_HIGH_THRESOLD:
            PSOH2[phase] += 1
        elif int(V*1000) < VOLTAGE_EXCURSION_10_LOW_THRESOLD:
            PSOH2[phase] += 1

        Energy[phase] =Energy[phase] + V*I
    
        if MaxTHDV[phase] < Thdv:
            MaxTHDV[phase] = Thdv

#check load balancing
def processComputedTotal():
    with processed_value:
        total = Energy[0]+Energy[1]+Energy[2]
        if total == 0 :
            Balance[0]=33
            Balance[1]=33
            Balance[2]=33
        else:
            for i in range(0,3):
                Balance[i] = int(Energy[i] * 100 / total)

def getComputedValues(phase):
    with processed_value:
        return (MaxVoltage[phase],MinVoltage[phase],MaxCurrent[phase],PSOH[phase],Balance[phase],MaxTHDV[phase],PSOH2[phase])

def computingTask():
    global compute_wdt_lock, compute_stop_flag

    logger = Logger(name = 'COMPUTING ' + __version__,level=logging.INFO,filename=None)
    logger.info('** Computing Task started **')

    deviceType=int(getConfigurationVariable(NVS_DEVICE_TYPE))
    numberOfPhase = getNumberOfPhase(deviceType)

    compute_stop_flag.release() 
    while not compute_stop_flag.locked():
        compute_wdt_lock.release()

        instdata = getLatestInstMeasurments(PHASE_ALL)
        for phase in range (0 , numberOfPhase):
            processComputedperPhase(instdata[phase][0],instdata[phase][1],phase,instdata[phase][4])
        
        if numberOfPhase == 3:
            processComputedTotal()

        time.sleep(1)

    logger.error('** COMPUTING Task ended **')