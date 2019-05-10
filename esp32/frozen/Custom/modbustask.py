#!/usr/bin/env python
"""modbusTask.py: PowerPilot python LoRa"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2019"

from  modbus import  initModbus, readPilot , getLatestMBError, MB_SUCCESS
from logging import Logger
import logging
import _thread
from globs import *
from helpers import *
import time

def modbusTask():

    global mb_wdt_lock, mb_stop_flag, MB_SUCCESS

    mb_error_count =0 

    logger = Logger(name = 'MODBUS ' + __version__,level=logging.DEBUG,filename=None)
    logger.debug('** MODBUS Task started **')

    initModbus()

    CT=int(getConfigurationVariable(NVS_CT_RATIO))
    deviceType=int(getConfigurationVariable(NVS_DEVICE_TYPE))

    # default is 22 if not defined in the config file
    DA=22
    try:
        from deviceid import modbusAddress
        DA=modbusAddress
    except :
        pass
    
    mb_stop_flag.release()
    while not mb_stop_flag.locked():
        res = readPilot(DA,deviceType,CT)
        if res != MB_SUCCESS:
            mb_error_count = mb_error_count + 1
            logger.error(str(mb_error_count) + " Read(s) failed - " + getLatestMBError() + " "  )

        mb_wdt_lock.release()
        time.sleep(1)
    
    logger.error('** MODBUS Task ended **')