#!/usr/bin/env python
"""wdttask.py: PowerPilot python WDT """
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

import uos
from machine import  WDT
import _thread
import time
from logging import Logger
import logging
from globs import *
import gc
import machine

def wdtTask():
    global main_wdt_lock, main_stop_flag
    global wifi_wdt_lock, wifi_stop_flag
    global li_wdt_lock,  li_stop_flag
    global lora_wdt_lock,  lora_stop_flag
    global mb_wdt_lock,  mb_stop_flag

    logger = Logger(name = 'WDT ' + __version__,level=logging.DEBUG,filename=None)
    logger.debug('** WDT Task started **')

    wdt = WDT(timeout=60000*5)  # 7 min
    logger.debug('** WDT set to 5 min **')

    while True:
        time.sleep(20)      
        mema = gc.mem_alloc()
        time.sleep(20)
        memf = gc.mem_free()
        gc.collect()
        time.sleep(20)
        wdtPassSuccessful = True
        
        if main_stop_flag.locked() == False:
            if main_wdt_lock.locked() == True:
                logger.critical('** MAIN TASK FAILED **')
                wdtPassSuccessful = False
            else:
                main_wdt_lock.acquire(0)

        if wifi_stop_flag.locked() == False:
            if wifi_wdt_lock.locked() == True:
                logger.critical('** WIFI TASK FAILED**')
                wdtPassSuccessful = False
            else:
                wifi_wdt_lock.acquire(0)

        if li_stop_flag.locked() == False:
            if li_wdt_lock.locked() == True:
                logger.critical('** LI TASK FAILED**')
                wdtPassSuccessful = False
            else:
                li_wdt_lock.acquire(0)

        if lora_stop_flag.locked() == False:
            if lora_wdt_lock.locked() == True:
                logger.critical('** LORA TASK FAILED**')
                wdtPassSuccessful = False
            else:
                lora_wdt_lock.acquire(0)
        
        if mb_stop_flag.locked() == False:
            if mb_wdt_lock.locked() == True:
                logger.critical('** MODUBUS TASK FAILED**')
                wdtPassSuccessful = False
            else:
                mb_wdt_lock.acquire(0)
            
        if event_stop_flag.locked() == False:
            if event_wdt_lock.locked() == True:
                logger.critical('** EVENT TASK FAILED**')
                wdtPassSuccessful = False
            else:
                event_wdt_lock.acquire(0)
        
        if compute_wdt_lock.locked() == False:
            if compute_wdt_lock.locked() == True:
                logger.critical('** COMPUTING TASK FAILED**')
                wdtPassSuccessful = False
            else:
                compute_wdt_lock.acquire(0)
                

        if wdtPassSuccessful == True :
             wdt.feed() 
             logger.debug('** FEED THE WDT | Heap Allocated: ' + str(mema) + 'B | Heap Free: ' + str(memf) + 'B')
