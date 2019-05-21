#!/usr/bin/env python
"""wifitask.py: PowerPilot python Wifi"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

import uos
import machine
import time
import network
from logging import Logger
import logging
import _thread
try:
     from deviceid import ota_wifi
except:
     pass

from network import WLAN
from pycom import *
import binascii

from OTA import WiFiOTA

from globs import *

def settimeout(duration):
     pass

#from wifitask import do_wifi_fota
#do_wifi_fota("Irontech01","powerpilot","192.168.10.106",8000)

#public function to perfrom FOTA
def do_wifi_fota(ssid, password,server_ip,server_port):
    ota = WiFiOTA(ssid, password, server_ip, server_port)
    fota_wifi_release_sem.acquire(0) # disconnect wifi but keep the task going to avoid triggering the watchdog is the upgrade is over 5 min
    # Perform OTA
    time.sleep(3)
    try:
        print("Connecting to "  + str(ssid))
        ota.connect()
        print("Updating...")
        ota.update()
    except Exception as e:
        print("Error while int FOTA: " + str(e))     

# run from a thread, continiously try to join any network in the ota_wifi list.
# _thread.start_new_thread(_wifiSTAconnector)
def wifiTask():
    global wifi_wdt_lock, wifi_stop_flag

    logger = Logger(name = 'WIFI ' + __version__,level=logging.INFO,filename=None)
    logger.info('** WIFI Task started **')

    wlan=WLAN()
    wlan.deinit()
    uid=str(binascii.hexlify(machine.unique_id())[6:])
    Ssid ='ENS-PP-'+uid[2:8]
    logger.info('Using Hidden AP SSID:' + Ssid )

    wlan.init(mode=WLAN.AP, ssid=Ssid, auth=(WLAN.WPA2,'Li56#fd0gertQQ'), channel=8, antenna=WLAN.INT_ANT, hidden=True)

    server = network.Server()
    server.deinit() # disable the server
    # enable the server again with new settings
    server.init(login=('ensuser', 'n0ty0urbu5ine55'), timeout=120)

    wifi_ap_init_done=True    

    while not wifi_stop_flag.locked():
        if wifi_wdt_lock.locked():
            wifi_wdt_lock.release()
        time.sleep(1)
        if fota_wifi_release_sem.locked() and wifi_ap_init_done == True:
            wlan.deinit()
            wifi_ap_init_done = False
            wlan=None
            logger.info('Deinit AP mode for FOTA')

    try:
        wlan.deinit()
    except:
        pass

    logger.error('** WIFI Task ended **')