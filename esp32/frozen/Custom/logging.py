#!/usr/bin/env python
"""logging.py: basic logger"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="Electronet ltd 2018"

import sys
import uos
import time
from machine import SD
import _thread
import machine

CRITICAL = 50
ERROR    = 40
WARNING  = 30
INFO     = 20
DEBUG    = 10
NOTSET   = 0

MAXNUMBEROFCHARACTERS   = 1024*20 # default is 20K
NAMEFILESD = '/sd/log'

_level_dict = {
    CRITICAL: "CRIT",
    ERROR: "ERROR",
    WARNING: "WARN",
    INFO: "INFO",
    DEBUG: "DEBUG",
}

_stream = sys.stderr
_sd = None
log_lock = _thread.allocate_lock()

class Logger:

    def __init__(self, name,level=NOTSET,filename=None,maxfilesize=MAXNUMBEROFCHARACTERS):
        global _sd
        self.level = level
        self.name = name
        self.filename = filename
        self.numberOfCharacterWritten = 0
        self.currentIndex = 0
        self.lasttimestampsd = None
        self.maxfilesize = maxfilesize
        #read the size of the file
        try:
            if(self.filename != None):
                self.numberOfCharacterWritten = uos.stat(self.getCurrentLogFileName())[6]
                if self.numberOfCharacterWritten > self.maxfilesize:
                    self.currentIndex = 1 # we must have been working from the next index log
                    self.numberOfCharacterWritten = uos.stat(self.getCurrentLogFileName())[6]

        except:
            pass

        
        #try to mount the SD card if present
        try:
            if(self.filename != None and _sd == None):
                _sd = SD()
                uos.mount(_sd,'/sd')
        except:
            with log_lock:
                print("No SD card present")
            pass
        with log_lock:
            print(self.name +" logging to " + (self.getCurrentLogFileName()))

    def _level_str(self, level):
        if level in _level_dict:
            return _level_dict[level]
        return "LVL" + str(level)

    def log(self, level, msg, *args):
        try:
            if level >= (self.level):
                localtime = time.localtime()
                with log_lock:
                    print(("{}:{}:{}|{}:{}:" + str(msg)).format(localtime[3],localtime[4],localtime[5],self.name,self._level_str(level)))
        except:
            pass

    def getCurrentLogFileName(self):
        if(self.filename != None):
            return  '/flash/' + self.filename + str(self.currentIndex) + '.log'
        else:
            return 'console only'

    def getCurrentLogFileNameSD(self):
        return  NAMEFILESD + self.filename + str(self.lasttimestampsd) + '.log'
    
    def rotateLogFile(self, timestamp):
        #copy file to SD if possible
        global _sd
        if _sd != None:
            try:
                self.lasttimestampsd = timestamp
                f_flash = open(self.getCurrentLogFileName(), 'r')
                f_sd = open(self.getCurrentLogFileNameSD(), 'w')
                copyfile(f_flash,f_sd)
                f_flash.close()
                f_sd.close()                
            except Exception as e:
                with log_lock:
                    print("Error writting to SD card" + str(e))

        #swap index for the flash log file
        if self.currentIndex == 0:
            self.currentIndex = 1
        else:
            self.currentIndex = 0

        #erase old file
        f = open(self.getCurrentLogFileName(), 'w')
        f.write('')
        f.close()
        
        with log_lock:
            print("Rotate Log file to " + (self.getCurrentLogFileName()))
            
    def debug(self, msg, *args):
        self.log(DEBUG, msg, *args)

    def info(self, msg, *args):
        self.log(INFO, msg, *args)

    def warning(self, msg, *args):
        self.log(WARNING, msg, *args)

    def error(self, msg, *args):
        self.log(ERROR, msg, *args)

    def critical(self, msg, *args):
        self.log(CRITICAL, msg, *args)

    def meminfodump(self):
        with log_lock:
            machine.info()

# static functions
def copyfile(source, dest, buffer_size=128):
    while 1:
        copy_buffer = source.read(buffer_size)
        if not copy_buffer:
            break
        dest.write(copy_buffer)
