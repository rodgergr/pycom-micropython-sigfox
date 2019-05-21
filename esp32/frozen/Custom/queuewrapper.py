#!/usr/bin/env python
"""queuewrapper.py: PowerPilot QueueWrapper """
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

import queue
import _thread
import time

# use circular FIFO buffer logic
# the current underlaying queue implementation is faulty resulting in coredumps.
# so we use our own application layer version
class QueueWrapper:
    def __init__(self,size,overwrite=False):
        self._queue = {}
        self._index_read = 1
        self._overwrite = overwrite
        self._index_write = 1
        self._size = size
        self.lock = _thread.allocate_lock()
        self.FULL = "FULL"
        self.SUCCESS = "SUCCESS"
        for i in range (1 , self._size+1):
            self._queue[i]=None

    def put(self, data):
        res = self.FULL
        with self.lock:
            if self._overwrite == False and self._queue[self._index_write] != None :  # no room at present
                return res
            else:
                self._queue[self._index_write] = data # copy the data over
                # find the next position for our write index
                if self._index_write == self._size:
                    self._index_write = 1  # go back at the start
                else:
                    self._index_write = 1 + self._index_write  # inc
                res = self.SUCCESS
        return res

    def get(self):
        with self.lock:
            res = self._queue[self._index_read]
            if res != None :  # something is in the queue
                if self._overwrite == False: 
                    self._queue[self._index_read] = None  # clean the spot
                if self._index_read == self._size:
                    self._index_read = 1  # go back at the start
                else:  
                    self._index_read = 1 + self._index_read  # just inc
            return res


#do not use!
class QueueWrapper2:

    def __init__(self,size):
        self._queue = queue.Queue(size)
        self.FULL = "FULL"
        self.SUCCESS = "SUCCESS"

    def put(self, data):
        try:
            self._queue.put(data,block = False)            
        except OSError:
            return self.FULL        
        return self.SUCCESS

    def get(self,block=False,timeout=None):
        try:
            res = self._queue.get(block,timeout)       
        except OSError:
            return None

        return res
           
      