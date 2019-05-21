import binascii, array, math
try:
    import ujson
except:
    import json as ujson
import hashlib

import gc

try:
    import _thread as thread
except:
    import thread as thread
import time

#seom helpers function
def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

# see https://powerpilot.visualstudio.com/PowerPilot/_wiki/wikis/PowerPilot.wiki?wikiVersion=GBwikiMaster&pagePath=%2FLoop%20Impedance%20Algorithm
# for more info

class LoopImpedance:

        def __init__(self, Imin=5000, Vmin=1000, ZEmin=200, Qmax=10.0, Tstable=3, ZEmatch=1.0, EventTableSize =20, Tstableperc = 1.00 ):
            self._Imin = Imin
            self._Vmin = Vmin
            self._ZEmin = ZEmin
            self._Qmax = Qmax
            if (Tstable < 3):
                Tstable = 3
            self._Tstable = Tstable
            self._ZEmatch = ZEmatch
            self._EventTableSize = EventTableSize
            self._Tstableperc = Tstableperc
            self.Ze = None  # lastest Ze
            self._index = 0
            self._counter = 0
            #define the lenght of the 2 stable periods
            self._startFirstPeriod =0
            self._endFirstPeriod=  self._Tstable - 2
            self._startSecondPeriod = self._Tstable + 1
            self._endSecondPeriod = self._Tstable  * 2

            self._fifo_current = array.array('i',(0 for i in range(0,self._Tstable *2)))
            self._fifo_voltage = array.array('i',(0 for i in range(0,self._Tstable *2)))
            self._fifo_reactivepower = array.array('i',(0 for i in range(0,self._Tstable *2)))
            self._event_list_ze = array.array('i',(0 for i in range(0,EventTableSize)))
            self._event_list_current = array.array('i',(0 for i in range(0,EventTableSize)))
            self._event_list_voltage= array.array('i',(0 for i in range(0,EventTableSize)))
            self._event_list_timestamp = array.array('i',(0 for i in range(0,EventTableSize)))
            self._event_list_matched = array.array('b',(False for i in range(0,EventTableSize)))
        

        def _insertElementInFifo(self, array, newVal):
            # first move all element by one index (erase index 0)            
            sizeofarray= len(array)          
            for index in range(0,sizeofarray -1):
                array[index] = array[index+1]
            #insert new val at the end
            array[sizeofarray-1] = newVal   
        
        def _isStable(self, array, limit):
            minimum = float(min(array))
            maximum = float(max(array))
            average = float(mean(array))
            if average == 0:
                return True
            if math.fabs(((maximum-average) *100 / average)) <  limit and math.fabs(((average-minimum) * 100 / average)) <  limit:
                return True
            else:
                return False
        
        # units are mA, mV and W , all integers
        def _addNewMeasurment(self, current, voltage, reactivePower):
            self._insertElementInFifo( self._fifo_current, current)
            self._insertElementInFifo( self._fifo_voltage, voltage)
            self._insertElementInFifo( self._fifo_reactivepower, reactivePower)
            if self._index <= self._Tstable * 2:
                self._index = self._index + 1
        
        def _validateRules(self, Ze):
            if self._index < self._Tstable * 2:
                return False # not enough data yet
            # check all rules here
            if math.fabs(self._fifo_current[self._endFirstPeriod] - self._fifo_current[self._startSecondPeriod] ) > self._Imin :                
                if math.fabs(self._fifo_voltage[self._endFirstPeriod] - self._fifo_voltage[self._startSecondPeriod] ) > self._Vmin :
                    if self._isStable(self._fifo_current[self._startFirstPeriod:self._endFirstPeriod+1],self._Tstableperc) and self._isStable(self._fifo_current[self._startSecondPeriod:self._endSecondPeriod],self._Tstableperc):
                        if self._isStable(self._fifo_voltage[self._startFirstPeriod:self._endFirstPeriod+1],self._Tstableperc) and self._isStable(self._fifo_voltage[self._startSecondPeriod:self._endSecondPeriod],self._Tstableperc):
                            if self._fifo_current[self._startSecondPeriod] == 0 or (self._fifo_current[self._startSecondPeriod] *self._fifo_voltage[self._startSecondPeriod]  * self._Qmax / 100 ) >= (1000 * 1000 * self._fifo_reactivepower[self._startSecondPeriod]):
                                if Ze > self._ZEmin:
                                     return True                            
            return False
        
        def _calculateZe(self):
            if math.fabs(self._fifo_current[self._endFirstPeriod] - self._fifo_current[self._startSecondPeriod] ) != 0 :
                return int(math.fabs(self._fifo_voltage[self._endFirstPeriod] - self._fifo_voltage[self._startSecondPeriod] )  * 1000 / math.fabs(self._fifo_current[self._endFirstPeriod] - self._fifo_current[self._startSecondPeriod] ))
            else:
                return None

        def _calculateCurrentDelta(self):
                return self._fifo_current[self._startSecondPeriod] - self._fifo_current[self._endFirstPeriod]  

        def _calculateVoltageDelta(self):
            return self._fifo_voltage[self._startSecondPeriod ] - self._fifo_voltage[self._endFirstPeriod] 
        
        def _printEventsTable(self,start=0):
            for ii in range(start,self._EventTableSize):
                print ( str(ii) + ":" + str(self._event_list_ze[ii]) + "mOhm " + str(self._event_list_current[ii]) + "mA " + str(self._event_list_voltage[ii]) + "mV  " + str(self._event_list_matched[ii]))


        def _addNewEvent(self, Ze, current,voltage, timestamp):
             self._insertElementInFifo( self._event_list_ze, Ze)
             self._insertElementInFifo( self._event_list_current,current)
             self._insertElementInFifo( self._event_list_voltage,voltage)
             self._insertElementInFifo( self._event_list_timestamp, timestamp)
             self._insertElementInFifo( self._event_list_matched, False)

        def _matchEvents(self):
            # try to match the last event first , stop if one is found
            for reference in range (self._EventTableSize-1, 1 , -1):
                ref_current = self._event_list_current[reference]
                ref_voltage = self._event_list_voltage[reference]
                ref_ze = self._event_list_ze[reference]
                # index 0 is the oldest
                for index in range (reference -1, 0, -1):
                    current = self._event_list_current[index]
                    voltage = self._event_list_voltage[index]
                    ze = self._event_list_ze[index]
                    if ze > 0 and math.fabs((ref_current + current) * 100 / current ) < self._ZEmatch and self._event_list_matched[index] == False and self._event_list_matched[reference] == False:
                        self._event_list_matched[index] = True
                        self._event_list_matched[reference] = True
                        return (ze + ref_ze ) / 2            
              
            return None
        
        def getZe(self):
            return self.Ze
        
        # unit are A,V and W, s
        # return True if a new Ze value has been found
        def addMeasurment(self,V,I,Q,TS):
            self._addNewMeasurment(int(I*1000),int(V*1000),int(Q))
            self._counter = self._counter + 1
            if self._counter > 2 :  # jumps 2 slots after a successful read to avoid recording multiple instance of the same event             
                ze = self._calculateZe()
                if self._validateRules(ze) == True:
                    self._counter = 0                    
                    self._addNewEvent(ze,self._calculateCurrentDelta(),self._calculateVoltageDelta(),TS)
                    ze = self._matchEvents()
                    if ze != None and self.Ze != ze:
                        self.Ze = ze 
                        return True                
            return False



