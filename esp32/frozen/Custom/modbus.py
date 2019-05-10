#!/usr/bin/env python
"""modbus.py: PowerPilot python Modbus"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2019"

#
# MODBUS DRiver for Pilot SPM91/SPM93  Phase Meters
# *************************************************
#
# Requests data from mbAddress and stores values in data structure
# Calculates CRC checksum using high and low byte lookup tables

# Need to re-factor this in a class structure for all models, base: Pilot,  derived: SPM91, SPM93, etc.. and use the modbus library from Pycom

import pycom
import time
import binascii
import struct
from machine import I2C
from machine import Pin
from machine import UART
import gc
from struct import *
from globs import *
import math
import _thread

extension_wdt_lock = _thread.allocate_lock()
mb_error_lock = _thread.allocate_lock()

MB_SUCCESS = "Success"; MB_FAIL = "Fail"; MB_ERROR_NODATA = "NoData"; MB_ERROR_WRONG_ADDRESS = "WrongAddress"
MB_ERROR_WRONG_FUNCTION = "WrongFunction"; MB_ERROR_NUMCHARS = "WrongNbOfChar"; MB_ERROR__CRC16 = "WrongCRC";MB_ERROR_UNKNONWN_FUNCTION = "UnknownFunction"


#  Table of CRC values for high-order byte
auchCRCHi = b'\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\
\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\
\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\
\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\
\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\
\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\
\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\
\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\
\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\
\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\
\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\
\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\
\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\
\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\
\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\
\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\
\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\
\x40'


# /* Table of CRC values for low-order byte */
auchCRCLo = b'\x00\xC0\xC1\x01\xC3\x03\x02\xC2\xC6\x06\x07\xC7\x05\xC5\xC4\
\x04\xCC\x0C\x0D\xCD\x0F\xCF\xCE\x0E\x0A\xCA\xCB\x0B\xC9\x09\
\x08\xC8\xD8\x18\x19\xD9\x1B\xDB\xDA\x1A\x1E\xDE\xDF\x1F\xDD\
\x1D\x1C\xDC\x14\xD4\xD5\x15\xD7\x17\x16\xD6\xD2\x12\x13\xD3\
\x11\xD1\xD0\x10\xF0\x30\x31\xF1\x33\xF3\xF2\x32\x36\xF6\xF7\
\x37\xF5\x35\x34\xF4\x3C\xFC\xFD\x3D\xFF\x3F\x3E\xFE\xFA\x3A\
\x3B\xFB\x39\xF9\xF8\x38\x28\xE8\xE9\x29\xEB\x2B\x2A\xEA\xEE\
\x2E\x2F\xEF\x2D\xED\xEC\x2C\xE4\x24\x25\xE5\x27\xE7\xE6\x26\
\x22\xE2\xE3\x23\xE1\x21\x20\xE0\xA0\x60\x61\xA1\x63\xA3\xA2\
\x62\x66\xA6\xA7\x67\xA5\x65\x64\xA4\x6C\xAC\xAD\x6D\xAF\x6F\
\x6E\xAE\xAA\x6A\x6B\xAB\x69\xA9\xA8\x68\x78\xB8\xB9\x79\xBB\
\x7B\x7A\xBA\xBE\x7E\x7F\xBF\x7D\xBD\xBC\x7C\xB4\x74\x75\xB5\
\x77\xB7\xB6\x76\x72\xB2\xB3\x73\xB1\x71\x70\xB0\x50\x90\x91\
\x51\x93\x53\x52\x92\x96\x56\x57\x97\x55\x95\x94\x54\x9C\x5C\
\x5D\x9D\x5F\x9F\x9E\x5E\x5A\x9A\x9B\x5B\x99\x59\x58\x98\x88\
\x48\x49\x89\x4B\x8B\x8A\x4A\x4E\x8E\x8F\x4F\x8D\x4D\x4C\x8C\
\x44\x84\x85\x45\x87\x47\x46\x86\x82\x42\x43\x83\x41\x81\x80\
\x40'

# the maximum length is 128 bytes per message, so no more than 124 bit for payload: Max 62 x 16-bit register per message 
rxBuf = bytearray(150)
#rxdataOffset = 8
rxBufRaw = bytearray(150)
txBuf = bytearray(8)
mBuf = bytearray(200)
data = {}
data['voltage'] = 0
data['voltage2'] = 0
data['voltage3'] = 0
data['current'] = 0
data['current2'] = 0
data['current3'] = 0
data['activepower'] = 0
data['activepower2'] = 0
data['activepower3'] = 0
data['apparentpower'] = 0
data['reactivepower'] = 0
data['reactivepower2'] = 0
data['reactivepower3'] = 0
data['frequency'] = 0
data['powerfactor'] = 0
data['powerfactor2'] = 0
data['powerfactor3'] = 0
data['importActiveEnergy'] = 0
data['exportActiveEnergy'] = 0
data['importReactiveEnergy'] = 0
data['exportReactiveEnergy'] = 0
data['totalReactiveEnergy'] = 0
data['totalActiveEnergy'] = 0
data['timestamp'] = 0
data['thdv'] = 0
data['thdv2'] = 0
data['thdv3'] = 0
data['thdi'] = 0
data['thdi2'] = 0
data['thdi3'] = 0
data['hv_3'] = 0
data['hv_5'] =0
data['hv_7'] = 0
data['hv_9'] = 0
data['hv_11'] = 0
data['hv_13'] = 0
data['hv2_3'] =0
data['hv2_5'] = 0
data['hv2_7'] = 0
data['hv2_9'] = 0
data['hv2_11'] = 0
data['hv2_13'] = 0
data['hv3_3'] =0
data['hv3_5'] = 0
data['hv3_7'] = 0
data['hv3_9'] = 0
data['hv3_11'] = 0
data['hv3_13'] = 0

READ_REGS = 0x03
WRITE_REGS = 0x10
RSE=Pin('P11', mode=Pin.OUT)
#TX data is received by the UART 

uart1=UART(1)

latestMBError=MB_SUCCESS

MAXINT16 = 32767.0
MININT16 = -32768.0
MAXUINT16 = 65535.0

def getDynamicPowerGain(v,i):
	power_total = v * i # i and v are always positive
	if power_total > MAXINT16 and power_total <= MAXINT16 * 10:
		return 10
	elif power_total > MAXINT16 * 10:
		return 100
	else :
		return 1
	

def CAPPED(val, max, min):
	if val > max :
		return max
	elif val < min :
		return min
	else :
		return val
	
def UINT16_CAPPED(val) :
	return CAPPED(val,MAXUINT16, 0 )

def INT16_CAPPED(val) :
	return CAPPED(val,MAXINT16, MININT16 )

def UINT8_CAPPED(val) :
	return CAPPED(val,255, 0 )


def _lowByte(num):
	return(num & 0xFF)

def _highByte(num):
	return((num & 0xFF00) >> 8)


def _CRC16(puchMsg, usDataLen):
# puchMsg-  message to calculate CRC upon
# usDataLen-  number of bytes in message
	uchCRCHi = 0xFF					# high byte of CRC initialized
	uchCRCLo = 0xFF					# low byte of CRC initialized
	for x in range(usDataLen): 				# pass through message buffer
		uIndex = uchCRCLo ^ puchMsg[x]		# will index into CRC lookup table
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex]
		uchCRCHi = auchCRCLo[uIndex]
	return (uchCRCHi << 8 | uchCRCLo)		# Return CRC already byte swapped

def _getReg(mbReg):
	global mBuf
	i = (mbReg - 40001) * 2			# Get unsigned 16 bit register from buffer
	s16 = mBuf[i] << 8 | mBuf[i+1]
	if (s16 & 0x8000):	# check sign bit
		return -(~s16 + 0xffff)
	return(s16)

def _getRegUint16(mbReg):
	global mBuf
	i = (mbReg - 40001) * 2			# Get unsigned 16 bit register from buffer
	s16 = mBuf[i] << 8 | mBuf[i+1]
	return(s16)

def _getDReg(mbReg):						# Get 32 bit register by calling _getDReg twice for consecutive registers
	i = (mbReg - 40001) * 2			# Get unsigned 16 bit register from buffer
	s32 = mBuf[i] << 8 | mBuf[i+1] | mBuf[i+2] << 24 | mBuf[i+3] << 16
	if (s32 & 0x80000000):	# check sign bit
		return -(~s32 + 0xffffffff)
	return(s32)

def _getDRegUint32(mbReg):						# Get 32 bit register by calling _getDReg twice for consecutive registers
	i = (mbReg - 40001) * 2			# Get unsigned 16 bit register from buffer
	s32 = mBuf[i] << 8 | mBuf[i+1] | mBuf[i+2] << 24 | mBuf[i+3] << 16
	return(s32)

def _mbWrite(txBuf):
	global uart1
	uart1.readall()
	RSE.value(1)
	uart1.write(txBuf)
	# print("txBuf = {:30s}".format(binascii.hexlify(txBuf," ")))
	time.sleep_ms(10)
	RSE.value(0)

def _rcvModbusData(mbAddress, numRegisters,rxdataOffset,function=READ_REGS,startRegister=0):
	global rxBuf
	global rxBufRaw
	global data
	global uart1
	global latestMBError

	

	nbrOFCharToRead =  rxdataOffset + 2 * numRegisters + 5
	if function == WRITE_REGS:
		nbrOFCharToRead = 8	
	
	

	rxChars = uart1.readinto(rxBufRaw, nbrOFCharToRead)

	if (rxChars == None) or rxChars == rxdataOffset:   # Read response bytes into buffer
		#print("No Data Received using address: " + str(mbAddress))
		return(MB_ERROR_NODATA)				# Return NO_DATA if nothing received or timed out
	rxBuf = rxBufRaw[rxdataOffset:]
	rxChars = rxChars -rxdataOffset

	#print("{:d} chars received : rxBuf = {:50s}".format(rxChars,binascii.hexlify(rxBuf,' ')))

	if (rxBuf[0] != mbAddress):
		print("MB WRONG ADDESS")
		with mb_error_lock:
			latestMBError =MB_ERROR_WRONG_ADDRESS
		return(MB_ERROR_WRONG_ADDRESS)		# Return with WRONG_ADDRESS error code

	if (rxBuf[1] != function):
		print("MB WRONG FUNCTION!")
		with mb_error_lock:	
			latestMBError =MB_ERROR_WRONG_FUNCTION
		return(MB_ERROR_WRONG_FUNCTION)		# Return with WRONG_FUNC error code
		
	CRCindex = 0

	if (function == READ_REGS ) :
		if (rxBuf[2] != numRegisters * 2):
			print("MB WRONG NUM CHARS")
			with mb_error_lock:
				latestMBError =MB_ERROR_NUMCHARS
			return(MB_ERROR_NUMCHARS)				# Return with WRONG_CRC error code
		
		CRCindex =  numRegisters * 2 + 3	# default to READ
	
	elif (function == WRITE_REGS ) :

		if ((rxBuf[2] != _highByte(startRegister - 40001)) or
		(rxBuf[3] != _lowByte(startRegister - 40001)) or 
		(rxBuf[4] != _highByte(numRegisters))  or
		(rxBuf[5] != _lowByte(numRegisters))):
			print("MB WRONG NUM CHARS")
			with mb_error_lock:
				latestMBError =MB_ERROR_NUMCHARS
			return(MB_ERROR_NUMCHARS)				# Return with WRONG_CRC error code

		CRCindex =  6			# Calculate end of payload (start of checksum)
	
	else :
		print("MB UNKNOWN FUNCTION ")
		with mb_error_lock:	
			latestMBError =MB_ERROR_WRONG_FUNCTION
		return(MB_ERROR_WRONG_FUNCTION)		# Return with WRONG_FUNC error code

	if (_CRC16(rxBuf, CRCindex) != (rxBuf[CRCindex + 1] << 8 | rxBuf[CRCindex])):	# Calculate _CRC16 of payload and compare with message _CRC16 checksum
		print("MB WRONG CRC")
		with mb_error_lock:
			latestMBError = MB_ERROR__CRC16
		return(MB_ERROR__CRC16)				# Return with WRONG_CRC error code

	return(MB_SUCCESS)		# Return SUCCESS

def _reqModbusData(mbAddress, mbFunction, startRegister, numRegisters,mode=M11):
	global txBuf
	txBuf[0] = mbAddress						# Load Modbus Address (normally 1)
	txBuf[1] = mbFunction						# Load function cose (03 for read register)
	txBuf[2] = _highByte(startRegister - 40001)	# Starting address high byte
	txBuf[3] = _lowByte(startRegister - 40001)	# Starting address low byte
	txBuf[4] = _highByte(numRegisters)			# No. of registers high byte
	txBuf[5] = _lowByte(numRegisters)			# No. of registers low byte (40001 to 40024)
	CRC = _CRC16(txBuf, 6)						# Calculate _CRC16 checksum of entire message (6 bytes)
	txBuf[6] = _lowByte(CRC)						# Load first CRC byte into buffer
	txBuf[7] = _highByte(CRC)					# Load next CRC byte into buffer
	_mbWrite(txBuf)								# Output buffer to RS485 converter
	time.sleep_ms(150)
	#print(" txBuf = {:8s}".format(binascii.hexlify(txBuf,' ')))

	if mode == M11:
		return(_rcvModbusData(mbAddress, numRegisters,0,READ_REGS))
	else:
		return(_rcvModbusData(mbAddress, numRegisters,8,READ_REGS))
	

def _setModbusData(mbAddress, startRegister, numRegisters, newval, mode=M11):
	if numRegisters > 2 :
		print("No more than 2 registers at a time")
		return MB_ERROR_NUMCHARS
	txBufSet = bytearray(numRegisters*2+9)
	#txBufSet = bytearray(16)
	txBufSet[0] = mbAddress						# Load Modbus Address (normally 1)
	txBufSet[1] = WRITE_REGS					# Load function cose (16 for write registers)
	txBufSet[2] = _highByte(startRegister - 40001)	# Starting address high byte
	txBufSet[3] = _lowByte(startRegister - 40001)	# Starting address low byte
	txBufSet[4] = _highByte(numRegisters)			# No. of registers high byte
	txBufSet[5] = _lowByte(numRegisters)			# No. of registers low byte (40001 to 40024)
	txBufSet[6] = _lowByte(numRegisters*2)			# No. of byte to folow (40001 to 40024)
	# copy all bytes
	for i in range (0,numRegisters*2):
		txBufSet[7+i]=newval[i]
	
	CRC = _CRC16(txBufSet, 7+numRegisters*2)						# Calculate _CRC16 checksum of entire message (6 bytes)
	txBufSet[7+numRegisters*2] = _lowByte(CRC)						# Load first CRC byte into buffer
	txBufSet[8+numRegisters*2] = _highByte(CRC)					# Load next CRC byte into buffer
	#print(" txBufSet = {:16s}".format(binascii.hexlify(txBufSet,' ')))
	_mbWrite(txBufSet)
	time.sleep_ms(150)
	offset=8 # C11 
	if mode == M11:
		offset = 0
	return(_rcvModbusData(mbAddress, numRegisters,offset,WRITE_REGS,startRegister))


def _getMBRegisters(CT):
	if (_getDReg(40003) == 0) :
		return False#  zero is not a valid voltage!
		
	global data
	unixLocalTime= time.time() + time.timezone()
	data['totalActiveEnergy'] = _getDReg(40001) /10 * CT					# read Input Active Energy Accumulator in units of 0.1 kWh
	data['voltage'] = _getReg(40003) / 100.0						# read voltage
	data['current'] = _getDReg(40004) / 1000.0	* CT				# read current
	data['activepower'] = _getDReg(40006) / 10.0 * CT					# read active power
	data['apparentpower'] = _getDReg(40008) / 10.0	* CT			# read apparent power
	data['reactivepower'] = _getDReg(40010) / 10.0	* CT			# read reactive power
	data['frequency'] = _getReg(40012) / 100.0					# read frequency
	data['powerfactor'] = _getReg(40013) / 1000.0				# read frequency
	data['importActiveEnergy'] = _getDReg(40014) /10 * CT			# read Input Active Energy
	data['exportActiveEnergy'] = _getDReg(40016) / 10 * CT			# read Export Active Energy
	data['importReactiveEnergy'] = _getDReg(40020) / 10 * CT			# read Input Reactive Energy
	data['exportReactiveEnergy'] = _getDReg(40022) / 10	* CT		# read Export Reactive Energy
	data['totalReactiveEnergy'] = _getDReg(40024) / 10 * CT			# read Input Reactive Energy
	data['timestamp'] = unixLocalTime

	return True

def _readM11(mbAddress,CT):
	global mBuf
	global rxBuf
	mbGoodRead = False
	tryCount=0
	while(not mbGoodRead and tryCount < 3):
		tryCount += 1
		time.sleep_ms(300)
		# modbusLogger.debug("Try {:d} at reading Modbus".format(tryCount))
		if(_reqModbusData(mbAddress, READ_REGS, 40001, 12) == MB_SUCCESS):	# Request 12 x 16 bit registers starting at 40001		
 			mBuf = rxBuf[3:27]
			#time.sleep_ms(300)								# Put first 12 mb registers into mBuf
			if(_reqModbusData(mbAddress, READ_REGS, 40013, 5) == MB_SUCCESS):	# Request 5 x 16 bit registers starting at 40013
				mBuf += rxBuf[3:13] + bytearray(4)				# Put registers 40013-17 into mBuf and pad for 40018,19 (reserved)
				#time.sleep_ms(300)
				if(_reqModbusData(mbAddress, READ_REGS, 40020, 6) == MB_SUCCESS):	# Request 6 x 16 bit registers starting at 40020
					mBuf += rxBuf[3:15]								# Put registers 40020-26 into mBuf
					mbGoodRead = True
	if(mbGoodRead):
		#print("mBuf = {:75s}".format(binascii.hexlify(mBuf,' ')))
		if _getMBRegisters(CT) == True:
			return(MB_SUCCESS)
	return(MB_FAIL)

def _readSPM91(mbAddress, CT):
	global mBuf
	global rxBuf
	mbGoodRead = False
	tryCount=0
	while(not mbGoodRead and tryCount < 3):
		tryCount += 1
		time.sleep_ms(300)
		# modbusLogger.debug("Try {:d} at reading Modbus".format(tryCount))
		if(_reqModbusData(mbAddress, READ_REGS, 40001, 25,C11) == MB_SUCCESS):	# Request 12 x 16 bit registers starting at 40001		
 			mBuf = rxBuf[3:53]  			
			mbGoodRead = True
	if(mbGoodRead):
		#print("mBuf = {:75s}".format(binascii.hexlify(mBuf,' ')))
		if _getMBRegisters(CT) == True:
			return(MB_SUCCESS)
	return(MB_FAIL)

def _getMBRegistersSPM93():
	#before we udpate, let validate this data...
	if (_getReg(40101-100)== 0 or _getReg(40102-100) ==0 or _getReg(40103-100)==0) :
		return False #  zero is not a valid voltage!
	global data
	CT = _getRegUint16(44002-3942)   # the PT ratio is not supported  (LV only)
	data['ctratio'] = CT
	unixLocalTime= time.time() + time.timezone()
	data['voltage'] = _getRegUint16(40101-100) / 100.0				    # read voltage (phase Red)
	data['voltage2'] = _getRegUint16(40102-100) / 100.0				# read voltage (phase White)
	data['voltage3'] = _getRegUint16(40103-100) / 100.0				# read voltage (phase Blue)
	data['current'] = _getDRegUint32(40107-100) / 1000.0 * CT				# read current (phase Red)
	data['current2'] = _getDRegUint32(40109-100) / 1000.0 * CT				# read current (phase White)
	data['current3'] = _getDRegUint32(40111-100) / 1000.0 * CT				# read current (phase Blue)
	data['activepower'] = _getDReg(40115-100) / 100.0 * CT			# read active power (phase Red)
	data['activepower2'] = _getDReg(40117-100) / 100.0 * CT		# read active power (phase Blue)
	data['activepower3'] = _getDReg(40119-100) / 100.0 * CT		# read active power (phase White)
	data['reactivepower'] = _getDReg(40123-100) / 100.0 * CT			# read reactive power (phase Red)
	data['reactivepower2'] = _getDReg(40125-100) / 100.0 * CT			# read reactive power (phase Blue)
	data['reactivepower3'] = _getDReg(40127-100) / 100.0  * CT			# read reactive power (phase white)
	data['frequency'] = _getRegUint16(40143-100) / 100.0				# read frequency 
	data['apparentpower'] = _getDReg(40131-100) / 100.0 * CT			# read apparent power (phase 1)
	data['apparentpower2'] = _getDReg(40133-100) / 100.0 * CT				# read apparent power (phase 1)
	data['apparentpower3'] = _getDReg(40135-100) / 100.0 * CT				# read apparent power (phase 1)
	data['powerfactor'] = _getRegUint16(40139-100) / 1000.0				# read power factor
	data['powerfactor2'] = _getRegUint16(40140-100) / 1000.0				# read power factor
	data['powerfactor3'] = _getRegUint16(40141-100) / 1000.0				# read power factor
	data['importActiveEnergy'] = _getDRegUint32(41001-953) / 10 * CT			# read Input Active Energy (total all phases)
	data['exportActiveEnergy'] = _getDRegUint32(41003-953) / 10 * CT		# read Export Active Energy (total all phases)
	data['totalActiveEnergy'] = _getDRegUint32(41005-953) /10 * CT		# read Total Active Energy (total all phases)
	data['importReactiveEnergy'] = _getDRegUint32(41007-953) / 10 * CT		# read Input Reactive Energy (total all phases)
	data['exportReactiveEnergy'] = _getDRegUint32(41009-953) / 10 * CT		# read Export Reactive Energy (total all phases)
	data['totalReactiveEnergy'] = _getDRegUint32(41011-953) / 10 * CT			# read Total Reactive Energy (total all phases)
	data['timestamp'] = unixLocalTime

	return True

def _getMBRegistersSPM33(CTSecondary):
	#before we udpate, let validate this data...
	#if (_getReg(40001)== 0 or _getReg(40002) ==0 or _getReg(40003)==0) :
	#	print("Zero voltage")
	#	return False #  zero is not a valid voltage!
	global data
	unixLocalTime= time.time() + time.timezone()
	ctRatio = int(_getRegUint16(40041) / CTSecondary ) # the secondary is set to 5 on the SPM33
	data['ctratio'] = ctRatio
	data['voltage'] = _getRegUint16(40001) / 100.0				# read voltage (phase Red)
	data['voltage2'] = _getRegUint16(40002) / 100.0				# read voltage (phase White)
	data['voltage3'] = _getRegUint16(40003) / 100.0				# read voltage (phase Blue)
	data['current'] = _getRegUint16(40007) / 1000.0 * ctRatio		# read current (phase Red)
	data['current2'] = _getRegUint16(40008) / 1000.0 * ctRatio    # read current (phase White)
	data['current3'] = _getRegUint16(40009) / 1000.0 * ctRatio    # read current (phase Blue)
	data['activepower'] = _getReg(40016) / 10.0 * ctRatio					# read active power (phase Red)
	data['activepower2'] = _getReg(40017) / 10.0 * ctRatio				# read active power (phase Blue)
	data['activepower3'] = _getReg(40018) / 10.0 * ctRatio				# read active power (phase White)
	data['reactivepower'] = _getReg(40019) / 10.0 * ctRatio			# read reactive power (phase Red)
	data['reactivepower2'] = _getReg(40020) / 10.0 * ctRatio			# read reactive power (phase Blue)
	data['reactivepower3'] = _getReg(40021) / 10.0	* ctRatio		# read reactive power (phase white)
	data['frequency'] = _getRegUint16(40025) / 100.0				# read frequency 
	data['powerfactor'] = _getReg(40022) / 1000.0				# read power factor
	data['powerfactor2'] = _getReg(40023) / 1000.0				# read power factor
	data['powerfactor3'] = _getReg(40024) / 1000.0				# read power factor
	data['importActiveEnergy'] = _getDRegUint32(40030) / 10			# read Input Active Energy (total all phases)
	data['exportActiveEnergy'] = _getDRegUint32(40032) / 10			# read Export Active Energy (total all phases)
	data['importReactiveEnergy'] = _getDRegUint32(40034) / 10			# read Input Reactive Energy (total all phases)
	data['exportReactiveEnergy'] = _getDRegUint32(40036) / 10			# read Export Reactive Energy (total all phases)
	data['totalActiveEnergy'] = _getDRegUint32(40026) / 10			# read  Active Energy Accumulator in units of 0.1 kWh
	data['totalReactiveEnergy'] = _getDRegUint32(40028) / 10			# read Total Reactive Energy (total all phases)
	data['thdv'] = _getRegUint16(40801-759) / 10.0				    # read thd voltage (phase Red)
	data['thdv2'] = _getRegUint16(40802-759) / 10.0				    # read thd voltage (phase White)
	data['thdv3'] = _getRegUint16(40803-759) / 10.00				    # read thd voltage (phase Blue)
	data['thdi'] = _getRegUint16(40804-759) / 10.0				    # read thd voltage (phase Red)
	data['thdi2'] = _getRegUint16(40805-759) / 10.0				    # read thd voltage (phase White)
	data['thdi3'] = _getRegUint16(40806-759) / 10.0				    # read thd voltage (phase Blue)
	data['hv_3'] = _getRegUint16(40808-759) / 10.0				    # read 3rd harmonic voltage (phase Red)
	data['hv_5'] = _getRegUint16(40810-759) / 10.0				    # read 5th harmonic voltage (phase Red)
	data['hv_7'] = _getRegUint16(40812-759) / 10.0				    # read 7th harmonic voltage (phase Red)
	data['hv_9'] = _getRegUint16(40814-759) / 10.0				    # read 9th harmonic voltage (phase Red)
	data['hv_11'] = _getRegUint16(40816-759) / 10.0				    # read 11th harmonic voltage (phase Red)
	data['hv_13'] = _getRegUint16(40818-759) / 10.0				    # read 13th harmonic voltage (phase Red)
	data['hv2_3'] = _getRegUint16(40838-778) / 10.0				    # read 3rd harmonic voltage (phase White)
	data['hv2_5'] = _getRegUint16(40840-778) / 10.0				    # read 5th harmonic voltage (phase White)
	data['hv2_7'] = _getRegUint16(40842-778) / 10.0				    # read 7th harmonic voltage (phase White)
	data['hv2_9'] = _getRegUint16(40844-778) / 10.0				    # read 9th harmonic voltage (phase White)
	data['hv2_11'] = _getRegUint16(40846-778) / 10.0				    # read 11th harmonic voltage (phase White)
	data['hv2_13'] = _getRegUint16(40848-778) / 10.0				    # read 13th harmonic voltage (phase White)
	data['hv3_3'] = _getRegUint16(40868-797) / 10.0				    # read 3rd harmonic voltage (phase Blue)
	data['hv3_5'] = _getRegUint16(40870-797) / 10.0				    # read 5th harmonic voltage (phase Blue)
	data['hv3_7'] = _getRegUint16(40872-797) / 10.0				    # read 7th harmonic voltage (phase Blue)
	data['hv3_9'] = _getRegUint16(40874-797) / 10.0				    # read 9th harmonic voltage (phase Blue)
	data['hv3_11'] = _getRegUint16(40876-797) / 10.0				    # read 11th harmonic voltage (phase Blue)
	data['hv3_13'] = _getRegUint16(40878-797) / 10.0				    # read 13th harmonic voltage (phase Blue)
	data['timestamp'] = unixLocalTime
	return True

def _readSPM93(mbAddress):
	global mBuf
	global rxBuf
	mbGoodRead = False
	tryCount=0
	while(not mbGoodRead and tryCount < 3):
		tryCount += 1
		# modbusLogger.debug("Try {:d} at reading Modbus".format(tryCount))
		if(_reqModbusData(mbAddress, READ_REGS, 40101, 47,C11) == MB_SUCCESS):# Request 47 x 16 bit registers starting at 40101
 			mBuf = rxBuf[3:97] # Put registers 40101-40147 into address 40001-40047, offset is -100						#
			if(_reqModbusData(mbAddress, READ_REGS, 41001, 12,C11) == MB_SUCCESS):# Request 12 x 16 bit registers starting at 41001
				mBuf += rxBuf[3:27]## Put registers 41001-41012 into address 40048-40059 , offset is -953
				if(_reqModbusData(mbAddress, READ_REGS, 44002, 1,C11) == MB_SUCCESS):# Request 1 x 16 bit registers starting at 44002
					mBuf += rxBuf[3:5]## Put registers 44002 into address 40060 , offset is -3942
					mbGoodRead = True

	if(mbGoodRead):
		if _getMBRegistersSPM93() == True:
			return(MB_SUCCESS)
	return(MB_FAIL)

def _readSPM33(mbAddress,CTSecondary):
	global mBuf
	global rxBuf
	mbGoodRead = False
	tryCount=0
	while(not mbGoodRead and tryCount < 3):
		tryCount += 1
		# modbusLogger.debug("Try {:d} at reading Modbus".format(tryCount))
		if(_reqModbusData(mbAddress, READ_REGS, 40001, 41,C11) == MB_SUCCESS):# Request 41 x 16 bit registers starting at 40001
 			mBuf = rxBuf[3:85] # Put registers 40001-40041 into address 40001-40041, offset is 0		#
			if(_reqModbusData(mbAddress, READ_REGS, 40801, 18,C11) == MB_SUCCESS):# Request 18 x 16 bit registers starting at 40801  Va harmonics
				mBuf += rxBuf[3:39]## Put registers 40801-40818 into address 40042-40059 , offset is -759
				if(_reqModbusData(mbAddress, READ_REGS, 40838, 11,C11) == MB_SUCCESS):# Request 12 x 16 bit registers starting at 40837   Vb harmonics
					mBuf += rxBuf[3:25]## Put registers 40838-40848 into address 40060-40070 , offset is -778
					if(_reqModbusData(mbAddress, READ_REGS, 40868, 11,C11) == MB_SUCCESS):# Request 12 x 16 bit registers starting at 40867   Vc harmonics
						mBuf += rxBuf[3:25]## Put registers 40868-40878 into address 40071-40081 , offset is -797
						mbGoodRead = True

	if(mbGoodRead):
		if _getMBRegistersSPM33(CTSecondary) == True:
			return(MB_SUCCESS)
	return(MB_FAIL)


def _getInstDataM11():
	unixLocalTime= time.time() + time.timezone()
	sMsg = ('"V":{:.1f},"I":{:.2f},"P":{:.1f},"Q":{:.1f},"S":{:.1f},'
		'"PF":{:.2f},"Mem":{:d},"T":{:d}').format(
		data['voltage'],data['current'],data['activepower'],data['reactivepower'],
		data['apparentpower'],data['powerfactor'],gc.mem_free(),unixLocalTime)
	return(sMsg)

def _getLoRaInstDataM11():
	myarray=bytearray(8)
	auto_scale_power_gain=getDynamicPowerGain((data['voltage']),(data['current']))
	myarray=pack('HHhh',UINT16_CAPPED(int(data['voltage'] * 100)),
						UINT16_CAPPED(int(data['current'] * 10)),
						INT16_CAPPED(int(data['activepower'] / auto_scale_power_gain)),
						INT16_CAPPED(int(data['reactivepower'] / auto_scale_power_gain))) # short unsigned 
	return [myarray,8]
		
def _getHHDataM11():
	sMsg = ('"i":{:d},"k":{:d},"j":{:d},"l":{:d},"m":"{}"'
		).format(int(data['importActiveEnergy']),int(data['exportActiveEnergy']),
		int(data['importReactiveEnergy']),int(data['exportReactiveEnergy']),data['Timestamp'])
	return(sMsg)   #test

def _getHHDataSPM93():
	sMsg = ('"i":{:d},"k":{:d},"j":{:d},"l":{:d},"m":"{}"'
		).format(int(data['importActiveEnergy']),int(data['exportActiveEnergy']),
		int(data['importReactiveEnergy']),int(data['exportReactiveEnergy']),data['Timestamp'])
	return(sMsg)  

def _getLoRaHHDataM11():
	myarray=bytearray(16)
	myarray=pack('iiii',int(data['importActiveEnergy'] ),
						int(data['exportActiveEnergy'] ),
						int(data['importReactiveEnergy'] ),
						int(data['exportReactiveEnergy'] ))
	return myarray

def _getLoRaHHDataSPM93():
	myarray=bytearray(16)
	myarray=pack('iiii',int(data['importActiveEnergy'] ),
						int(data['exportActiveEnergy'] ),
						int(data['importReactiveEnergy'] ),
						int(data['exportReactiveEnergy'] ))
	return myarray

def _getInstDataSPM93():

	# sMsg = ('"Va":{:.1f},"Vb":{:.1f},"Vc":{:.1f},"Ia":{:.2f},"Ib":{:.2f},'
	# 	'"Ic":{:.2f},"f":{:.2f},Mem":{:d},"T":{:d}').format(
	# 	data['Va'],data['Vb'],data['Vc'],data['Ia'],data['Ib'],data['Ic'])
	sMsg = ('"Va":{:.1f},"Vb":{:.1f},"Vc":{:.1f},"Ia":{:.2f},"Ib":{:.2f},"Ic":{:.2f},"Pa":{:.1f},"Pb":{:.1f},"Pc":{:.1f},"Qa":{:.1f},"Qb":{:.1f},"Qc":{:.1f},"f":{:.2f},"Mem":{:d},"T":{:d}').format(data['voltage'],
	    data['voltage2'],data['voltage3'],
		data['current'],data['current2'],data['current3'],
		data['activepower'],data['activepower2'],data['activepower3'],
		data['reactivepower'],data['reactivepower2'],data['reactivepower3'],
		data['frequency'],gc.mem_free(),data['timestamp'])
	return(sMsg)

def _getLoRaInstDataSPM93():
	myarray=bytearray(24)
	auto_scale_power_gain_1=getDynamicPowerGain((data['voltage']),(data['current']))
	auto_scale_power_gain_2=getDynamicPowerGain((data['voltage2']),(data['current2']))
	auto_scale_power_gain_3=getDynamicPowerGain((data['voltage3']),(data['current3']))
	myarray=pack('HHhhHHhhHHhh',UINT16_CAPPED(int(data['voltage'] * 100)),
							 UINT16_CAPPED(int(data['current'] * 10)),
							 INT16_CAPPED(int(data['activepower'] / auto_scale_power_gain_1)),
							 INT16_CAPPED(int(data['reactivepower'] / auto_scale_power_gain_1)),
							 UINT16_CAPPED(int(data['voltage2'] * 100)),
							 UINT16_CAPPED(int(data['current2'] * 10)),
							 INT16_CAPPED(int(data['activepower2'] /auto_scale_power_gain_2)),
							 INT16_CAPPED(int(data['reactivepower2'] / auto_scale_power_gain_2)),
							 UINT16_CAPPED(int(data['voltage3'] * 100)),
							 UINT16_CAPPED(int(data['current3'] * 10)),
							 INT16_CAPPED(int(data['activepower3'] / auto_scale_power_gain_3)),
							 INT16_CAPPED(int(data['reactivepower3'] /auto_scale_power_gain_3))) 

	return [myarray,24]

def _getLoRaInstDataSPM33(typereq):
	myarray=bytearray(42)
	auto_scale_power_gain_1=getDynamicPowerGain((data['voltage']),(data['current']))
	auto_scale_power_gain_2=getDynamicPowerGain((data['voltage2']),(data['current2']))
	auto_scale_power_gain_3=getDynamicPowerGain((data['voltage3']),(data['current3']))
	if typereq == 0:
		myarray=pack('HHhhHBBBBHHhhHBBBBHHhhHBBBB',UINT16_CAPPED(int(data['voltage'] * 100)),
								UINT16_CAPPED(int(data['current'] * 10)),
								INT16_CAPPED(int(data['activepower'] / auto_scale_power_gain_1)),
								INT16_CAPPED(int(data['reactivepower'] / auto_scale_power_gain_1)),
								UINT16_CAPPED(int(data['thdv']* 100)),
								UINT8_CAPPED(int(data['thdi'])),
								UINT8_CAPPED(int(data['hv_3'])),
								UINT8_CAPPED(int(data['hv_5'])),
								UINT8_CAPPED(int(data['hv_7'])),
								UINT16_CAPPED(int(data['voltage2'] * 100)),
								UINT16_CAPPED(int(data['current2'] * 10)),
								INT16_CAPPED(int(data['activepower2'] / auto_scale_power_gain_2)),
								INT16_CAPPED(int(data['reactivepower2'] / auto_scale_power_gain_2)),
								UINT16_CAPPED(int(data['thdv2']* 100)),
								UINT8_CAPPED(int(data['thdi2'])),
								UINT8_CAPPED(int(data['hv2_3'])),
								UINT8_CAPPED(int(data['hv2_5'])),
								UINT8_CAPPED(int(data['hv2_7'])),
								UINT16_CAPPED(int(data['voltage3'] * 100)),
								UINT16_CAPPED(int(data['current3'] * 10)),
								INT16_CAPPED(int(data['activepower3'] / auto_scale_power_gain_3)),
								INT16_CAPPED(int(data['reactivepower3'] / auto_scale_power_gain_3)),
								UINT16_CAPPED(int(data['thdv3']* 100)),
								UINT8_CAPPED(int(data['thdi3'])),
								UINT8_CAPPED(int(data['hv3_3'])),
								UINT8_CAPPED(int(data['hv3_5'])),
								UINT8_CAPPED(int(data['hv3_7']))
								)
	if typereq == 1:
			myarray=pack('HHhhHBBBBHHhhHBBBBHHhhHBBBB',UINT16_CAPPED(int(data['voltage'] * 100)),
							UINT16_CAPPED(int(data['current'] * 10)),
							INT16_CAPPED(int(data['activepower'] / auto_scale_power_gain_1)),
							INT16_CAPPED(int(data['reactivepower'] / auto_scale_power_gain_1)),
							UINT16_CAPPED(int(data['thdv']* 100)),
							UINT8_CAPPED(int(data['thdi'])),
							UINT8_CAPPED(int(data['hv_9'])),
							UINT8_CAPPED(int(data['hv_11'])),
							UINT8_CAPPED(int(data['hv_13'])),
							UINT16_CAPPED(int(data['voltage2'] * 100)),
							UINT16_CAPPED(int(data['current2'] * 10)),
							INT16_CAPPED(int(data['activepower2'] / auto_scale_power_gain_2)),
							INT16_CAPPED(int(data['reactivepower2'] / auto_scale_power_gain_2)),
							UINT16_CAPPED(int(data['thdv2']* 100)),
							UINT8_CAPPED(int(data['thdi2'])),
							UINT8_CAPPED(int(data['hv2_9'])),
							UINT8_CAPPED(int(data['hv2_11'])),
							UINT8_CAPPED(int(data['hv2_13'])),
							UINT16_CAPPED(int(data['voltage3'] * 100)),
							UINT16_CAPPED(int(data['current3'] * 10)),
							INT16_CAPPED(int(data['activepower3'] / auto_scale_power_gain_3)),
							INT16_CAPPED(int(data['reactivepower3'] / auto_scale_power_gain_3)),
							UINT16_CAPPED(int(data['thdv3']* 100)),
							UINT8_CAPPED(int(data['thdi3'])),
							UINT8_CAPPED(int(data['hv3_9'])),
							UINT8_CAPPED(int(data['hv3_11'])),
							UINT8_CAPPED(int(data['hv3_13']))
							)
	return [myarray,42]

#  thread safe  - > access global data structure data {}

def getLatestInstMeasurments(phase = PHASE_RED):
	res = None
	with extension_wdt_lock:
		if phase == PHASE_RED:
			res = [data['voltage'],data['current'],data['activepower'],data['reactivepower'], data['thdv']]
		elif phase == PHASE_WHITE:
			res = [data['voltage2'],data['current2'],data['activepower2'],data['reactivepower2'],data['thdv2']]
		elif phase == PHASE_BLUE:
			res = [data['voltage3'],data['current3'],data['activepower3'],data['reactivepower3'],data['thdv3']]
		elif phase == PHASE_ALL:
			res = [[data['voltage'],data['current'],data['activepower'],data['reactivepower'],data['thdv']],
				   [data['voltage2'],data['current2'],data['activepower2'],data['reactivepower2'],data['thdv2']],
				   [data['voltage3'],data['current3'],data['activepower3'],data['reactivepower3'],data['thdv3']]]
	return res

def getLatestMBError():
	res = None
	with mb_error_lock:
		res = latestMBError
	return res
	

def initModbus():
	global uart1
	with extension_wdt_lock:
		uart1=UART(1)
		uart1.init(9600, bits=8, parity=None, stop=1, timeout_chars=50, pins=('P3','P4'))


def deinitModbus():
	global uart1
	with extension_wdt_lock:
		uart1.deinit()

def readRegisters(mbAddress,devicetype,startRegister, numRegisters):
	res= None
	with extension_wdt_lock:
		if devicetype == M11:
			res= _reqModbusData(mbAddress, READ_REGS, startRegister, numRegisters,M11)
		else:
			res= _reqModbusData(mbAddress, READ_REGS, startRegister, numRegisters,C11)
	
	databuf = bytearray(numRegisters*2)
	if res == MB_SUCCESS:
		databuf = rxBuf[3:numRegisters*2+3]
	return databuf


def writeRegisters(mbAddress,devicetype,startRegister, numRegisters, newvalues):
	res= None
	with extension_wdt_lock:
		if devicetype == M11:
			res= _setModbusData(mbAddress, startRegister, numRegisters, newvalues, M11)
		else:
			res= _setModbusData(mbAddress, startRegister, numRegisters, newvalues, C11)
	return res

def readPilot(mbAddress,devicetype,CT=1):
	res= None
	with extension_wdt_lock:
		if devicetype == M11:
			res= _readM11(mbAddress,CT) # configured CT  as SPM91 (& M11) do not support CT yet 
		elif devicetype == C11_SPM91:
			res= _readSPM91(mbAddress,CT)
		elif devicetype == C11_SPM93:
			res= _readSPM93(mbAddress)
		elif devicetype == C11_SPM32:
			res= _readSPM33(mbAddress,1)
		elif devicetype == C11_SPM33:
			res= _readSPM33(mbAddress,5)
	return res

def getLoRaInstData(devicetype,typereq):
	res= None
	with extension_wdt_lock:
		if devicetype == M11 or devicetype == C11_SPM91:
			res=  _getLoRaInstDataM11()
		elif devicetype == C11_SPM93:
			res=  _getLoRaInstDataSPM93()
		elif devicetype == C11_SPM32:
			res=  _getLoRaInstDataSPM33(typereq)
		elif devicetype == C11_SPM33:
			res=  _getLoRaInstDataSPM33(typereq)  # same as SPM32

	return res

def getHHData():
	res = None
	with extension_wdt_lock:
		res=  _getHHDataSPM93()
	return res

def getLoRaHHData(phaseNumber=1):
	res = None
	with extension_wdt_lock:
		if phaseNumber == 1:
			res=  _getLoRaHHDataM11()
		else:
			res=  _getLoRaHHDataSPM93()

	return res

def getInstData(phaseNumber=1):
	with extension_wdt_lock:
		if phaseNumber == 1:
			return _getInstDataM11()
		else:
			return _getInstDataSPM93()