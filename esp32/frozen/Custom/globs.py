#!/usr/bin/env python
"""globs.py: PowerPilot python global POC"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

from micropython import const
import _thread
from queuewrapper import QueueWrapper

#const
FORCEDRETRY = const(2)
MEMLOWTHREASHOLD=const(1000) # not used for now
MAXFAILEDACKMESSAGEBEFORERESET=const(10)
LORAMESSAGETIMEOUT = const(5*60) #5min timeout (base to acked)
MAXPERIODLORAMESSAGE = const(2*60)  # do not send lora message more than every 2 min
RESET_COUNTER_SECONDS = const(300)
WIFI_STA_RETRY_SECONDS = const(300)
MB_FAIL = const(0)
FAIL = "FAIL"
SUCCESS = "SUCCESS"
INSTDATA = False
HHDATA = True
MAXFAILEDAMODBUSREAD=const(100)
M11='M11'
C11='C11'
POWER_FAIL_PIN = 'P16'
LED_OUT_PIN = 'P11'
LORA_EXT_ANTENNA= 1
LORA_INT_ANTENNA= 0
VOLTAGE_EXCURSION_10_HIGH_THRESOLD=253000
VOLTAGE_EXCURSION_10_LOW_THRESOLD=207000
VOLTAGE_EXCURSION_6_HIGH_THRESOLD=243800
VOLTAGE_EXCURSION_6_LOW_THRESOLD=216200
VOLTAGE_EXCURSION_2_LOW_THRESOLD=225400
P1TIMEOUT= 10*60 * 0.15 # 10 min  MIN
P2TIMEOUT= 8*60 * 0.15# 8 min  MIN
P3TIMEOUT= 6*60 * 0.15# 6 min  MIN
P4TIMEOUT= 4*60  * 0.2# 4 min  MIN
P5TIMEOUT= 2*60  * 0.2# 2 min  MIN

#. (253 V to 216.2 V)
#  (243.8 V to 225.4 V)

UPGRADE_URL="https://powerpilotupgrade.azurewebsites.net/api/latest"

# queue indexes meaning
LENGTH = 2
TIMESTAMP = 1
MESSAGE = 0

LoopImpedanceValue=0

BINARY_PROTOCOL_HEADER = 7 # msg type (1B) + Header B1 + Header B2 + timestamp (4B)
POWER_FAIL_STATE = "powerfail"
NVS_RADIO_DELAY = "radiodelay"
NVS_HV_ALARM = "HVAlarm"
NVS_LV_ALARM = "LVAlarm"
NVS_VHV_ALARM = "VHVAlarm"
NVS_VLV_ALARM = "VLVAlarm"
NVS_CT_RATIO = "ctRatio"
NVS_DEVICE_TYPE = "deviceType"
NVS_ANTENNA_CONFIGURATION = "antenna"
NVS_INST_DATA_FREQ = "InstDataFreq"
NVS_LORA_JOIN_METHOD = "Lorajoinmethod"

POWER_FAIL_TS = "PFTS"

#define the semaphores
main_wdt_lock = _thread.allocate_lock()
main_stop_flag = _thread.allocate_lock()
lora_wdt_lock = _thread.allocate_lock()
lora_stop_flag = _thread.allocate_lock()
wifi_wdt_lock = _thread.allocate_lock()
wifi_stop_flag = _thread.allocate_lock()
li_wdt_lock = _thread.allocate_lock()
li_stop_flag = _thread.allocate_lock()
mb_wdt_lock = _thread.allocate_lock()
mb_stop_flag = _thread.allocate_lock()
event_wdt_lock = _thread.allocate_lock()
event_stop_flag = _thread.allocate_lock()
compute_wdt_lock = _thread.allocate_lock()
compute_stop_flag = _thread.allocate_lock()

event_message_flag = _thread.allocate_lock()

power_fail_semaphore = _thread.allocate_lock()
lora_queue_immediate_semaphore = _thread.allocate_lock()

lora_rssi_read = _thread.allocate_lock()
li_latest_value = _thread.allocate_lock()
processed_value = _thread.allocate_lock()

time_set_flag = _thread.allocate_lock()  #  released = TIME is set

fota_wifi_release_sem  =_thread.allocate_lock()  # indicate a FOTA in progress

MODE_POWER_FAIL = 1
MODE_NORMAL = 0
WAKEUP_DELAY = 5
RNG_MAX = 16777215 # 24bit
PHASE_RED=0
PHASE_WHITE=1
PHASE_BLUE=2
PHASE_ALL=3
INST_VOLTAGE=0
INST_CURRENT=1
INST_ACTIVEPOWER=2
INST_REACTIVEPOWER=3

VOLTAGE_NORMAL = 0

MESSAGE_TYPE_INSTANTANEOUS_DATA = 't0st0'
MESSAGE_TYPE_METERING_DATA = 't0st1'
MESSAGE_TYPE_COMPUTED_DATA = 't0st3'
MESSAGE_TYPE_ALARM_LI = 't8st0'
MESSAGE_TYPE_ALARM_VOLTAGE = 't8st1'
MESSAGE_TYPE_ALARM_POWER_FAIL = 't8st2'
MESSAGE_TYPE_TIME_REQUEST = 't1st0'
MESSAGE_TYPE_D2C_GET_RESPONSE = 't5st0'
MESSAGE_TYPE_TIME_RESPONSE = 't4st1'
MESSAGE_TYPE_C2D_SET_REQUEST = 't3st1'
MESSAGE_TYPE_C2D_GET_REQUEST = 't3st0'
MESSAGE_TYPE_C2D_GET_MODBUS_REQUEST = 't7st0'
MESSAGE_TYPE_C2D_SET_MODBUS_REQUEST = 't7st1'
MESSAGE_TYPE_C2D_GET_MODBUS_RESPONSE = 't9st0'

GET_ID_HH_DATA = 0

MESSAGE_TYPE_INSTANTANEOUS_DATA_WIFI =   "PP001"
MESSAGE_TYPE_METERING_DATA_WIFI =   "HH01"
MESSAGE_TYPE = 0
MESSAGE_DATA = 1


VOLTAGE_NORMAL = 0
ALARM_HIGH = 1
ALARM_VERY_HIGH = 2
ALARM_LOW = 3
ALARM_VERY_LOW = 4

#define the LoRaWAN QOS queues
LoraQueueP1 = QueueWrapper(4) # Very Low Priority   (10 min data -  10 min min between messages @1.2s per message   0.2% duty cycle) offset = radio_offset
LoraQueueP2 = QueueWrapper(8) # Low  Priority  ( HH data  - 8 min min between messages @1.2s per message  -  => 0.25%)       offset = radio_offset * 4/5
LoraQueueP3 = QueueWrapper(6) # Low Priority (6 min min between messages  @1.2s per message  => 0.33% duty cycle)  offset = radio_offset * 3/5
LoraQueueP4 = QueueWrapper(4) # Low Priority  ( 4 min between messages@1.2s per message  => 0.5% duty cycle)    offset = radio_offset * 2/5
LoraQueueP5 = QueueWrapper(3) # High Priority (2 min between messages @1.2s per message  => 1% duty cycle)  offset = radio_offset * 1/5
LoraQueueImmediate = QueueWrapper(1) # No throttling

LoraQueueRx = QueueWrapper(5) # Queue for Messages Received from LoRaWAN
LiQueue = QueueWrapper(10) # Queue for Loop impedance values

WifiQueueTx = QueueWrapper(5) # Egress Wifi queue
WifiQueueRx = QueueWrapper(5)  # Ingress Wifi queue

MAX_HH_QUEUE_ITEM = 48
HHQueue = QueueWrapper(MAX_HH_QUEUE_ITEM,True)  # Half Hour queue day worth of storage (no delete )

C2D_PARAM_UPGRADE = "Upgrade"
C2D_PARAM_RELAY = "Relay"
C2D_PARAM_HHDATA = "HHData"
C2D_PARAM_INST_DATA = "InstData"
C2D_PARAM_PROCESSED_DATA = "ExtendedData"
C2D_PARAM_RESET = "Reset"
C2D_PARAM_RADIO_OFFSET = "RadioOffset"
C2D_PARAM_HV_ALARM = "HVAlarm"
C2D_PARAM_VHV_ALARM = "VHVAlarm"
C2D_PARAM_LV_ALARM= "LVAlarm"
C2D_PARAM_VLV_ALARM= "VLVAlarm"
C2D_PARAM_CT_RATIO= "CtRatio"
C2D_PARAM_DEV_TYPE= "devType"
C2D_PARAM_ANTENNA= "Antenna"
C2D_PARAM_INST_FREQ= "InstFreq"
C2D_PARAM_LOAD= "Load"

CD2_SET_PARAMETERS_LIST = [C2D_PARAM_UPGRADE,C2D_PARAM_RELAY,C2D_PARAM_RESET,C2D_PARAM_RADIO_OFFSET,C2D_PARAM_HV_ALARM,C2D_PARAM_LV_ALARM,C2D_PARAM_CT_RATIO,C2D_PARAM_DEV_TYPE,C2D_PARAM_ANTENNA,C2D_PARAM_INST_FREQ,C2D_PARAM_VHV_ALARM,C2D_PARAM_VLV_ALARM,C2D_PARAM_LOAD]
CD2_GET_PARAMETERS_LIST = [C2D_PARAM_HHDATA,C2D_PARAM_INST_DATA,C2D_PARAM_PROCESSED_DATA,C2D_PARAM_LOAD]



C2D_REQUEST=0
# SET
C2D_SET_REQ_ID=1
C2D_SET_REQ_VAL=2
#GET
C2D_GET_REQ_ID=1
C2D_GET_REQ_PARAM=2
WAIT_TIME_BEFORE_SLEEP=1500

#GET MODBUS
C2D_GET_MD_REQ_ADDRESS=1
C2D_GET_MD_REQ_PARAM=2


MESSAGE_DATA = 0
MESSAGE_LENGTH = 1

WAIT_TIME_BEFORE_SLEEP=2000
#define in deviceid
OTAA_JOIN = 0
ABP_JOIN=1

MAXINT16 = 32767
MININT16 = -32768
MAXUINT16 = 65535
MAXINT8 = 127
MININT8 = -128

#convention for computed variables
MAX_VOLTAGE=0
MIN_VOLTAGE=1
MAX_CURRRENT=2
PSOH=3
BALANCE=4
MAX_THDV=5
PSOH2=6

# convention used in deviceid.py
M11=0
C11_SPM91=1
C11_SPM93=2
C11_SPM32=3
C11_SPM33=4

#load id
LOAD_NUMBER=0
LOAD_ADDRESS=1
LOAD_VALUE=2   
LOAD_SPARE=3

# 0x00 v0.2.3
# 0x01 v0.2.4
# 0x02 v0.2.5
# 0x03 v0.2.6
# 0x04 v0.3.0
# 0x05 v0.3.1
# 0x06 v0.3.2
# 0x07 v0.3.3
# 0x08 v0.3.4
# 0x09 v0.4.0
# 0x0A v0.4.1
# 0x0B v0.4.2
# 0x0C v0.4.3
# 0x0D v0.6.0
VERSION_SOFTWARE=0x0D
VERSION="0.6.0"



