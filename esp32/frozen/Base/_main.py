#!/usr/bin/env python
"""main.py: PowerPilot python main POC"""
__version__="0.6.0"
__author__="Rodger Griffiths"
__copyright__="ElectroNet Ltd 2018"

def runMain():
    import pycom
    #first thing first
    pycom.wdt_on_boot(True)
    pycom.wdt_on_boot_timeout(60000*5) # 5 min

    import _thread
    import uos

    from globs import *
    from helpers import hardwareInit, buildPFAlarmMessage, initConfigurationVariable,getConfigurationVariable,getRandom
    import pycom
    import time
    import machine
    from machine import  Pin
    from loratask import loraTask
    from  modbus import readRegisters,initModbus,deinitModbus,readPilot,getLatestInstMeasurments,MB_SUCCESS
    from network import WLAN
    from machine import  WDT
    import binascii
    import network
    from machine import  WDT
    import gc

    import _thread
    from queuewrapper import QueueWrapper

    print('** MAIN POC ' + __version__ +  ' By ' + __author__ + ' Copyright '  + __copyright__ + ' **')
    print(str(uos.uname()))

    DEFAULT_STACK_SIZE=5000

    _thread.stack_size(DEFAULT_STACK_SIZE)

    AS=0
    try:
        from deviceid import antenna_setting
        AS=antenna_setting
    except:
        print('antenna_setting internal')
    else:
        print('antenna_setting is'  +str (AS))

    initConfigurationVariable(NVS_ANTENNA_CONFIGURATION,AS)
    antenna = getConfigurationVariable(NVS_ANTENNA_CONFIGURATION)
    # init the Hardware ( mainly the LoRa switch) and get the running mode, PF or NORMAL
    mode = hardwareInit(antenna) 
    pycom.nvs_set(POWER_FAIL_STATE, mode)
    if mode == MODE_POWER_FAIL :
        LED_OUT = Pin(LED_OUT_PIN)
        LED_OUT.value(1)       
        timeOfFault = pycom.nvs_get(POWER_FAIL_TS) 
        alarmLoraMessageToSend = buildPFAlarmMessage(timeOfFault)
        lora_queue_immediate_semaphore.acquire(0)
        LoraQueueImmediate.put(alarmLoraMessageToSend)
        _thread.start_new_thread(loraTask,())  # we just start the lora Task, send the PF alarm and go back to sleep
        if lora_stop_flag.locked():
            lora_stop_flag.release()
        
        time.sleep_ms(WAIT_TIME_BEFORE_SLEEP*10) # time for the LoRA MAC layer to send that message
        rand=int(getRandom() * 20) # 0-20 value
        #machine.deepsleep((WAKEUP_DELAY+rand)*1000) # GO TO SLEEP NOW AND PRESERVE POWER        

    from wdttask import wdtTask
    from wifitask import wifiTask
    from loopimpedancetask import loopImpedanceTask
    from modbustask import modbusTask
    from maintask import mainTask
    from eventtask import eventTask
    from computingtask import computingTask

    FCTMode=False # if true this mode is used in the factory during FCT 
    SERIAL_NUMBER_REGISTER=48888  # only works for M11 and C11 with SPM93
    MIN_SERIAL_NUMBER_VAL=10000000
    MAX_SERIAL_NUMBER_VAL=99999999
    # Read config variables
    CT=20 # default is 20 for M11 (only apply to M11)
    try:
        from deviceid import ctRatio
        CT=ctRatio
    except:
        pass

    print('M11 SW Ct Ratio is'  +str (CT))

    RO=int(getRandom() * 300) # 0-300 value
    try:
        from deviceid import radio_offset
        RO=radio_offset
    except:
        print('Radio Offset not defined, using random ' + str(RO))
    else:
        print('Radio Offset is'  +str (RO))

    DT=0 # assume M11 
    try:
        from deviceid import deviceType
        DT=deviceType
    except :
        print('Device type is not defined, using M11')
        FCTMode = True  # no config means FCT phase 
    else:
        print('Device type is'  +str (DT))


    if FCTMode == True:
        print('FCT mode')
        # read the S/N from the meter
        initModbus()
        serial_number=0
        dev_type=0
        MB_ADD=22
        MODEL_M11="00" #M11
        MODEL_M11E_20="01" #M11E-20
        Model=MODEL_M11
        # ONLY SUPPORT m11 AT PRESENT
        try:
            raw_serial_number=readRegisters(MB_ADD,dev_type,SERIAL_NUMBER_REGISTER,2)
            # convert to Uint32
            serial_number = int(raw_serial_number[2] << 8 | raw_serial_number[3] | raw_serial_number[0] << 24 | raw_serial_number[1] << 16)
        except:
            print("Calibration mode")
            serial_number=0

        print('S/N:' + str(serial_number))
        uid=str(binascii.hexlify(machine.unique_id()))
        #validate
        Ssid="TESTPP"
        if serial_number >= MIN_SERIAL_NUMBER_VAL and serial_number <= MAX_SERIAL_NUMBER_VAL:
            # calibration successfull use the SN as SSID 
            # 
            #  Read current and voltage M11 (5A - 230V) if successfull append S if not append F
            Result="F"
            try:
                res = readPilot(MB_ADD,dev_type,20)  # M11, address 22 with  ct is 20
                if res != MB_SUCCESS:
                    res = readPilot(MB_ADD,dev_type,20)  # M11, address 22 with  ct is 20
                    if res != MB_SUCCESS:
                        res = readPilot(MB_ADD,dev_type,20)  # M11, address 22 with  ct is 20

                data=getLatestInstMeasurments()
                V = float(data[INST_VOLTAGE])
                I = float(data[INST_CURRENT])
                print("Voltage:" + str(V) + "V - Current" + str(I) + "A")            
                if (V >= 228) and (V <= 232) and (I >= 4.8) and (I <= 5.2):
                    Result="S"
                    Model=MODEL_M11E_20 # if this is successful then it means we have CT ratio set up
            except:
                pass

            if dev_type == 0:
                DEV="M"
            else:
                DEV="C"
            Ssid =DEV+"{0:0=8d}".format(serial_number)+"-{0:0=3d}".format(VERSION_SOFTWARE)+uid[2:14].upper()+Result+Model
            # M19000001-022000000000000S03
            # TSSSSSSSSFFFMMMMMMMMMMMMRII  
            
        else:
            #possible calibration in progress 
            # display serial port must be high impedance:       
            deinitModbus()
            p3 = Pin('P3', mode=Pin.IN, pull=None)
            p4 = Pin('P4', mode=Pin.IN, pull=None)
            wdt = WDT(timeout=60000*5)  # 7 min
            print("Released display lines and waiting")
            while True: #forever loop there
                wdt.feed()
                time.sleep(10)      
        

        print('SSID:' + Ssid)
        wlan=WLAN() 
        wlan.init(mode=WLAN.AP, ssid=Ssid, auth=(WLAN.WPA2,'powerpilot'), channel=8, antenna=WLAN.INT_ANT, hidden=False)

        server = network.Server()
        server.deinit() # disable the server
        # enable the server again with new settings
        server.init(login=('factory', 'pilot'), timeout=120)

    else: # FCTMode == False

        # Init the configurable parameters in NVS if needed (first time)
        initConfigurationVariable(NVS_RADIO_DELAY,RO)
        initConfigurationVariable(NVS_HV_ALARM,VOLTAGE_EXCURSION_6_HIGH_THRESOLD)
        initConfigurationVariable(NVS_LV_ALARM,VOLTAGE_EXCURSION_6_LOW_THRESOLD)
        initConfigurationVariable(NVS_VHV_ALARM,VOLTAGE_EXCURSION_10_HIGH_THRESOLD)
        initConfigurationVariable(NVS_VLV_ALARM,VOLTAGE_EXCURSION_10_LOW_THRESOLD)
        initConfigurationVariable(NVS_CT_RATIO,CT)
        initConfigurationVariable(NVS_DEVICE_TYPE,DT)
        initConfigurationVariable(NVS_INST_DATA_FREQ,10)

        time_set_flag.acquire(0) #time need to be set (released = TIME is set)

        # NORMAL MODE, start all tasks
        _thread.start_new_thread(modbusTask,())
        gc.mem_free()
        _thread.start_new_thread(loopImpedanceTask,())
        _thread.start_new_thread(eventTask,())
        _thread.start_new_thread(wdtTask,())
        gc.mem_free()
        _thread.start_new_thread(computingTask,())
        #_thread.stack_size(DEFAULT_STACK_SIZE )
        _thread.start_new_thread(loraTask,())
        _thread.start_new_thread(wifiTask,())
        gc.mem_free()
        _thread.start_new_thread(mainTask,())

        time.sleep(1)

        if mb_stop_flag.locked():
            mb_stop_flag.release()
        
        if compute_stop_flag.locked():
            compute_stop_flag.release()
        
        if event_stop_flag.locked():
            event_stop_flag.release()

        if li_stop_flag.locked():
            li_stop_flag.release()

        if wifi_stop_flag.locked():
            wifi_stop_flag.release()

        if lora_stop_flag.locked():
            lora_stop_flag.release()
        
        if main_stop_flag.locked():
            main_stop_flag.release()

#runMain()
