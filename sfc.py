#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on January 2021
Driver to control Sofar ME3000 via python3
@author: Iqas
"""

import minimalmodbus as mm
from time import sleep
import paho.mqtt.client as mqtt
from struct import *
import configparser as cp
import os.path
import schedule
import time, datetime
import functools
import os
import logging
from systemd import journal
from urllib.request import Request, urlopen
import json
from datetime import datetime, date, timedelta

def _update_logger():
    if (config.has_option('main','log_error')):
        if (config.get('main','log_error') == 'debug'):
            logger.setLevel(logging.DEBUG)
        elif (config.get('main','log_error') == 'notset'):
            logger.setLevel(logging.NOTSET)
        elif (config.get('main','log_error') == 'info'):
            logger.setLevel(logging.INFO)
        elif (config.get('main','log_error') == 'warning'):
            logger.setLevel(logging.WARNING)
        elif (config.get('main','log_error') == 'error'):
            logger.setLevel(logging.ERROR)
        elif (config.get('main','log_error') == 'critical'):
            logger.setLevel(logging.CRITICAL)
    else:
        logger.setLevel(logging.INFO)



class configControl():

    def __init__(self, cfg_file='/etc/sfc.ini'):
        self.filename = cfg_file
        self._cached_stamp = os.stat(self.filename).st_mtime
        
    def testCfg(self):
        stamp = os.stat(self.filename).st_mtime
        ret_value = (stamp == self._cached_stamp)
        self._cached_stamp = stamp
        return (ret_value)

class me3000ModBus(mm.Instrument ):
    """Instrument class for SOFAR ME3000 inverter process controller.
    Args:
        * portname (str): port name
        * slaveaddress (int): slave address in the range 1 to 247
    """
    arrayStatus = [] ; ## array for store the status of running control
    imode = 'auto'
    charge = 0
    batteryStatusModes = ['Normal','Battery Low','Battery Full']
    inverterstatemodes = ['Waiting','Test Charging','Charging','Check Discharging','Discharging','EPS mode','Fault State','Permanent Fault']
    battery = 0
    batteryStatus = 0
    BatCurrent = 0
    BatCapacity = 0
    BatVoltage = 0
    InvTemp = 0
    BatTemp = 0
    SolarPower = 0
    FeedPower = 0
    InvState = 0
    LoadPower = 0
    EPSVoltage = 0
    EPSPower = 0
    DayLoad = 0
    DayBuyFeed = 0
    DaySellFeed = 0
    DayFV = 0
    IOPower = 0
    Charge = 1500
    Discharge = 1500
    TrueFeedPower = 0
    dynamic_charge = False
    _lastpower = 0
    com_busy = False
    initmode = True
    max_soc = 100 
    lastDayBuyFeed = None
    lastDaySellFeed = None
    lastDayFV = None
    lastTimeGetData = None

    def __init__(self, portname, slaveaddress):
        
        mm.Instrument.__init__(self, portname, slaveaddress)    
        # Atributes
        if (config.has_option('main','level_console_error') and config.get('main','level_console_error')=='debug'):
            self.debug=True
        else:
            self.debug=False
        self.serial.baudrate = 9600
        self.bytesize = 8
        self.serial.parity = mm.serial.PARITY_NONE
        self.serial.stopbits = 1
        self.serial.timeout = 1
        self.mode = mm.MODE_RTU

    def _test_busy(self):
        counter = 4 ## Wait four seconds for busy port
        while (self.com_busy):
                if (counter == 0):
                    self.com_busy = False
                    break
                logger.warning('Port busy, waiting.........')
                sleep (1)
                counter -= 1

        
    def _getData(self, addr, dec, signed=False): ## Function Code 0x03
        try:
            if (config.has_option('main','level_console_error') and (config.get('main','level_console_error') == 'debug')):
                self.debug=True
            else:
                self.debug=False
            self._test_busy()
            self.com_busy = True
            data=self.read_register(addr,dec,3,signed)
            self.com_busy = False
            if data!=None:
                return data
        except IOError:
            logger.warning("Failed to read from instrument")

    
    def _setData(self, mode, value, func): ## Function Code 0x44
        try:
            if (config.has_option('main','level_console_error') and (config.get('main','level_console_error') == 'debug')):
                self.debug=True
            else:
                self.debug=False
            self._test_busy                
            self.com_busy = True
            data = mode + mm._num_to_twobyte_string(value)
            resp=repr(self._perform_command(func, data))
            self.com_busy = False

            resp=resp.replace('\\x','')
            hibyte=int(resp[3:5])
            lobyte=int(resp[5:7])
            print ("---------------------------------------")
            
            ## TODO Crear mqtt con estados del inversor
            
            if(lobyte&1 == 0):
                    logger.info ("Code Accept")
            if(lobyte&1 >= 1):
                    if(hibyte&1 >= 1):
                        logger.info("Charge enabled")
                    if(hibyte&2 >= 1):
                        logger.info("Discharge enabled")
                    if(hibyte&4 >= 1):
                        logger.info("The battery is full")
                        Sofar.battery=2 
                    elif(hibyte&8 >= 1):
                        logger.info("The battery is flat")
                        Sofar.battery=1
                    else:
                        Sofar.battery=0 
            if(lobyte&1 >= 1):
                    logger.info ("Invalid Mode,check inverter work-mode")
            if(lobyte&2 >= 1):
                    logger.info( "Busy inverter")
            if(lobyte&4 >= 1):
                    logger.info("Invalid Data")
            
            if(hibyte&1 >= 1):
                    logger.info("Charge enabled")
            if(hibyte&2 >= 1):
                    logger.info("Discharge enabled")
            if(hibyte&4 >= 1):
                    logger.info("The battery is full")
            if(hibyte&8 >= 1):
                    logger.info("The battery is flat")
    
            
        except IOError:
            logger.error("Failed to read from instrument")
            
    ## SET FUNCTIONS ##############################0x42 - 66d
    
    def set_Discharge(self, value):
    # Mode discharge.
        return self._setData("\x01\x01",value,66)

    def set_Standby(self, value):
    # Mode Standby.
        return self._setData("\x01\x00",value,66)

    def set_Charge(self, value):
    # Mode charge.
        return self._setData("\x01\x02",value,66)

    def set_Auto(self, value):
    # Mode Auto. Positve->discharge Negative->charge 0x5555->0 value
        return self._setData("\x01\x03",value,66)
    
    ## SET FUNCTIONS ##############################0x41 - 65d
    def set_EpsOn(self):
    # EPS On
        return self._setData("\x00\x00","\x00\x01\x00\x55",65)

    def set_EpsOff(self):
    # EPS Off
        return self._setData("\x00\x00","\x00\x01\x00\x55",65)

    ## GET FUNCTIONS ##############################
    
    def get_DayLoad(self):
    # Return the value of total load compsumption.
        return self._getData(0x021B,0,False)
    
    def get_DayBuyFeed(self):
    # Return the value of total day feed buy.
        return self._getData(0x021A,0,False)
        
    def get_DaySellFeed(self):
    # Return the value of total day feed sell.
        return self._getData(0x0219,0,False)
            
    def get_DayFV(self):
    # Return the value of total day FV.
        return self._getData(0x0218,0,False)
    
    def get_BatCapacity(self):
    # Return the value of battery capacity.
        return self._getData(0x0210,0,False)
    
    def get_BatVoltage(self):
    # Return the value of battery voltage.
        return self._getData(0x020E,2,False)

    def get_BatCurrent(self):
    # Return the value of battery current.
        return self._getData(0x020F,2,True) # Signed

    def get_InvTemp(self):
    # Return the value of inverter temperature.
        return self._getData(0x0238,0,False)

    def get_BatTemp(self):
    # Return the value of battery temperature.
        return self._getData(0x0211,0,False)

    def get_SolarPower(self):
    # Return the value of generation power.
        return self._getData(0x0215,0,False)

    def get_FeedPower(self):
    # Return the value of feed power. 
        try:
            return (self._getData(0x0212,0,True)*10)
        except:
            logger.error ('Get data fail')

    def get_InvState(self):
    # Return the value of inverter state.
        return self._getData(0x0200,0,False)

    def get_LoadPower(self):
    # Return the value of load power.
        return self._getData(0x0213,0,False)

    def get_IOPower(self):
    # Return the value of Iput-Output  power.
        return self._getData(0x0214,0,True)

    def get_EPSVoltage(self):
    # Return the value of EPS voltage.
        return self._getData(0x0216,1,False)
        
    def get_EPSPower(self):
    # Return the value of EPS power.
        return self._getData(0x0217,0,False)
    
    ## End inverter class

def with_logging(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        logger.info('Running job "%s"' % func.__name__)
        result = func(*args, **kwargs)
        logger.info('Job "%s" completed' % func.__name__)
        return result
    return wrapper

def mqttRun():
    logger.info('Process mqqtt running')

    if (config.has_option('mqtt-pub','BatCapacity')):
        client.publish(config.get('mqtt-pub','BatCapacity'),str(Sofar.BatCapacity))
    if (config.has_option('mqtt-pub','BatVoltage')):
        client.publish(config.get('mqtt-pub','BatVoltage'),str(Sofar.BatVoltage))
    if (config.has_option('mqtt-pub','BatCurrent')):
        client.publish(config.get('mqtt-pub','BatCurrent'),str(Sofar.BatCurrent))
    if (config.has_option('mqtt-pub','BatTemperature')):
        client.publish(config.get('mqtt-pub','BatTemperature'),str(Sofar.BatTemp))
    if (config.has_option('mqtt-pub','InverterTemperature')):
        client.publish(config.get('mqtt-pub','InverterTemperature'),str(Sofar.InvTemp))
    if (config.has_option('mqtt-pub','SolarPower')):
        client.publish(config.get('mqtt-pub','SolarPower'),str(Sofar.SolarPower))
    if (config.has_option('mqtt-pub','FeedPower')):
        client.publish(config.get('mqtt-pub','FeedPower'),str(Sofar.FeedPower*-1))
    if (config.has_option('mqtt-pub','InverterState')):
        client.publish(config.get('mqtt-pub','InverterState'),str(Sofar.InvState))
    if (config.has_option('mqtt-pub','LoadPower')):
        client.publish(config.get('mqtt-pub','LoadPower'),str(Sofar.LoadPower))
    if (config.has_option('mqtt-pub','EPSPower')):
        client.publish(config.get('mqtt-pub','EPSPower'),str(Sofar.EPSPower))
    if (config.has_option('mqtt-pub','EPSVoltage')):
        client.publish(config.get('mqtt-pub','EPSVoltage'),str(Sofar.EPSVoltage))
    if (config.has_option('mqtt-pub','DayLoad')):
        client.publish(config.get('mqtt-pub','DayLoad'),str(Sofar.DayLoad))
    if (config.has_option('mqtt-pub','DayBuyFeed')):
        client.publish(config.get('mqtt-pub','DayBuyFeed'),str(Sofar.DayBuyFeed))
    if (config.has_option('mqtt-pub','DaySellFeed')):
        client.publish(config.get('mqtt-pub','DaySellFeed'),str(Sofar.DaySellFeed))
    if (config.has_option('mqtt-pub','DayFV')):
        client.publish(config.get('mqtt-pub','DayFV'),str(Sofar.DayFV))
    if (config.has_option('mqtt-pub','IOPower')):
        client.publish(config.get('mqtt-pub','IOPower'),str(Sofar.IOPower))
    ## mqtt domoticz
    if (config.has_option('mqtt-domoticz','BatCapacity')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','BatCapacity'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.BatCapacity)+'" }')
    if (config.has_option('mqtt-domoticz','BatVoltage')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','BatVoltage'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.BatVoltage)+'" }')
    if (config.has_option('mqtt-domoticz','BatCurrent')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','BatCurrent'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.BatCurrent)+'" }')
    if (config.has_option('mqtt-domoticz','BatTemperature')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','BatTemperature'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.BatTemp)+'" }')
    if (config.has_option('mqtt-domoticz','InverterTemperature')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','InverterTemperature'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.InvTemp)+'" }')
    if (config.has_option('mqtt-domoticz','SolarPower')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','SolarPower'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.SolarPower)+'" }')
    if (config.has_option('mqtt-domoticz','FeedPower')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','FeedPower'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.FeedPower*-1)+'" }')
    if (config.has_option('mqtt-domoticz','InverterState')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','InverterState'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.InvState)+'" }')
    if (config.has_option('mqtt-domoticz','LoadPower')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','LoadPower'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.LoadPower)+'" }')
    if (config.has_option('mqtt-domoticz','EPSPower')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','EPSPower'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.EPSPower)+'" }')
    if (config.has_option('mqtt-domoticz','EPSVoltage')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','EPSVoltage'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.EPSVoltage)+'" }')
    if (config.has_option('mqtt-domoticz','DayLoad')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','DayLoad'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.DayLoad)+'" }')
    if (config.has_option('mqtt-domoticz','DayBuyFeed')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','DayBuyFeed'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.DayBuyFeed)+'" }')
    if (config.has_option('mqtt-domoticz','DaySellFeed')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','DaySellFeed'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.DaySellFeed)+'" }')
    if (config.has_option('mqtt-domoticz','DayFV')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','DayFV'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.DayFV)+'" }')
    if (config.has_option('mqtt-domoticz','IOPower')):
        client.publish("domoticz/in",'{ "idx" : '+str(config.get('mqtt-domoticz','IOPower'))+', "nvalue" : 0, "svalue" : "'+str(Sofar.IOPower)+'" }')
    
def dynamicCharge():
    #Control for Dynamic charge
    if ((Sofar.dynamic_charge and Sofar.imode == 'charge') and config.has_option('main','feedpower')): 
        try:
            Sofar.FeedPower = Sofar.get_FeedPower()
            if (config.has_option('mqtt-get','truefeedpower')):
                available_power = config.getint('main', 'feedpower') - int(Sofar.TrueFeedPower) + Sofar._lastpower 
            else:
                available_power = config.getint('main', 'feedpower') + Sofar.FeedPower + Sofar._lastpower
            logger.info ('Available power: '+str(available_power))

            if ((Sofar._lastpower != available_power) and (Sofar.FeedPower is not None)):
                if (available_power < 0):
                   available_power = 0
                elif (available_power >= 3000):
                    available_power = 3000
                if (available_power < 10):
                    available_power = 0
                if (Sofar.charge <  available_power): 
                    available_power = Sofar.charge
                logger.info ( 'Dynamic charge: LastPower: ' + str(Sofar._lastpower)+' Available: ' + str(available_power))
                # test
                if (str(Sofar._lastpower) == str(available_power)):
                    logger.info('Charge power static')
                else:
                    Sofar.set_Charge(available_power)
                    Sofar._lastpower = available_power
                    logger.info ('Updating the charge power to: '+str(available_power))        
        except Exception as e:
            logger.error (str(e))


#@with_logging
def getDataRun():
    try:
        logger.debug('Get data running')
        if (config.has_option('main','level_console_error') and (config.get('main','level_console_error') == 'debug')):
            Sofar.debug=True
        else:
            Sofar.debug=False
        Sofar.InvState = Sofar.get_InvState()
        Sofar.BatVoltage = Sofar.get_BatVoltage()
    
        if (config.has_option('mqtt-pub','batcurrent')) or (config.has_option('mqtt-domoticz','batcurrent')):
            Sofar.BatCurrent = Sofar.get_BatCurrent()
        if (config.has_option('mqtt-pub','batcapacity')) or (config.has_option('mqtt-domoticz','batcapacity')):    
            Sofar.BatCapacity = Sofar.get_BatCapacity()
        if (config.has_option('mqtt-pub','invertertemperature')) or (config.has_option('mqtt-domoticz','invertertemperature')):
            Sofar.InvTemp = Sofar.get_InvTemp()
        if (config.has_option('mqtt-pub','battemperature')) or (config.has_option('mqtt-domoticz','battemperature')):
            Sofar.BatTemp = Sofar.get_BatTemp()
        if (config.has_option('mqtt-pub','solarpower')) or (config.has_option('mqtt-domoticz','solarpower')):
            Sofar.SolarPower = Sofar.get_SolarPower()
        Sofar.FeedPower = Sofar.get_FeedPower() 
        if (config.has_option('mqtt-pub','loadpower')) or (config.has_option('mqtt-domoticz','loadpower')):
            Sofar.LoadPower = Sofar.get_LoadPower()
        if (config.has_option('mqtt-pub','epsvoltage')) or (config.has_option('mqtt-domoticz','epsvoltage')):
            Sofar.EPSVoltage = Sofar.get_EPSVoltage()
        if (config.has_option('mqtt-pub','epspower')) or (config.has_option('mqtt-domoticz','epspower')):
            Sofar.EPSPower = Sofar.get_EPSPower()
        if (config.has_option('mqtt-pub','dayload')) or (config.has_option('mqtt-domoticz','dayload')):
            Sofar.DayLoad = Sofar.get_DayLoad()
        Sofar.DayBuyFeed = Sofar.get_DayBuyFeed()
        Sofar.DaySellFeed = Sofar.get_DaySellFeed()
        Sofar.DayFV = Sofar.get_DayFV()
        Sofar.IOPower = Sofar.get_IOPower()

        if (Sofar.DayFV is not None):
            Sofar.DayFV = Sofar.DayFV*10
        if (Sofar.IOPower is not None):
            Sofar.IOPower = Sofar.IOPower*10
        if (Sofar.SolarPower is not None):
            Sofar.SolarPower = Sofar.SolarPower*10
        if (Sofar.DayLoad is not None):
            Sofar.DayLoad = Sofar.DayLoad*10
        if (Sofar.DayBuyFeed is not None):
            Sofar.DayBuyFeed = Sofar.DayBuyFeed*10
        if (Sofar.DaySellFeed is not None):
            Sofar.DaySellFeed = Sofar.DaySellFeed*10
        if (Sofar.LoadPower is not None):
            Sofar.LoadPower = Sofar.LoadPower*10

        logger.info('STATE: ' + Sofar.inverterstatemodes[Sofar.InvState] + ' Bat:' + str(Sofar.BatVoltage) + 'v Power:' +str(Sofar.IOPower) + 'w' )

    except Exception as e:
        logger.error (str(e))


#@with_logging
def controlRun():
    logger.debug('Running control: '+time.strftime("%d-%m-%yy %H:%M"))
    if ( (int(Sofar.BatCapacity) >=  Sofar.max_soc) and Sofar.imode == 'charge' ):
        logger.info ('Max SOC detected, stoping charge')
        Sofar.set_Charge(1)

    if (not cfgCtrl.testCfg()):
        logger.info ('Configuration has been changed, reloading values')
        config.read('/etc/sfc.ini')
        _update_logger()
        ## Test global dynamic charge
        if (config.has_option('main','dynamic_charge')):
            Sofar.dynamic_charge = config.getboolean('main', 'dynamic_charge')
    
    if (Sofar.BatVoltage is not None):
        if (Sofar.BatVoltage >= config.getfloat('battery', 'max_voltage')  and Sofar.InvState == 2):
            logger.error('Max battery voltage detected go to standby')
            Sofar.set_Standby(0x5555)
            Sofar.imode = 'standby'
        if (Sofar.BatVoltage <= config.getfloat('battery', 'minimal_voltage')  and (Sofar.InvState == 4) ):
            logger.error('Min battery voltage detected go to standby')
            Sofar.set_Standby(0x5555)
            Sofar.imode = 'standby'
    
    # End hour (Stop Mode)            
    cont=0
    option=''
    charge = 0
    discharge = 0
    while True:
        cont += 1
        print("Processing Control{0} end configuration".format(str(cont)))
        if ( config.has_section('control'+str(cont)) and config.getboolean('control'+str(cont), 'enable') == True):
            #End hour
           now=datetime.now()
           if( (not config.has_option('control'+str(cont),'weekday')) or (config.has_option('control'+str(cont),'weekday') and (config.get('control'+str(cont), 'weekday').count(now.strftime("%u")) > 0) )):
                # Month and day                
                datenow = date.today()
                if (config.has_option('control'+str(cont),'start_date') and config.has_option('control'+str(cont),'end_date')):
                    init=datetime.strptime(str(datenow.year)+'-'+config.get('control'+str(cont), 'start_date'),'%Y-%m-%d').date()
                    end = datetime.strptime(str(datenow.year)+'-'+config.get('control'+str(cont), 'end_date'),'%Y-%m-%d').date()
                else:
                    init=datenow
                    end=datenow
                    logger.warning("There is no date control activated in control"+str(cont));
                if (datenow >= init and datenow <= end):
                    if (config.get('control'+str(cont), 'end_hour') == time.strftime("%H:%M") ):
                        logger.debug("Start Ending option :"+config.get('control'+str(cont), 'mode'));
                        if (len(Sofar.arrayStatus) > 1 ):
                            Sofar.arrayStatus.pop()
                            Sofar.imode = Sofar.arrayStatus[0]
                            logger.info('Back to the {0} mode'.format(option))
                        else:
                            Sofar.imode = 'auto'
                            logger.info('Back to the default auto mode')
                        option = Sofar.imode
                        if (config.has_option('control'+str(cont),'charge')):
                            charge = config.getint('control'+str(cont), 'charge')
                        if (config.has_option('control'+str(cont),'discharge')):
                            discharge = config.getint('control'+str(cont), 'discharge')
        else:
            break
    
    # Init hour (start mode)
    cont=0
    while True:
        cont += 1
        if ( config.has_section('control'+str(cont)) and config.getboolean('control'+str(cont), 'enable') == True):
            #Week 
            now=datetime.now()
            if( (not config.has_option('control'+str(cont),'weekday')) or (config.has_option('control'+str(cont),'weekday') and (config.get('control'+str(cont), 'weekday').count(now.strftime("%u")) > 0) )):
                # Month and day
                datenow = date.today()
                if (config.has_option('control'+str(cont),'start_date') and config.has_option('control'+str(cont),'end_date')):
                    init=datetime.strptime(str(datenow.year)+'-'+config.get('control'+str(cont), 'start_date'),'%Y-%m-%d').date()
                    end = datetime.strptime(str(datenow.year)+'-'+config.get('control'+str(cont), 'end_date'),'%Y-%m-%d').date()
                else:
                    init=datenow
                    end=datenow
                    logger.warning("There is no date control activated in control"+str(cont));
                if (datenow >= init and datenow <= end):
                    if (config.get('control'+str(cont), 'start_hour') == time.strftime("%H:%M")  and config.getboolean('control'+str(cont), 'enable') == True ):
                        Sofar.imode = config.get('control'+str(cont), 'mode')
                        logger.info("Proccesing start option :"+Sofar.imode);
                        if (config.has_option('control'+str(cont),'max_soc')):
                            Sofar.max_soc = config.getint('control'+str(cont),'max_soc')
                            logger.info("Max SOC charge set to {0}".format(str(Sofar.max_soc)))
                        else:
                            Sofar.max_soc = 100
                            logger.info("Max SOC charge set to {0}".format(str(Sofar.max_soc)))
                        if (config.has_option('control'+str(cont),'charge')):
                            Sofar.charge = config.getint('control'+str(cont),'charge')
                            charge = config.getint('control'+str(cont), 'charge')
                        if (config.has_option('control'+str(cont),'discharge')):
                            discharge = config.getint('control'+str(cont), 'discharge')
                    
                        if (config.has_option('control'+str(cont),'dynamic_charge')):
                            Sofar.dynamic_charge = config.getboolean('control'+str(cont), 'dynamic_charge')
                        if (config.has_option('control'+str(cont),'charge') and config.get('control'+str(cont), 'mode') == 'charge' and Sofar.dynamic_charge):
                            print('Dynamic mode is active. Charge value changed to: '+ str(Sofar.charge))
                            #auto, discharge, charge, standby, auto+
                        Sofar.arrayStatus.append(option)
                        option = Sofar.imode
        else:
            break


    if (len(option)>0):
        logger.info("Activating mode: " + Sofar.imode)
        if (Sofar.imode == 'auto'):
            Sofar.set_Auto(0x5555)
        elif (Sofar.imode == 'charge'):
            Sofar.set_Charge(charge)
        elif (Sofar.imode == 'discharge'):
            Sofar.set_Discharge(discharge)
        elif (Sofar.imode == 'standby'):
            Sofar.set_Standby(0x5555)
        elif  (Sofar.imode == 'auto+'):
            logger.info('Opción no implementada todavía')

## Mqtt Control    
def on_message(client, userdata, msg):
    logger.info("MQTT Received `{payload}` from `{topic}` topic".format(payload=msg.payload.decode(),topic=msg.topic ))
    payload=msg.payload.decode()
    topic=msg.topic
    
    if (config.has_option('mqtt-get','mode')):
        if (topic == config.get('mqtt-get','mode')):
            if (payload == 'auto'):
                Sofar.set_Auto(0x5555)
                Sofar.imode = 'auto'
                logger.info('Auto mode selected')
            elif (payload == 'charge'):
                Sofar.set_Charge(1500)
                Sofar.charge = 1500
                Sofar.imode = 'charge'
                logger.info('Charge mode selected ({watts} w)'.format(watts=1500))
            elif (payload == 'discharge'):
                Sofar.set_Discharge(1500)
                Sofar.imode = 'discharge'
                logger.info('Discharge mode selected ({watts} w)'.format(watts=1500))
            elif (payload == 'standby'):
                Sofar.set_Standby(0x5555)
                Sofar.imode = 'standby'
                logger.info('Standby mode selected')

    if (config.has_option('mqtt-get','truefeedpower')):
        if (topic == config.get('mqtt-get','truefeedpower')):
            Sofar.TrueFeedPower = payload
    if (config.has_option('mqtt-get','charge')):
        if (topic == config.get('mqtt-get','charge')):
            Sofar.set_Charge(int(payload))
            Sofar.imode = 'charge'
            Sofar.charge = int(payload)
            logger.info('Charge mode selected ({watts} w)'.format(watts=payload))
    if (config.has_option('mqtt-get','discharge')):
        if (topic == config.get('mqtt-get','discharge')):
            Sofar.set_Discharge(int(payload))
            Sofar.imode = 'discharge'
            logger.info('Discharge mode selected ({watts} w)'.format(watts=payload))
    if (config.has_option('mqtt-get','dynamic_charge')):
        if (topic == config.get('mqtt-get','dynamic_charge')):
            if (int(payload)==1):
                Sofar.dynamic_charge = True
                logger.info('Dinamic Charge activated')
            else:
                Sofar.dynamic_charge = False
                logger.info('Dinamic Charge deactivated')


def on_disconnect(client, userdata, flags, rc):
    """Called when disconnected from MQTT broker."""
    logger.warning("DISCONNECT")
    client.loop_stop()
    client.reconnect()
    client.loop_start()
    
def on_connect(client, userdata, flags, rc):
    logger.warning("CONNECT")
    #Subscribe
    logger.info("Mqtt client connect, subscribing topics")
    if (config.has_option('mqtt-get','mode')):
        client.subscribe(config.get('mqtt-get','mode')) # subscribe
    if (config.has_option('mqtt-get','charge')):
        client.subscribe(config.get('mqtt-get','charge'))
    if (config.has_option('mqtt-get','discharge')):
        client.subscribe(config.get('mqtt-get','discharge'))
    if (config.has_option('mqtt-get','dynamic_charge')):
        client.subscribe(config.get('mqtt-get','dynamic_charge'))
    if (config.has_option('mqtt-get','truefeedpower')):
        client.subscribe(config.get('mqtt-get','truefeedpower'))
    
##########
#  Init  #
##########
print('sfc v 1.0 running...')
print('For log run: journalctl -f -u sfc')
logger = logging.getLogger('SoFarCtrl')
logger.addHandler(journal.JournaldLogHandler())
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger.setLevel(logging.INFO)

config = cp.RawConfigParser()
if os.path.isfile('/etc/sfc.ini'):
    logger.info ("Reading config sfc.ini")
    config.read('/etc/sfc.ini')
    _update_logger()
else:
    logger.warning("Config not config found, creating new file")    
    config.add_section('main')
    config.set('main', 'feedpower', '4600') ## Max power from the feed
    config.set('main', 'port', '/dev/ttyUSB0') ## Port for Modbus to USB
    config.set('main', 'modbus_slave', '1') ## Address for the inverter modbus connection
    config.set('main', 'log_error', 'WARNING') ## Level of the journalctl error: CRITICAL, ERROR, WARNING, INFO, DEBUG, NOTSET
    config.set('main', 'max_soc', '90') ## Max soc to charge
    config.set('main', 'dynamic_charge', 'yes') ## If yes, the power charge is dynamically adjusted to the maximum grid power (feedpower value) Global value
    config.set('main', 'peak_shaving', '2000') ## Supplement grid power with batteries in the case of low power networks, value max w from the feed (Only in standby or charge mode)
    config.add_section('battery')
    config.set('battery', 'minimal_voltage', '48')
    config.set('battery', 'max_voltage', '57.4')
    config.add_section('control1')
    config.set('control1', 'enable', 'yes') ## Enable or disable
    config.set('control1', 'overconsumption', '1') ## Discharge in automatic mode if we exceed the maximum grid power
    config.set('control1', 'start_date', '01-01') ## MM-DD
    config.set('control1', 'end_date', '12-31') ## MM-DD 
    config.set('control1', 'start_hour', '00:00')
    config.set('control1', 'end_hour', '23:59')
    config.set('control1', 'weekday', '1,2,3,4,5,6,7') #1 is Sunday and 7 is Sunday
    config.set('control1', 'mode', 'auto') ## auto, discharge, charge, standby, auto+
    config.set('control1', 'discharge', '500') ## Discharge in watt
    config.set('control1', 'charge', '500') ## Charge in watt
    config.set('control1', 'dynamic_charge', 'yes') ## If yes, the power charge is dynamically adjusted to the maximum grid power (feedpower value)

    ##TODO: Add mqtt config
    config.add_section('mqtt-broker') 
    config.set('mqtt-broker','name', 'SofarControl')
    config.set('mqtt-broker','user', 'moscon')
    config.set('mqtt-broker','password', 'pepon')
    config.set('mqtt-broker','broker', '192.168.90.17')
    config.set('mqtt-broker','port', '1883')
    config.set('mqtt-broker','publishTime','60')
    
    config.add_section('mqtt-pub')
    config.set('mqtt-pub','BatCapacity', 'sofar/soc')
    config.set('mqtt-pub','BatVoltage', 'sofar/batvoltage')
    config.set('mqtt-pub','BatCurrent', 'sofar/batcurrent')
    config.set('mqtt-pub','BatTemperature', 'sofar/battemp')
    config.set('mqtt-pub','InverterTemperature', 'sofar/invtemp')
    config.set('mqtt-pub','SolarPower', 'sofar/solpower')
    config.set('mqtt-pub','FeedPower', 'sofar/feedpower')
    config.set('mqtt-pub','InverterState', 'sofar/invstate')
    config.set('mqtt-pub','LoadPower', 'sofar/loadpower')
    config.set('mqtt-pub','EPSPower', 'sofar/epspower')
    config.set('mqtt-pub','EPSVoltage', 'sofar/epsvoltage')
    config.set('mqtt-pub','DayFV', 'sofar/dayfv')
    config.set('mqtt-pub','DayLoad', 'sofar/dayload')
    config.set('mqtt-pub','DayBuyFeed', 'sofar/daybuyfeed')
    config.set('mqtt-pub','DaySellFeed', 'sofar/daysellfeed')
    config.set('mqtt-pub','IOPower', 'sofar/iopower')
    
    config.add_section('mqtt-get')
    config.set('mqtt-get','mode', 'sofar/mode')
    config.set('mqtt-get','charge', 'sofar/charge')
    config.set('mqtt-get','discharge', 'sofar/discharge')
    config.set('mqtt-get','dynamic_charge', 'sofar/dynamic_charge')
    config.set('mqtt-get','truefeedpower', 'has/truefeedpower')
        
    config.add_section('mqtt-domoticz')
    config.set('mqtt-domoticz','BatCapacity', '45') # For domoticz only mqtt idx
    config.set('mqtt-domoticz','BatVoltage', '32')
    config.set('mqtt-domoticz','BatCapacity', '55')
    config.set('mqtt-domoticz','BatVoltage', '57')
    config.set('mqtt-domoticz','BatCurrent', '59')
    config.set('mqtt-domoticz','BatTemperature', '60')
    config.set('mqtt-domoticz','InverterTemperature', '63')
    config.set('mqtt-domoticz','SolarPower', '64')
    config.set('mqtt-domoticz','FeedPower', '87')
    config.set('mqtt-domoticz','InverterState', '65')
    config.set('mqtt-domoticz','LoadPower', '49')
    config.set('mqtt-domoticz','EPSPower', '34')
    config.set('mqtt-domoticz','EPSVoltage', '67')
    config.set('mqtt-domoticz','DayFV', '67')
    config.set('mqtt-domoticz','DayLoad', '67')
    config.set('mqtt-domoticz','DayBuyFeed', '67')
    config.set('mqtt-domoticz','DaySellFeed', '67')
    config.set('mqtt-domoticz','IOPower', '67')
    
    with open('/etc/sfc.ini', 'w') as f:
        config.write(f) 



logger.info("Initializing SFC")

try:
    Sofar = me3000ModBus(config.get('main','port'), config.getint('main','modbus_slave'))
    logger.info("Found serial port " + config.get('main','port'))
except Exception as e:
    logger.error (str(e))
    logger.critical("Not found serial port " + config.get('main','port'))
    sys.exit("End of program")


try:
    if config.has_section('mqtt-broker'):
        mqttPubName=config.get('mqtt-broker','name') # publish data
        client = mqtt.Client("", True, None, mqtt.MQTTv31)
        client.on_disconnect = on_disconnect
        client.on_connect = on_connect
        client.username_pw_set(config.get('mqtt-broker','user'),password=config.get('mqtt-broker','password'))
        client.connect(config.get('mqtt-broker','broker'),config.getint('mqtt-broker','port'),60)
        client.on_message = on_message
        client.loop_start()

        time.sleep(2)
except Exception as e:
        logger.error (str(e))
        logger.critical ("Unrecoverable error, disable mqtt in configuration ")
        sys.exit()

# cfgControl
cfgCtrl = configControl()

## Jobs
schedule.every(5).seconds.do(dynamicCharge)
if config.has_section('mqtt-broker'):
    try:
        if (config.has_option('mqtt-broker','publishTime')):
            schedule.every(config.getint('mqtt-broker', 'publishTime')).seconds.do(mqttRun)
        else:
            schedule.every(60).seconds.do(mqttRun)
    except Exception as e:
           logger.critical (str(e))
           sys.exit(1)
schedule.every(60).seconds.do(getDataRun)
sleep(5)
schedule.every(60).seconds.do(controlRun) ## Control the operating inverter modes
init = 0
exec = 0
## Test global dynamic charge
if (config.has_option('main','dynamic_charge')):
    Sofar.dynamic_charge = config.getboolean('main', 'dynamic_charge')


while True:
    schedule.run_pending()
    time.sleep(1)
    init = 1
