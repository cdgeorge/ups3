# coding=UTF-8
#
# U-GEEK Raspi Smart UPS HAT V3
#

import os
import sys
import time
import smbus
import signal
import threading
import traceback
from neopixel import *

# Global settings
BUS_ADDR 		= 1
disconnectflag 	= False
exit_thread 	= False
POWEROFF_POWER  = 5
count           = 0
UsbPluggedCount  = 0
DumpRegisters   = 0
ServiceWasRunning = False

# Global status vars
bq25895_status : dict = {}
stats_usbPlugged : dict = {}

#MAX17048 settings
MAX17048_ADDR 	= 0x36

# BQ25895 setgins
BQ25895_ADDRESS = 0x6A
REG_WATCHDOG    = 0x07
BYTE_WATCHDOG_STOP = 0b10001101 #Stop Watchdog timer
REG_ILIM 		= 0x00 #ILIM register
#BYTE_ILIM 		= 0b01101000 #2A input current limit
#BYTE_ILIM 		= 0b01111100 #3A input current limit
BYTE_ILIM 		= 0b01111111 #3.25A input current limit
REG_ICHG 		= 0x04
BYTE_ICHG 		= 0b01111111 #.5A charging current limit
REG_CONV_ADC 	= 0x02
REG_BATFET 		= 0x09
BYTE_BATFET 	= 0b01001000 #delay before battery == disconnected
BAT_CAPACITY 	= 2500 #Battery capacity in mah
CURRENT_DRAW 	= 2000 #Current draw in mah
REG_CONV_ADC 	= 0x02
BYTE_CONV_ADC_START = 0b10011101
BYTE_CONV_ADC_STOP  = 0b00011101
REG_BATFET_DIS  = 0x09
BYTE_BATFET_DIS = 0b01101000
REG_STATUS		= 0x0B #address of status register
REG_BATV		= 0x0E
REG_FAULT		= 0x0C
REG_BATI		= 0x12

VBUS_STAT_TYPE = [
	"N/C", #000: No Input 
	"SDP", #001: USB Host SDP
	"CDP", #010: USB CDP (1.5A)
	"DCP", #011: USB DCP (3.25A)
	"HiV", #100: Adjustable High Voltage DCP (MaxCharge) (1.5A)
	"Unk", #101: Unknown Adapter (500mA)
	"Nstd",#110: Non-Standard Adapter (1A/2A/2.1A/2.4A)
	"OTG"  #111: OTG
]

def log(str):
	print(str)

# Init i2c bus
def init_i2c():
	global bus
	bus = smbus.SMBus(BUS_ADDR)

# Init max17048	
def max17048_init():
	bus.write_word_data(MAX17048_ADDR, 0xFE ,0xFFFF)
	return True

class Max17048:
	def readReg(self, reg):
		val=bus.read_word_data(MAX17048_ADDR, reg)
		val=((val & 0x00FF) << 8) + (val >> 8)
		return val
	def update(self):
		self.v=self.readReg(0x02)*78.125/1000000
		self.soc=self.readReg(0x04)/256
		self.crate=self.readReg(0x16)
		if self.crate>=0x8000:
			self.crate-=0x10000
		self.crate*=0.208

max17048 = Max17048()

# Get voltage from max17048
def max17048_getstatus():
	global max17048
	max17048.update()

# Init bq25895
def bq25895_init():
	bus.write_byte_data(BQ25895_ADDRESS, REG_WATCHDOG, BYTE_WATCHDOG_STOP)
	bus.write_byte_data(BQ25895_ADDRESS, REG_ILIM, BYTE_ILIM)
	bus.write_byte_data(BQ25895_ADDRESS, REG_ICHG, BYTE_ICHG)
	bus.write_byte_data(BQ25895_ADDRESS, REG_BATFET, BYTE_BATFET)

def bq25895_int_to_bool_list(num):
	return [bool(num & (1<<n)) for n in range(8)]

def bq25895_translate(val, in_from, in_to, out_from, out_to):
	out_range = out_to - out_from
	in_range = in_to - in_from
	in_val = val - in_from
	val=(float(in_val)/in_range)*out_range
	out_val = out_from+val
	return out_val

# BQ25895 read status
def bq25895_read_status():
	global SLEEPDELAY, disconnectflag, batpercentprev, bq25895_status, UsbPluggedCount
	bq25895_regs={}
	bus.write_byte_data(BQ25895_ADDRESS, REG_CONV_ADC, BYTE_CONV_ADC_START)
	sample = bus.read_byte_data(BQ25895_ADDRESS, REG_STATUS)
	bq25895_regs["REG0B"]=sample
	status = bq25895_int_to_bool_list(sample)
	time.sleep(1.2)
	sample = bus.read_byte_data(BQ25895_ADDRESS, REG_BATV)
	bq25895_regs["REG0E"]=sample
	batvbool = bq25895_int_to_bool_list(sample)
	bus.write_byte_data(BQ25895_ADDRESS, REG_CONV_ADC, BYTE_CONV_ADC_STOP)
	sample = bus.read_byte_data(BQ25895_ADDRESS, 0x13)
	bq25895_regs["REG13"]=sample
	IDPM_LIM_BITS=sample&0x3f
	vbus_stat = status[7] * 4 + status[6] * 2 + status[5]

	if BYTE_ILIM&0x3f != IDPM_LIM_BITS:
		log("Detect diff:" +str(IDPM_LIM_BITS) + " expect:" + str(BYTE_ILIM&0x3f))
		UsbPluggedCount = UsbPluggedCount +1
		bus.write_byte_data(BQ25895_ADDRESS, REG_ILIM, BYTE_ILIM)

	if status[2]:
		power = "Connected"
	else:
		power = "Not Connected"

	if status[3] and status[4]:
		charge = "Charging done"
	elif status[4] and  not status[3]:
		charge = "Charging"
	elif not status[4] and status[3]:
		charge = "Pre-Charge"
	else:
		charge = "Not Charging"
	
	#convert batv register to volts
	batv = 2.304
	batv += batvbool[6] * 1.280
	batv += batvbool[5] * 0.640
	batv += batvbool[4] * 0.320
	batv += batvbool[3] * 0.160
	batv += batvbool[2] * 0.08
	batv += batvbool[1] * 0.04
	batv += batvbool[0] * 0.02   

	batpercent = bq25895_translate(batv,3.5,4.184,0,1)
	if batpercent<0 :
		batpercent = 0
	elif batpercent >1 :
		batpercent = 1
	
	timeleftmin = int( batpercent * 60* BAT_CAPACITY / CURRENT_DRAW)
	if timeleftmin < 0 :
		timeleftmin = 0
	
	if power == "Connected" :
		timeleftmin = -1        
	
	if power == "Not Connected" and disconnectflag == False :
		disconnectflag = True
		message = "echo Power Disconnected, system will shutdown in %d minutes! | wall" % (timeleftmin)
		#os.system(message)
	
	if power == "Connected" and disconnectflag == True :
		disconnectflag = False
		message = "echo Power Restored, battery at %d percent | wall" % (batpercentprev * 100)
		#os.system(message)

	batpercentprev = batpercent
	if DumpRegisters:
		for reg in range(0x14+1):
			key = "REG%02X" %reg
			if key in bq25895_regs:
				value = bq25895_regs[key]
			else:
				value = bus.read_byte_data(BQ25895_ADDRESS, reg)
				bq25895_regs[key]= value


	
	bq25895_status = {
		**bq25895_regs,
		'batv' : batv,
		'Input': power,
		'ChargeStatus' : charge,
		'BatteryVoltage' : '%.2f' % batv,
		"BatterySOC" : int(batpercent*100),
		'TimeRemaining' : int(timeleftmin),
		'vbus_stat': vbus_stat,
		'VBUS_STAT' : VBUS_STAT_TYPE[vbus_stat]
	}
	
	if(batv < 3.5):
		bus.write_byte_data(BQ25895_ADDRESS, REG_BATFET_DIS, BYTE_BATFET_DIS)

def calc_stats():
	global bq25895_status, stats_usbPlugged
	monoNow=time.clock_gettime(time.CLOCK_MONOTONIC)
	usbSocPerSec=0
	stepSocPerSec=0
	vbus_stat=bq25895_status['vbus_stat']
	soc=max17048.soc
	count=0
	if not 'vbus_stat' in stats_usbPlugged or stats_usbPlugged['vbus_stat'] != vbus_stat:
		stepTime=monoNow
		usbPluggedTime=monoNow
		usbPluggedSoc=soc
	else:
		count=stats_usbPlugged['count']
		usbPluggedTime=stats_usbPlugged['usbPluggedTime']
		usbPluggedSoc=stats_usbPlugged['usbPluggedSoc']
		stepTime=stats_usbPlugged['stepTime']
		lastSoc=stats_usbPlugged['soc']
		if soc!=lastSoc:
			count+=1
			stepSocPerSec=(soc-lastSoc)/(monoNow-stepTime)
			usbSocPerSec=(soc-usbPluggedSoc)/(monoNow-usbPluggedTime)
		else:
			return

	stats_usbPlugged = {
		'count' : count,
		'usbPluggedTime' : usbPluggedTime,
		'usbPluggedSoc' : usbPluggedSoc,
		'stepTime' : stepTime,
		'soc' : soc,
		'vbus_stat' : vbus_stat,
		'usbSocPerSec' : usbSocPerSec,
		'stepSocPerSec' : stepSocPerSec
	}

def log_all_registers_bq25895():
	for reg in range(0x14+1):
		key = "REG%02X" %reg
		value = bq25895_status[key]
		addString=bin(value)
		if reg == 0x00:
			addString= "EN_HIZ:" + str(value&0x80>>7) +" EN_ILIM:" + str(value&0x40>>6) +" IINLIM:" + str((value&0x3F)*50+100)+"mA"
		elif reg == 0x04:
			addString= "EN_PUMPX:" + str(value&0x80>>7) +" ICHG:" + str((value&0x7f)*64) +"mA"
		elif reg == 0x06:
			addString= "VREG:" + str((value&0xFC>>2)*16+3840) +"mV BATLOWV:" + str((value&0x2)>>1) +" VRECHG:" +  str((value&0x1)*100+100)+"mV"
		elif reg == 0x0A:
			addString= "BOOSTV:" + str((value&0xF0>>4)*64+4550) +"mV"
		elif reg == 0x0B:
			addString= "VBUS_STAT:" + str(value&0xE0>>5) +" CHRG_STAT:" + str(value&0x18>>3) +" PG_STAT:" + str((value&0x04)>>2)+" SDP_STAT:" + str((value&0x2)>>1)+" VSYS_STAT:" + str((value&0x1))
		elif reg == 0x0C:
			addString= "WATCHDOG_FAULT:" + str(value&0x80>>7) +" BOOST_FAULT:" + str(value&0x40>>6) +" CHRG_FAULT:" + str((value&0x30)>>4)+" BAT_FAULT:" + str((value&0xC)>>2)+" NTC_FAULT:" + str((value&0x3))
		elif reg == 0x0D:
			addString= "FORCE_VINDPM:" + str(value&0x80>>7) + " VINDPM:" + str((value&0x7f)*100+2600)+"mV"
		elif reg == REG_BATV: # 0x0E
			addString= "THERM_STAT:" + str(value&0x80>>7) + " BATV:" + str((value&0x7f)*20+2304)+"mV"
		elif reg == 0x0F:
			addString= "SYSV:" + str((value&0x7f)*20+2304)+"mV"
		elif reg == 0x10:
			addString= "TSPCT:" + str((value&0x7f)*0.465+21.0)+"%"
		elif reg == 0x11:
			addString= "VBUS_GD:" + str(value&0x80>>7) + " VBUSV:" + str((value&0x7f)*100+2600)+"mV"
		elif reg == REG_BATI: # 0x12
			addString= "ICHGR:" + str((value&0x7f)*50)+"mA"
		elif reg == 0x13:
			addString= "VDPM_STAT:" + str(value&0x80>>7) +" IDPM_STAT:" + str(value&0x40>>6) +" IDPM_LIM:" + str((value&0x3f)*50+100)+"mA"
		elif reg == 0x14:
			addString= "REG_RST:" + str(value&0x80>>7) +" ICO_OPTIMIZED:" + str(value&0x40>>6) +" PN:" + str(value&0x38>>3)  +" TS_PROFILE:" + str(value&0x04>>2)  +" DEV_REV:" + str((value&0x3))
		log( " bq25895 " +key+":  0x%02x"%value+" "+addString )

def print_bq25895status():
	global count
	count = count + 1
	print ("         Count: " , count)
	print ("         Input: " , bq25895_status['Input'])
	print ("  ChargeStatus: " , bq25895_status['ChargeStatus'])
	print ("BatteryVoltage: " , bq25895_status['BatteryVoltage'], "V")
	print ("     Vbus_stat: " , bq25895_status['VBUS_STAT'])
	if DumpRegisters:
         log_all_registers_bq25895()
	
def print_max17048status():
	#print ("Status of max17048:")
	#print ("BatteryVoltage: " , '%.2f' % max17048.v , "V")
	print ("           SOC: " , max17048.soc , "%")
	print ("         crate:  %.3f" % (max17048.crate) , "%/h")
	#print ("Status of bq25895:")

def print_stats():
	global UsbPluggedCount
	usbSocPerSec=stats_usbPlugged['usbSocPerSec']
	stepSocPerSec=stats_usbPlugged['stepSocPerSec']
	print ("UsbPluggedCount: ", UsbPluggedCount)
	print ("         count: ", stats_usbPlugged['count'])
	print ("  usbSocPerSec:  %.2f"% (usbSocPerSec*3600) + "%/h")
	print (" stepSocPerSec:  %.2f"% (stepSocPerSec*3600) + "%/h")
	powerLeft=max17048.soc - POWEROFF_POWER
	powerUp=95-max17048.soc
	if powerLeft>0 and usbSocPerSec<0:
		print ("  power off in:  %.1f"% (powerLeft/(-usbSocPerSec*60)), "min")
	elif powerUp>0 and usbSocPerSec>0:
		print ("    charged in:  %.1f"% (powerUp/(usbSocPerSec*60)), "min")
	

def get_print_all_status(): 
	max17048_getstatus()
	bq25895_read_status()
	calc_stats()
	print_bq25895status()
	print_max17048status()
	print_stats()
	print ("")
	
def handler(signum, frame):
	global DumpRegisters
	log("Signal is received:" + str(signum))
	if signum == signal.SIGUSR1:
		DumpRegisters=1
		log("Enable dump registers")
		return
	elif signum == signal.SIGUSR2:
		DumpRegisters=0
		log("Disable dump registers")
		return
	if ServiceWasRunning:
		os.system("systemctl start smartups")
	exit_thread=True
	#thread_led.join()
	exit(0)
	
def handle_signal():
	signal.signal(signal.SIGUSR1, handler)
	signal.signal(signal.SIGUSR2, handler)
	signal.signal(signal.SIGALRM, handler)
	signal.signal(signal.SIGINT, handler)
	signal.signal(signal.SIGQUIT, handler)

# Main Loop
if __name__ == '__main__':
	service_status = os.system("systemctl is-active --quiet smartups")
	if service_status == 0:
		ServiceWasRunning=True
		os.system("systemctl stop smartups")
	status_loop = False
	init_i2c()
	max17048_init()
	bq25895_init()
	handle_signal()
	print ("Reading status... ")
	
	try:
		if len(sys.argv) >= 2:
			if sys.argv[1] == "-t":
				status_loop = True

		if status_loop:
			print ("Start dump regs: sudo kill -10 "+str(os.getpid()))
			print (" Stop dump regs: sudo kill -12 "+str(os.getpid()))
		
		get_print_all_status()
			
		while status_loop is True:
			get_print_all_status()
		
		if ServiceWasRunning:
			os.system("systemctl start smartups")
	except Exception as e:
		print(e)
		traceback.print_exc()
		if ServiceWasRunning:
			os.system("systemctl start smartups")
		exit(20)
