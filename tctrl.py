#!/usr/bin/python

import sys
import serial
import argparse

def StartHeating():
	print "Heating Nozzle and Bed..."
	ser.write('at sm o\r')
	ser.readline()

def CancelHeating():
	print "Cooling Nozzle and Bed..."
	ser.write('at sm f\r')
	ser.readline()

def PrintSerialResponse():
	while True:
		value = ser.readline().strip()
		if value == "OK":
			break
		print value
		if value == "ERROR":
			break

#def SetMode(mode):

def Set_Temp():
	class SetTemp(argparse.Action):
		def __call__(self, parser, namespace, values, options_string=None):
			message = ''
			if len(values) != 2:
				message = 'argument {} requires two(2) arguments'.format(self.dest)
			elif values[0].upper() not in ['EST','EET','BST','BET']:
				message = 'first argument to {} must be one of EST,EET,BST or BET'.format(self.dest)
			else:
				try:
					values[1] = float(values[1])
				except ValueError:
					message = ('second argument to {} must be a floating point value'.format(self.dest))
			if message:
				raise argparse.ArgumentError(self, message)
			setattr(namespace,self.dest,values)
	return SetTemp

def Set_PID():
	class SetPID(argparse.Action):
		def __call__(self, parser, namespace, values, options_string=None):
			message = ''
			if len(values) != 2:
				message = 'argument {} requires two(2) arguments'.format(self.dest)
			elif values[0].upper() not in ['EKP','EKI','EKD','BKP','BKI','BKD']:
				message = 'first argument to {} must be one of EKP,EKI,BKD,BKP,BKI or BKD'.format(self.dest)
			else:
				try:
					values[1] = float(values[1])
				except ValueError:
					message = ('second argument to {} must be a floating point value'.format(self.dest))
			if message:
				raise argparse.ArgumentError(self, message)
			setattr(namespace,self.dest,values)
	return SetPID
  
parser = argparse.ArgumentParser(description='3D Printer Temperature Controller')
parser.add_argument('--enable', action='store_true', help='Begin heating bed and nozzle')
parser.add_argument('--disable', action='store_true', help='Stop heating bed and nozzle')
parser.add_argument('--device', nargs=1, default='/dev/ttyUSB0', help='Specify serial device (/dev/ttyUSB0 default)')
parser.add_argument('--baud', nargs=1, default=19200, help='Specify serial device baud rate (19200 default)')
parser.add_argument('--readtemp', nargs=1, choices=['ECT','ect','EST','est','EET','eet','BCT','bct','BST','bst','BET','bet','TM1','tm1','TM2','tm2','TC1','tc1','TC2','tc2'], default=None, help='Read specified device temperature')
parser.add_argument('--settemp', nargs='+', action=Set_Temp(), help='Set indicated temperature', metavar=['TempItem','TempValue'])
parser.add_argument('--getpid', nargs=1, choices=['EKP','ekp','EKI','eki','EKD','ekd','BKP','bkp','BKI','bki','BKD','bkd'], help='Get PID tuning values')
parser.add_argument('--setpid', nargs='+', action=Set_PID(), help='Set PID tuning values', metavar=['PID factor', 'Pid Value'])
parser.add_argument('--status', nargs='?', const="n", help='Display controller status')

try:
	args = parser.parse_args()
except ValueError as e:
	print e.strerror
	sys.exit(e.errno)

ser = serial.Serial(args.device, args.baud, timeout=1)

if args.enable:
	StartHeating()
elif args.disable:
	CancelHeating()

if args.readtemp is not None:
#	print "at gv {0}\r".format(args.readtemp[0])
	ser.write("at gv {0}\r".format(args.readtemp[0]))
	PrintSerialResponse()

if args.status is not None:
	verbose = args.status.upper()
	if verbose != "V":
		verbose = ""
	ser.write("at st {0}\r".format(verbose))
	PrintSerialResponse()

if args.settemp is not None:
	ser.write("at sv {0} {1}\r".format(args.settemp[0], args.settemp[1]))
	PrintSerialResponse()

if args.getpid is not None:
	ser.write("at gv {0}\r".format(args.getpid[0]))
	PrintSerialResponse()

if args.setpid is not None:
	ser.write("at sv {0} {1}\r".format(args.setpid[0], args.setpid[1]))
	PrintSerialResponse()

