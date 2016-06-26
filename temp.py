#!/usr/bin/python

import sys
import serial
import Tkinter as tk
import tkFont
import time
from Tkinter import *
from tkMessageBox import *

class App():
	def __init__(self):
		self.ser = serial.Serial("/dev/ttyUSB0", 19200, timeout=1)
		self.root = tk.Tk()
		self.font = tkFont.Font(family="Helvetica", size=16)
		self.extruder_label = tk.Label(text="Extruder:", width=15, anchor=W, font=self.font)
		self.extruder_temp = tk.Label(text="", width=10, justify=RIGHT, font=self.font)
		self.extruder_set = tk.Label(text="", width=10, justify=RIGHT, font=self.font)
		self.extruder_label.grid(row=0,column=0)
		self.extruder_temp.grid(row=0,column=1)
		self.extruder_set.grid(row=0,column=2)
		self.bed_label = tk.Label(text="Bed:", width=15, anchor=W, font=self.font)
		self.bed_temp = tk.Label(text="", width=10, justify=RIGHT, font=self.font)
		self.bed_set = tk.Label(text="", width=10, justify=RIGHT, font=self.font)
		self.bed_label.grid(row=1,column=0)
		self.bed_temp.grid(row=1,column=1)
		self.bed_set.grid(row=1,column=2)
		self.enable_button = Button(text="Enable", command=self.enable_heat)
		self.disable_button = Button(text="Disable", command=self.disable_heat)
		self.enable_button.grid(row=2,column=0)
		self.disable_button.grid(row=2,column=2)
		self.root.title("Temp Controller")
		self.root.attributes("-topmost", True)
		self.update_temps()

	def update_temps(self):
		self.ser.write('at gv ect\r')
		current_temp = self.get_serial_response()
		self.extruder_temp.configure(text="{0}".format(current_temp))
		self.ser.write('at gv est\r')
		current_temp = self.get_serial_response()
		self.extruder_set.configure(text="{0}".format(current_temp))
		self.ser.write('at gv bct\r')
		current_temp = self.get_serial_response()
		self.bed_temp.configure(text="{0}".format(current_temp))
		self.ser.write('at gv bst\r')
		current_temp = self.get_serial_response()
		self.bed_set.configure(text="{0}".format(current_temp))
		self.root.after(1000, self.update_temps)

	def get_serial_response(self):
		result=""
		while True:
			value = self.ser.readline().strip()
			if value == "OK":
				break
			result = value;
			if value == "ERROR":
				break
		return result

	def get_mode(self):
		self.ser.write('at gv ctl\r')
		response = self.get_serial_response()
		if response == "ERROR":
			showerror("Mode", "Cannot get controller mode info")
		return int(response)

	def cancel_modes(self):
		self.ser.write('at rs\r')
		response = self.get_serial_response()
		if response == "ERROR":
			showerror("Mode", "Cannot cancel controller mode")

	def enable_heat(self):
		if self.get_mode() != 2:
			self.cancel_modes()			
		self.ser.write('at sm o\r')
		if self.get_serial_response() == "ERROR":
			showerror("Enable", "Controller in wrong mode or not on")

	def disable_heat(self):
		if self.get_mode() == 2:
			self.ser.write('at sm f\r')
			if self.get_serial_response() == "ERROR":
				showerror("Disable", "Controller in wrong mode or not on")

	def run(self):
		self.root.mainloop()

app=App()
app.run()
