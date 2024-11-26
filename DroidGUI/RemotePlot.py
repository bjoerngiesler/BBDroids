#!/usr/bin/env python3.10

import numpy as np
from scipy import signal
import time

import sys
import os
import re
import fcntl
import select

import dearpygui.dearpygui as dpg
from math import sqrt, pi

from DataPlot import DataPlot
from Filters import CurioResHPF as HPF
from Filters import CurioResBPF as BPF
from Filters import CurioResLPF as LPF
from Filters import MultiFilter

def appendMaxLen(l, item, maxlen):
	l.append(item)
	if len(l) > maxlen:
		return l[len(l)-maxlen:]
	return l

class BBRemotePlot:
	def __init__(self):
		self.lastseqnum = 0

		self.frequency = 100
		self.listlen = int(10*self.frequency) # 3s - running status comes every 40ms

		self.seqnumList = []

		self.axList = []
		self.ayList = []
		self.azList = []
		self.aList = []

		aLowFilterFreq = 1
		aHighFilterFreq = 20000
		self.axFilter = MultiFilter(aLowFilterFreq, aHighFilterFreq, self.frequency)
		self.ayFilter = MultiFilter(aLowFilterFreq, aHighFilterFreq, self.frequency)
		self.azFilter = MultiFilter(aLowFilterFreq, aHighFilterFreq, self.frequency)
		self.filterAccel = False	
		
		self.vx = 0
		self.vy = 0
		self.vz = 0
		self.vxList = []
		self.vyList = []
		self.vzList = []
		self.vList = []

		self.vxFilter = MultiFilter(aLowFilterFreq, aHighFilterFreq, self.frequency)
		self.vyFilter = MultiFilter(aLowFilterFreq, aHighFilterFreq, self.frequency)
		self.vzFilter = MultiFilter(aLowFilterFreq, aHighFilterFreq, self.frequency)
		self.filterVel = True

		self.x = 0
		self.y = 0
		self.z = 0
		self.xList = []
		self.yList = []
		self.zList = []

		self.f = self.openPort()
		if self.f is None:
			print("Error opening serial connection")
			sys.exit(-1)

		self.paused = False

	def openPort(self):
		dir_name = "/dev"
		base_name = "cu.usbmodem"
		files = [os.path.join(dir_name, f) for f in os.listdir(dir_name) if re.match(base_name+".*", f)]

		f = None
		for file in files:
			try:
				f = open(file, "r+")
			except OSError:
				continue
			print("Reading from ", file)
			f.write("remote running_status on\n")

			fd = f.fileno()
			flag = fcntl.fcntl(fd, fcntl.F_GETFD)
			print(flag)
			print(os.O_NONBLOCK)
			fcntl.fcntl(fd, fcntl.F_SETFD, flag | os.O_NONBLOCK)
			flag = fcntl.fcntl(fd, fcntl.F_GETFD)
			print(flag)
			if flag & os.O_NONBLOCK:
			    print("O_NONBLOCK!!")

		return f

	def dataAvailable(self):
		l = select.select((self.f.fileno(),),(),(),0)
		if len(l[0]) != 0:
			return True
		else:
			return False

	def readLineIfAvailable(self):
		if self.dataAvailable() == False:
			return
		line = ""
		while True:
			c = self.f.read(1)
			if(c == "\n"):
				return line
			line = line + c

	def collectData(self):
		line = self.readLineIfAvailable()
		if not line.startswith("S"):
			#print("Unhandled line \"" + line + "\"")
			pass
		matches = re.match("S([0-9]*) .*AX(-?[0-9]+\\.[0-9]+) AY(-?[0-9]+\\.[0-9]+) AZ(-?[0-9]+\\.[0-9]+).*", line)
		if matches is None: 
			return
		if self.paused:
			return
		seqnum = int(matches[1])
		ax = float(matches[2])*9.80665
		ay = float(matches[3])*9.80665
		az = float(matches[4])*9.80665
		self.processData(seqnum, ax, ay, az)

	def processData(self, seqnum, ax, ay, az):
		if self.filterAccel is True:
			ax = self.axFilter.filter(ax)
			ay = self.ayFilter.filter(ay)
			az = self.azFilter.filter(az)
		
		a = sqrt(ax*ax + ay*ay + az*az)
		
		self.seqnumList = appendMaxLen(self.seqnumList, seqnum, self.listlen)

		# Collect and plot acceleration
		self.axList = appendMaxLen(self.axList, ax, self.listlen)
		self.ayList = appendMaxLen(self.ayList, ay, self.listlen)
		self.azList = appendMaxLen(self.azList, az, self.listlen)
		self.aList = appendMaxLen(self.aList, a, self.listlen)

		# Integrate for velocity
		if seqnum != self.lastseqnum+1 and self.lastseqnum != 0:
			print("Expected seqnum %d, got %d - packet loss" % (self.lastseqnum+1, seqnum))
		if self.lastseqnum == 0:
			self.lastseqnum = seqnum
			return

		dt = 1.0/self.frequency
		self.vx = self.vx + ax*dt
		self.vy = self.vy + ay*dt
		self.vz = self.vz + az*dt
		if self.filterVel:
			self.vx = self.vxFilter.filter(self.vx)
			self.vy = self.vyFilter.filter(self.vy)
			self.vz = self.vzFilter.filter(self.vz)
		v = sqrt(self.vx*self.vx + self.vy*self.vy + self.vz*self.vz)

		# Collect and plot velocity
		self.vxList = appendMaxLen(self.vxList, self.vx, self.listlen)
		self.vyList = appendMaxLen(self.vyList, self.vy, self.listlen)
		self.vzList = appendMaxLen(self.vzList, self.vz, self.listlen)
		self.vList = appendMaxLen(self.vList, v, self.listlen)

		# Integrate once more for position
		self.x = self.x + self.vx*dt + ax*dt*dt/2.0
		self.y = self.y + self.vy*dt + ay*dt*dt/2.0
		self.z = self.z + self.vz*dt + az*dt*dt/2.0

		# Collect and plot position
		self.xList = appendMaxLen(self.xList, self.x, self.listlen)
		self.yList = appendMaxLen(self.yList, self.y, self.listlen)
		self.zList = appendMaxLen(self.zList, self.z, self.listlen)
		
		self.lastseqnum = seqnum

	def visualize(self):
		if len(self.seqnumList) < 1:
			return

		dpg.set_value(self.axSeries, [self.seqnumList, self.axList])
		dpg.set_value(self.aySeries, [self.seqnumList, self.ayList])
		dpg.set_value(self.azSeries, [self.seqnumList, self.azList])
		dpg.set_value(self.aSeries, [self.seqnumList, self.aList])

		# Create accel FFT
		if len(self.aList) > 10:
			ffta = np.fft.fft(self.aList)
			fftabs = np.abs(ffta).tolist()

			n = len(self.aList)
			t = 1.0/self.frequency
			xf = np.linspace(0.0, 1.0/(2.0*t), n//2)
			dpg.set_value(self.accelFFTSeries, [xf.tolist(), fftabs])

		dpg.set_value(self.vxSeries, [self.seqnumList, self.vxList])
		dpg.set_value(self.vySeries, [self.seqnumList, self.vyList])
		dpg.set_value(self.vzSeries, [self.seqnumList, self.vzList])
		dpg.set_value(self.vSeries, [self.seqnumList, self.vList])

		# Create vel FFT
		if len(self.vList) > 10:
			fftv = np.fft.fft(self.vList)
			fftabs = np.abs(fftv).tolist()

			n = len(self.vList)
			t = 1.0/self.frequency
			xf = np.linspace(0.0, 1.0/(2.0*t), n//2)
			dpg.set_value(self.velFFTSeries, [xf.tolist(), fftabs])

		dpg.set_value(self.xSeries, [self.seqnumList, self.xList])
		dpg.set_value(self.ySeries, [self.seqnumList, self.yList])
		dpg.set_value(self.zSeries, [self.seqnumList, self.zList])

	def createRemoteWindow(self):
		w=620
		h=250
		with dpg.window(label="Remote", tag="Remote", height=-1, width=-1):
			with dpg.group(horizontal=False):
				with dpg.group(horizontal=True):
					with dpg.plot(label="Accel", width=w, height=h):
						dpg.add_plot_legend()
						self.accelXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="t", auto_fit=True)
						self.accelYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="g", auto_fit=True)
						self.axSeries = dpg.add_line_series([], [], label="ax", parent=self.accelYAxis)
						self.aySeries = dpg.add_line_series([], [], label="ay", parent=self.accelYAxis)
						self.azSeries = dpg.add_line_series([], [], label="az", parent=self.accelYAxis)
						self.aSeries = dpg.add_line_series([], [], label="|a|", parent=self.accelYAxis)
						dpg.set_axis_limits_auto(self.accelXAxis)
						dpg.set_axis_limits_auto(self.accelYAxis)

					with dpg.plot(label="FFT", width=w, height=h):
						dpg.add_plot_legend()
						self.accelFFTXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="freq", auto_fit=True)
						self.accelFFTYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="ampl", auto_fit=True)
						self.accelFFTSeries = dpg.add_line_series([], [], label="FFT", parent=self.accelFFTYAxis)

				with dpg.group(horizontal=True):
					with dpg.plot(label="Vel", width=w, height=h):
						dpg.add_plot_legend()
						self.velXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="t", auto_fit=True)
						self.velYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="m/s", auto_fit=True)
						self.vxSeries = dpg.add_line_series([], [], label="vx", parent=self.velYAxis)
						self.vySeries = dpg.add_line_series([], [], label="vy", parent=self.velYAxis)
						self.vzSeries = dpg.add_line_series([], [], label="vz", parent=self.velYAxis)
						self.vSeries = dpg.add_line_series([], [], label="|v|", parent=self.velYAxis)

					with dpg.plot(label="FFT", width=w, height=h):
						dpg.add_plot_legend()
						self.velFFTXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="freq", auto_fit=True)
						self.velFFTYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="ampl", auto_fit=True)
						self.velFFTSeries = dpg.add_line_series([], [], label="FFT", parent=self.velFFTYAxis)

				with dpg.group(horizontal=True):
					with dpg.plot(label="Pos", width=w*2, height=h):
						dpg.add_plot_legend()
						self.posXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="t", auto_fit=True)
						self.posYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="m", auto_fit=True)
						self.xSeries = dpg.add_line_series([], [], label="x", parent=self.posYAxis)
						self.ySeries = dpg.add_line_series([], [], label="y", parent=self.posYAxis)
						self.zSeries = dpg.add_line_series([], [], label="z", parent=self.posYAxis)
					self.pauseButton = dpg.add_button(label="Pause", callback=self.pauseCallback)

	def pauseCallback(self):
		if self.paused == True:
			dpg.configure_item(self.pauseButton, label="Pause")
			self.paused = False
		else:
			dpg.configure_item(self.pauseButton, label="Continue")
			self.paused = True

	def runStandalone(self):
		dpg.create_context()
		dpg.create_viewport()
		
		self.createRemoteWindow()

		dpg.setup_dearpygui()
		dpg.show_viewport()
		dpg.set_primary_window("Remote", True)
		while dpg.is_dearpygui_running():
			while self.dataAvailable():
				self.collectData()
			self.visualize()
			dpg.render_dearpygui_frame()

		dpg.destroy_context()

if __name__ == "__main__":
	plot = BBRemotePlot()
	plot.runStandalone()

