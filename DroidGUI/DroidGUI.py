#!/usr/bin/env python3

from UDPHandler import UDPHandler
from RemoteHandler import RemoteHandler
from Utilities import vectorToAngles
import dearpygui.dearpygui as dpg
import sys
import time

from DataPlot import DataPlot

class BB8DearPyGui:
	def __init__(self):
		self.handler = UDPHandler()
		self.handler.setNewDroidDiscoveredCB(self.newDroidDiscoveredCallback)
		self.remote = RemoteHandler()
		self.lastSeqnum = None
		self.droppedFrames = 0
		self.goodFrames = 0
		self.guiHz = 0
		self.droidMsgHz = 0
		self.lastProcessingTime = time.time()
		self.lastDroidMsgTime = None
		self.processingTimeIntervals = []
		self.droidMsgIntervals = []
		self.steerByIMU = False
		self.startTime = time.time()

		self.drive0Plot = DataPlot(("goal", "current", "err", "errI", "errD", "control"), 1024)
		self.drive1Plot = DataPlot(("goal", "current", "err", "errI", "errD", "control"), 1024)
		self.drive2Plot = DataPlot(("goal", "current", "err", "errI", "errD", "control"), 1024)
		self.imu0Plot = DataPlot(("r", "p", "h", "dr", "dp", "dh", "ax", "ay", "az"), 1024)
		self.imu1Plot = DataPlot(("r", "p", "h", "dr", "dp", "dh", "ax", "ay", "az"), 1024)
		self.imu2Plot = DataPlot(("r", "p", "h", "dr", "dp", "dh", "ax", "ay", "az"), 1024)
		self.remoteLPlot = DataPlot(("axis0", "axis1", "axis2", "axis3", "axis4"), 1024)
		self.remoteRPlot = DataPlot(("axis0", "axis1", "axis2", "axis3", "axis4"), 1024)
		self.batt1Plot = DataPlot(("batt1 voltage", "batt1 current"), 1024)
		self.batt2Plot = DataPlot(("batt2 voltage", "batt2 current"), 1024)
		self.servoLoadPlot = DataPlot(("servo load 1 [.1[]", "servo load 2 [.1[]", "servo load 3 [.1[]", "servo load 4 [.1[]"), 1024)

	def selectDroidCallback(self, appdata, userdata):
		self.handler.selectAddress(userdata)

	def selectRemoteCallback(self, appdata, userdata):
		print("Select: ")
		print(userdata)
		if self.remote.connect(userdata):
			print("Connected")

	def steerByIMUCallback(self, appdata, userdata):
		if userdata == True:
			if self.remote.isConnected() == False:
				dpg.configure_item(self.steerByIMUCheckbox, default_value=False)
			else:
				print("Starting steering by IMU")
				data = self.remote.getGyroData()

				self.imuOffsetRoll = data[0][-1]
				self.imuOffsetPitch = data[1][-1]
				self.imuOffsetYaw = data[2][-1]

				print("Starting from IMU offset r=%.2f p=%.2f y=%.2f" % (self.imuOffsetRoll, self.imuOffsetPitch, self.imuOffsetYaw))

				self.servoOffset = self.handler.getServoData()

				print("Starting from servo offset r=%.2f p=%.2f y=%.2f" % (self.servoOffset[1], self.servoOffset[2], self.servoOffset[0]))

				self.steerByIMU = True
		else:
			print("Stopping steering by IMU")
			self.steerByIMU = False

	def newDroidDiscoveredCallback(self, address):
		print("New droid at ", address)
		dpg.configure_item(self.hzText, default_value="IP: "+address)

	def createRemoteWindow(self):
		with dpg.window(label = "Remote", height=180, width=920, pos = [0,610]):
			with dpg.plot(label="Remote Orientation", width=900, height=150):
				dpg.add_plot_legend()
				self.plotXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="time")
				self.plotYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="angle")
				dpg.set_axis_limits(self.plotYAxis, -180.0, 180.0)
				self.rollSeries = dpg.add_line_series([], [], label="roll", parent=self.plotYAxis)
				self.pitchSeries = dpg.add_line_series([], [], label="pitch", parent=self.plotYAxis)
				self.yawSeries = dpg.add_line_series([], [], label="yaw", parent=self.plotYAxis)

	def createStatusWindow(self):
		with dpg.window(label="Status", tag="Status", height=-1, width=-1):
			self.hzText = dpg.add_text("IP: - GUI: -Hz Droid messages: -Hz Framedrops: - (-%)")
			with dpg.group(horizontal=True):
				dpg.add_text("Status: ")
				self.imusText = dpg.add_text("IMUs")
				self.motorsText = dpg.add_text("Motors")
				self.servosText = dpg.add_text("Servos")
				self.seqnumText = dpg.add_text("Sequence Num: -")

			with dpg.tab_bar():	
				with dpg.tab(label="Drive 0"):
					self.drive0Plot.createGUI(-1, -1)
				with dpg.tab(label="Drive 1"):
					self.drive1Plot.createGUI(-1, -1)
				with dpg.tab(label="Drive 2"):
					self.drive2Plot.createGUI(-1, -1)
				with dpg.tab(label="IMU 0"):
					self.imu0Plot.createGUI(-1, -1)
				with dpg.tab(label="IMU 1"):
					self.imu1Plot.createGUI(-1, -1)
				with dpg.tab(label="IMU 2"):
					self.imu2Plot.createGUI(-1, -1)
				with dpg.tab(label="Remote L"):
					self.remoteLPlot.createGUI(-1, -1)
				with dpg.tab(label="Remote R"):
					self.remoteRPlot.createGUI(-1, -1)
				with dpg.tab(label="Battery 1"):
					self.batt1Plot.createGUI(-1, -1)
				with dpg.tab(label="Battery 2"):
					self.batt2Plot.createGUI(-1, -1)
				with dpg.tab(label="Servo Load"):
					self.servoLoadPlot.createGUI(-1, -1)



	def collectData(self):
		while True:
			packet = self.handler.readIfAvailable()
			if packet is None:
				break


			# dpg.configure_item(self.seqnumText, default_value="Sequence: %d" % self.handler.getSeqNum())
			# if self.lastSeqnum is not None and self.handler.getSeqNum() != (self.lastSeqnum+1)%256:
			# 	fd = (self.handler.getSeqNum() - (self.lastSeqnum+1)%256)
			# 	if fd < 0:
			# 		fd += 256
			# 	self.droppedFrames += fd
			# 	dpg.configure_item(self.seqnumText, color=(255, 0, 0, 255))
			# else:
			# 	dpg.configure_item(self.seqnumText, color=(255, 255, 255, 255))
			# 	self.goodFrames += 1
			
			# if self.lastDroidMsgTime is not None:
			# 	while len(self.droidMsgIntervals) > 20:
			# 		del self.droidMsgIntervals[0]
			# 	self.droidMsgIntervals.append(time.time() - self.lastDroidMsgTime)
			# 	self.droidMsgHz = 1.0 / (sum(self.droidMsgIntervals) / len(self.droidMsgIntervals))

			# self.lastDroidMsgTime = time.time()

			d = packet.drive[0]
			if d.errorState == 1:
				self.drive0Plot.addDataVector(packet.timestamp, (d.goal, d.presentSpeed, d.err, d.errI, d.errD, d.control))
			d = packet.drive[1]
			if d.errorState == 1:
				self.drive1Plot.addDataVector(packet.timestamp, (d.goal, d.presentSpeed, d.err, d.errI, d.errD, d.control))
			d = packet.drive[2]
			if d.errorState == 1:
				self.drive2Plot.addDataVector(packet.timestamp, (d.goal, d.presentSpeed, d.err, d.errI, d.errD, d.control))

			i = packet.imu[0]
			if i.errorState == 1:
				self.imu0Plot.addDataVector(packet.timestamp, (i.r, i.p, i.h, i.dr, i.dp, i.dh, i.ax, i.ay, i.az))
			i = packet.imu[1]
			if i.errorState == 1:
				self.imu1Plot.addDataVector(packet.timestamp, (i.r, i.p, i.h, i.dr, i.dp, i.dh, i.ax, i.ay, i.az))
			i = packet.imu[2]
			if i.errorState == 1:
				self.imu2Plot.addDataVector(packet.timestamp, (i.r, i.p, i.h, i.dr, i.dp, i.dh, i.ax, i.ay, i.az))

			b = packet.batt[0]
			if b.errorState == 1:
				self.batt1Plot.addDataVector(packet.timestamp, (b.voltage, b.current))
			b = packet.batt[1]
			if b.errorState == 1:
				self.batt2Plot.addDataVector(packet.timestamp, (b.voltage, b.current))

			# v = list(map(self.handler.getFloatVal, (VAL_ROLL_GOAL, VAL_ROLL_PRESENT, VAL_ROLL_ERR, VAL_ROLL_ERR_I, VAL_ROLL_ERR_D, VAL_ROLL_CONTROL)))
			# self.rollPlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			# v = list(map(self.handler.getFloatVal, (VAL_IMU_RAW_R, VAL_IMU_RAW_P, VAL_IMU_RAW_H, VAL_IMU_FILTERED_R, VAL_IMU_FILTERED_P, VAL_IMU_FILTERED_H)))
			# self.imuPlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			# v = list(map(self.handler.getFloatVal, (VAL_REMOTE_L_AXIS0, VAL_REMOTE_L_AXIS1, VAL_REMOTE_L_AXIS2, VAL_REMOTE_L_AXIS3, VAL_REMOTE_L_AXIS4)))
			# self.remoteLPlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			# v = list(map(self.handler.getFloatVal, (VAL_REMOTE_R_AXIS0, VAL_REMOTE_R_AXIS1, VAL_REMOTE_R_AXIS2, VAL_REMOTE_R_AXIS3, VAL_REMOTE_R_AXIS4)))
			# self.remoteRPlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			# v = list(map(self.handler.getFloatVal, (VAL_BATT1_VOLTAGE, VAL_BATT2_VOLTAGE)))
			# self.battVoltagePlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			# v = list(map(self.handler.getFloatVal, (VAL_BATT1_CURRENT, VAL_BATT2_CURRENT)))
			# self.battCurrentPlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			# v = list(map(self.handler.getFloatVal, (VAL_SERVO1_LOAD, VAL_SERVO2_LOAD, VAL_SERVO3_LOAD, VAL_SERVO4_LOAD)))
			# self.servoLoadPlot.addDataVector(self.handler.getFloatVal(VAL_TIMESTAMP), v)
			
			# if self.handler.getMotorsOK():
			# 	dpg.configure_item(self.motorsText, color=(0, 255, 0, 255))
			# else:
			# 	dpg.configure_item(self.motorsText, color=(255, 0, 0, 255))
			

	def steerByIMUHandler(self):
		if self.steerByIMU == False or self.remote.isConnected() == False:
			return
		data = self.remote.getGyroData()
		deltaRoll = self.imuOffsetRoll - data[0][-1]
		deltaPitch = self.imuOffsetPitch - data[1][-1]
		deltaYaw = self.imuOffsetYaw - data[2][-1]

		servodata = self.handler.getServoData()

		print("Offsets: %.2f %.2f %.2f" % (deltaRoll, deltaPitch, deltaYaw))

		self.handler.queueFloatListCommand(CMD_SET_ALL_SERVOS_NR, deltaYaw + self.servoOffset[0], deltaPitch + self.servoOffset[1], 
			deltaRoll + self.servoOffset[2], servodata[3])

	def run(self):
		dpg.create_context()
		dpg.create_viewport()
		
		self.createStatusWindow()
		#self.createRemoteWindow()

		dpg.setup_dearpygui()
		dpg.show_viewport()
		dpg.set_primary_window("Status", True)
		while dpg.is_dearpygui_running():
			self.collectData()
			if self.steerByIMU:
				self.steerByIMUHandler()
			self.handler.sendCommandQueue()
			dpg.render_dearpygui_frame()

		dpg.destroy_context()


if __name__ == "__main__":
	gui = BB8DearPyGui()
	gui.run()
