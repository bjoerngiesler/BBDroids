#!/usr/bin/env python3

from UDPHandler import UDPHandler
from RemoteHandler import RemoteHandler
from Utilities import vectorToAngles
import dearpygui.dearpygui as dpg
import sys
import time

VAL_DRIVE_GOAL          = 0
VAL_DRIVE_CURRENT_PWM   = 1
VAL_DRIVE_CURRENT_SPEED = 2
VAL_DRIVE_CURRENT_POS   = 3

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
		self.imuXData = []
		self.imuYData = []
		self.imuZData = []
		self.driveCtrlGoalData = []
		self.driveCtrlCurrentData = []
		self.timeData = []
		self.steerByIMU = False
		self.startTime = time.time()

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

	def newDroidDiscoveredCallback(self):
		discovered = self.handler.addressesDiscovered()
		dpg.configure_item(self.droidsbox, items=discovered)
		if len(discovered) == 1:
			self.handler.selectAddress(discovered[0])
			dpg.configure_item(self.droidsbox, default_value = discovered[0])

	def servoSliderCallback(self, appdata, userdata):
		for i in range(len(self.servoSliders)):
			if self.servoSliders[i] == appdata:
				self.handler.queueIndexedFloatCommand(CMD_SET_SERVO_NR, i, userdata)

	def createDroidAndRemoteSelector(self):
		with dpg.window(label = "Select...", height=200, width=400, pos = [0,0]):
			self.droidsbox = dpg.add_listbox([], label="Droids", callback = self.selectDroidCallback)
			self.remotesbox = dpg.add_listbox(self.remote.getDeviceNameCandidates(), label="Remotes", callback = self.selectRemoteCallback)

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
		with dpg.window(label="Status", height=600, width=500, pos=[420, 0]):
			self.hzText = dpg.add_text("GUI: -Hz Droid messages: -Hz Framedrops: - (-%)")
			dpg.add_text("Status: ")
			dpg.add_same_line()
			self.imusText = dpg.add_text("IMUs")
			dpg.add_same_line()
			self.motorsText = dpg.add_text("Motors")
			dpg.add_same_line()
			self.servosText = dpg.add_text("Servos")
			dpg.add_same_line()
			self.seqnumText = dpg.add_text("Sequence Num: -")

			self.servoSliders = []
			for num in range(1, 5):
				self.servoSliders.append(dpg.add_slider_float(label="Servo %d" % num, min_value=60, max_value=300,
					callback = self.servoSliderCallback))

			self.steerByIMUCheckbox = dpg.add_checkbox(label="Use remote IMU", default_value=False, callback=self.steerByIMUCallback)

			with dpg.plot(label="Drive Control", width=480, height=200):
				dpg.add_plot_legend()
				self.driveCtrlPlotXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="t")
				dpg.set_axis_limits(self.driveCtrlPlotXAxis, self.startTime, self.startTime+10.24)
				self.driveCtrlPlotYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="value")
				self.driveCtrlGoalSeries = dpg.add_line_series([], [], label="goal", parent=self.driveCtrlPlotYAxis)
				self.driveCtrlCurrentSeries = dpg.add_line_series([], [], label="current", parent=self.driveCtrlPlotYAxis)

			with dpg.plot(label="IMU", width=480, height=200):
				dpg.add_plot_legend()
				self.bodyIMUPlotXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="seqnum")
				self.bodyIMUPlotYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="reading")
				self.bodyIMUXSeries = dpg.add_line_series([], [], label="x", parent=self.bodyIMUPlotYAxis)
				self.bodyIMUYSeries = dpg.add_line_series([], [], label="y", parent=self.bodyIMUPlotYAxis)
				self.bodyIMUZSeries = dpg.add_line_series([], [], label="z", parent=self.bodyIMUPlotYAxis)


	def collectData(self):
		if self.handler.readIfAvailable():
			self.timeData.append(time.time()-self.startTime)
			while len(self.timeData) > 256:
				del self.timeData[0]

			dpg.configure_item(self.seqnumText, default_value="Sequence: %d" % self.handler.getSeqNum())
			if self.lastSeqnum is not None and self.handler.getSeqNum() != (self.lastSeqnum+1)%256:
				fd = (self.handler.getSeqNum() - (self.lastSeqnum+1)%256)
				if fd < 0:
					fd += 256
				self.droppedFrames += fd
				dpg.configure_item(self.seqnumText, color=(255, 0, 0, 255))
			else:
				dpg.configure_item(self.seqnumText, color=(255, 255, 255, 255))
				self.goodFrames += 1
			
			if self.lastDroidMsgTime is not None:
				while len(self.droidMsgIntervals) > 20:
					del self.droidMsgIntervals[0]
				self.droidMsgIntervals.append(time.time() - self.lastDroidMsgTime)
				self.droidMsgHz = 1.0 / (sum(self.droidMsgIntervals) / len(self.droidMsgIntervals))

			self.lastDroidMsgTime = time.time()

			self.driveCtrlGoalData.append(self.handler.getFloatVal(VAL_DRIVE_GOAL))
			self.driveCtrlCurrentData.append(self.handler.getFloatVal(VAL_DRIVE_CURRENT_PWM))
			while len(self.driveCtrlGoalData) > 256:
				del self.driveCtrlGoalData[0]
			while len(self.driveCtrlCurrentData) > 256:
				del self.driveCtrlCurrentData[0]

			dpg.set_value(self.driveCtrlGoalSeries, [self.timeData, self.driveCtrlGoalData])
			dpg.set_value(self.driveCtrlCurrentSeries, [self.timeData, self.driveCtrlCurrentData])
			dpg.set_axis_limits(self.driveCtrlPlotXAxis, self.timeData[0], self.timeData[0]+10.24)

			
			if self.handler.getMotorsOK():
					dpg.configure_item(self.motorsText, color=(0, 255, 0, 255))
			else:
				dpg.configure_item(self.motorsText, color=(255, 0, 0, 255))
			

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
		
		self.createDroidAndRemoteSelector()	
		self.createStatusWindow()
		self.createRemoteWindow()

		dpg.setup_dearpygui()
		dpg.show_viewport()
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
