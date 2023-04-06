#!/usr/bin/env python3

from UDPHandler import UDPHandler, CMD_SET_SERVO_NR, CMD_SET_ALL_SERVOS_NR, CMD_SET_DRIVE_MOTOR_SPEED_NR
from RemoteHandler import RemoteHandler
from Utilities import vectorToAngles
import dearpygui.dearpygui as dpg
import sys
import time

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
		self.domeIMUXData = []
		self.domeIMUYData = []
		self.domeIMUZData = []
		self.bodyIMUXData = []
		self.bodyIMUYData = []
		self.bodyIMUZData = []
		self.imuTimeData = []
		self.steerByIMU = False

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

			with dpg.plot(label="Dome IMU", width=480, height=200):
				dpg.add_plot_legend()
				self.domeIMUPlotXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="seqnum")
				self.domeIMUPlotYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="reading")
				self.domeIMUXSeries = dpg.add_line_series([], [], label="x", parent=self.domeIMUPlotYAxis)
				self.domeIMUYSeries = dpg.add_line_series([], [], label="y", parent=self.domeIMUPlotYAxis)
				self.domeIMUZSeries = dpg.add_line_series([], [], label="z", parent=self.domeIMUPlotYAxis)

			with dpg.plot(label="Body IMU", width=480, height=200):
				dpg.add_plot_legend()
				self.bodyIMUPlotXAxis = dpg.add_plot_axis(dpg.mvXAxis, label="seqnum")
				self.bodyIMUPlotYAxis = dpg.add_plot_axis(dpg.mvYAxis, label="reading")
				self.bodyIMUXSeries = dpg.add_line_series([], [], label="x", parent=self.bodyIMUPlotYAxis)
				self.bodyIMUYSeries = dpg.add_line_series([], [], label="y", parent=self.bodyIMUPlotYAxis)
				self.bodyIMUZSeries = dpg.add_line_series([], [], label="z", parent=self.bodyIMUPlotYAxis)

	def collectData(self):
		if self.handler.readIfAvailable():
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

			self.lastSeqnum = self.handler.getSeqNum()
			self.lastDroidMsgTime = time.time()

			
			if self.handler.getIMUsOK():
				dpg.configure_item(self.imusText, color=(0, 255, 0, 255))
				x, y, z = self.handler.getDomeIMUData()
				self.domeIMUXData.append(x)
				self.domeIMUYData.append(y)
				self.domeIMUZData.append(z)
				while len(self.domeIMUXData) > 256:
					del self.domeIMUXData[0]
				while len(self.domeIMUYData) > 256:
					del self.domeIMUYData[0]
				while len(self.domeIMUZData) > 256:
					del self.domeIMUZData[0]
				x, y, z = self.handler.getBodyIMUData()
				self.bodyIMUXData.append(x)
				self.bodyIMUYData.append(y)
				self.bodyIMUZData.append(z)
				while len(self.bodyIMUXData) > 256:
					del self.bodyIMUXData[0]
				while len(self.bodyIMUYData) > 256:
					del self.bodyIMUYData[0]
				while len(self.bodyIMUZData) > 256:
					del self.bodyIMUZData[0]

				self.imuTimeData.append(self.handler.getSeqNum())
				while len(self.imuTimeData) > 256:
					del self.imuTimeData[0]

				dpg.set_value(self.domeIMUXSeries, [self.imuTimeData, self.domeIMUXData])
				dpg.set_value(self.domeIMUYSeries, [self.imuTimeData, self.domeIMUYData])
				dpg.set_value(self.domeIMUZSeries, [self.imuTimeData, self.domeIMUZData])
				dpg.fit_axis_data(self.domeIMUPlotXAxis)
				dpg.set_value(self.bodyIMUXSeries, [self.imuTimeData, self.bodyIMUXData])
				dpg.set_value(self.bodyIMUYSeries, [self.imuTimeData, self.bodyIMUYData])
				dpg.set_value(self.bodyIMUZSeries, [self.imuTimeData, self.bodyIMUZData])
				dpg.fit_axis_data(self.bodyIMUPlotYAxis)
			else:
				dpg.configure_item(self.imusText, color=(255, 0, 0, 255))

			if self.handler.getMotorsOK():
					dpg.configure_item(self.motorsText, color=(0, 255, 0, 255))
			else:
				dpg.configure_item(self.motorsText, color=(255, 0, 0, 255))
			
			if self.handler.getServosOK():
				dpg.configure_item(self.servosText, color=(0, 255, 0, 255))
				servodata = self.handler.getServoData()
				for i in range(len(servodata)):
					dpg.configure_item(self.servoSliders[i], default_value = servodata[i])
			else:
				dpg.configure_item(self.servosText, color=(255, 0, 0, 255))

		if self.remote.isConnected() == True:
			self.remote.readAndParseLine()
			self.remote.flush()

		rollData, pitchData, yawData = self.remote.getGyroData()
		seqnumData = self.remote.getSeqnumData()
		dpg.set_value(self.rollSeries, [seqnumData, rollData])
		dpg.set_value(self.pitchSeries, [seqnumData, pitchData])
		dpg.set_value(self.yawSeries, [seqnumData, yawData])
		dpg.fit_axis_data(self.plotXAxis)

		while len(self.processingTimeIntervals) > 20:
			del self.processingTimeIntervals[0]
		self.processingTimeIntervals.append(time.time() - self.lastProcessingTime)
		self.guiHz = 1.0 / (sum(self.processingTimeIntervals) / len(self.processingTimeIntervals))

		framedropPercent = 100.0
		if self.goodFrames:
			framedropPercent = 100.0*(self.droppedFrames / self.goodFrames)
			
		dpg.configure_item(self.hzText, default_value = "GUI: %.1fHz Droid messages: %.2fHz Framedrops: %d (%.0f%%)" % 
			(self.guiHz, self.droidMsgHz, self.droppedFrames, framedropPercent))
		self.lastProcessingTime = time.time()
		self.lastDroidMsgTime = time.time()

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
