import os
import re
import serial
import time

class RemoteHandler:
	def __init__(self):
		self.remote = None
		self.joyXData = []
		self.joyYData = []
		self.rollData = []
		self.pitchData = []
		self.yawData = []
		self.accelXData = []
		self.accelYData = []
		self.accelZData = []
		self.pot1Data = []
		self.pot2Data = []
		self.buttons = [False, False, False, False, False, False, False, False]
		self.seqnumData = []
		self.maxData = 256

	def getDeviceNameCandidates(self):
		devnames = []
		for f in os.listdir("/dev"):
			if re.match("tty.usbmodem.*", f):
				devnames.append("/dev/"+f)
		return devnames

	def connect(self, devname):
		self.remote = serial.Serial(devname, 115200)
		if self.remote != None:
			print("Successfully opened " + devname)
			return True
		else:
			print("Could not open " + devname)
			return false

		if self.remote == None:
			return False

	def isConnected(self):
		if self.remote == None:
			return False
		else:
			return True

	def dataAvailable(self):
		if self.remote == None:
			print("No remote opened");
			return False
		print("%d in waiting" % self.remote.in_waiting())
		return self.remote.in_waiting() != 0

	def readLine(self):
		if self.remote == None:
			print("No remote opened")
			return ""
		else:
			try:
				return self.remote.read_until()
			except serial.serialutil.SerialException:
				self.remote = None
				self.seqnumData = []
				self.rollData = []
				self.pitchData = []
				self.yawData = []
				return None

	def readAndParseLine(self):
		line = self.readLine()
		if line:
			line = line.decode("utf-8")
			m = re.match("S(\d+) H(-?[0-9\.]+) V(-?[0-9\.]+) R(-?[0-9\.]+) P(-?[0-9\.]+) Y(-?[0-9\.]+) AX(-?[0-9\.]+) AY(-?[0-9\.]+) AZ(-?[0-9\.]+) P1(-?[0-9\.]+) P2(-?[0-9\.]+) BattP2(-?[0-9\.]+) B([0|1])([0|1])([0|1])([0|1])([0|1])([0|1])([0|1])([0|1])", line)
			if m is None:
				print("Could not parse line \"" + line + "\"")
				return False
			seqnum = int(m.groups()[0])
			joyH = int(m.groups()[1])
			joyV = int(m.groups()[2])
			roll = int(m.groups()[3])
			pitch = int(m.groups()[4])
			yaw = int(m.groups()[5])
			ax = int(m.groups()[6])
			ay = int(m.groups()[7])
			az = int(m.groups()[8])
			pot1 = int(m.groups()[9])
			pot2 = int(m.groups()[10])
			batt = int(m.groups()[11])
			buttons = map(bool, m.groups()[12:])

			while len(self.seqnumData) > self.maxData:
				del self.seqnumData[0]
			self.seqnumData.append(seqnum)
			while len(self.joyHData) > self.maxData:
				del self.joyHData[0]
			self.joyHData.append(joyH)
			while len(self.joyVData) > self.maxData:
				del self.joyVData[0]
			self.joyVData.append(joyV)
			while len(self.rollData) > self.maxData:
				del self.rollData[0]
			self.rollData.append(roll)
			while len(self.pitchData) > self.maxData:
				del self.pitchData[0]
			self.pitchData.append(pitch)
			while len(self.yawData) > self.maxData:
				del self.yawData[0]
			self.yawData.append(yaw)
			while len(self.accelXData) > self.maxData:
				del self.accelXData[0]
			self.accelXData.append(ax)
			while len(self.accelYData) > self.maxData:
				del self.accelYData[0]
			self.accelYData.append(ay)
			while len(self.accelZData) > self.maxData:
				del self.accelZData[0]
			self.accelZData.append(az)
			while len(self.pot1Data) > self.maxData:
				del self.pot1Data[0]
			self.pot1Data.append(pot1)
			while len(self.pot2Data) > self.maxData:
				del self.pot2Data[0]
			self.pot2Data.append(pot2)

		return True

	def getGyroData(self):
		return (self.rollData, self.pitchData, self.yawData)

	def getSeqnumData(self):
		return self.seqnumData

	def flush(self):
		if self.remote == None:
			print("No remote opened")
		else:
			self.remote.reset_input_buffer()


if __name__ == "__main__":
	handler = RemoteHandler()
	while True:
		if handler.isConnected() == False:
			if handler.connect() == True:
				print("Connected")
		if handler.isConnected() == True:
			l = handler.readAndParseLine()
			handler.flush()
		time.sleep(.01)
