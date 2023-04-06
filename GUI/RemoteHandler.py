import os
import re
import serial
import time

class RemoteHandler:
	def __init__(self):
		self.remote = None
		self.rollData = []
		self.pitchData = []
		self.yawData = []
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
			m = re.match("S (\d+) G R(-?[0-9\.]+) P(-?[0-9\.]+) Y(-?[0-9\.]+)", line)
			if m is None:
				print("Could not parse line \"" + line + "\"")
				return False
			seqnum = int(m.groups()[0])
			roll = float(m.groups()[1])
			pitch = float(m.groups()[2])
			yaw = float(m.groups()[3])

			while len(self.rollData) > self.maxData:
				del self.rollData[0]
			self.rollData.append(roll)
			while len(self.pitchData) > self.maxData:
				del self.pitchData[0]
			self.pitchData.append(pitch)
			while len(self.yawData) > self.maxData:
				del self.yawData[0]
			self.yawData.append(yaw)
			while len(self.seqnumData) > self.maxData:
				del self.seqnumData[0]
			self.seqnumData.append(seqnum)

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
