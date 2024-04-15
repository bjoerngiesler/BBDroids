import struct

class DriveControlState:
	PACK_FORMAT = "2B8f"
	def __init__(self, t):
		(self.errorState, self.controlMode, self.presentPWM, self.presentSpeed, self.presentPos,
			self.goal, self.err, self.errI, self.errD, self.control) = t
	@classmethod
	def numValues(cls):
		return 10
class IMUState:
	PACK_FORMAT = "B9f"
	def __init__(self, t):
		(self.errorState, self.r, self.p, self.h, self.dr, self.dp, self.dh, self.ax, self.ay, self.az) = t
	@classmethod
	def numValues(cls):
		return 10
class CommandPacket:
	PACK_FORMAT = "7B"
	def __init__(self, t):
		self.t = t # FIXME later
	@classmethod
	def numValues(cls):
		return 7
class ServoState:
	PACK_FORMAT = "B3f"
	def __init__(self, t):
		(self.errorState, self.goal, self.present, self.load) = t
	@classmethod
	def numValues(cls):
		return 4
class BatteryState:
	PACK_FORMAT = "B2f"
	def __init__(self, t):
		(self.errorState, self.voltage, self.current) = t
		self.current = self.current / 1000
	@classmethod
	def numValues(cls):
		return 3

class LargeStatePacket:
	def __init__(self, packet):
		PACK_FORMAT = ("<fB16s" + 
			3*DriveControlState.PACK_FORMAT + 
			3*IMUState.PACK_FORMAT +  
			2*CommandPacket.PACK_FORMAT + 
			10*ServoState.PACK_FORMAT + 
			3*BatteryState.PACK_FORMAT)
		try:
			t = struct.unpack(PACK_FORMAT, packet)
		except struct.error:
			print("Got a buffer of size %d, expect %d" % (len(packet), struct.calcsize(PACK_FORMAT)))
			return

		self.timestamp, self.droidType, self.droidName = t[0:3]
		i = 3
		self.drive = []
		self.drive.append(DriveControlState(t[i:i+DriveControlState.numValues()]))
		i += DriveControlState.numValues()
		self.drive.append(DriveControlState(t[i:i+DriveControlState.numValues()]))
		i += DriveControlState.numValues()
		self.drive.append(DriveControlState(t[i:i+DriveControlState.numValues()]))
		i += DriveControlState.numValues()
		self.imu = []
		self.imu.append(IMUState(t[i:i+IMUState.numValues()]))
		i += IMUState.numValues()
		self.imu.append(IMUState(t[i:i+IMUState.numValues()]))
		i += IMUState.numValues()
		self.imu.append(IMUState(t[i:i+IMUState.numValues()]))
		i += IMUState.numValues()
		self.cmd = []
		self.cmd.append(CommandPacket(t[i:i+CommandPacket.numValues()]))
		i += CommandPacket.numValues()
		self.cmd.append(CommandPacket(t[i:i+CommandPacket.numValues()]))
		i += CommandPacket.numValues()
		self.servos = []
		for j in range(10):
			self.servos.append(ServoState(t[i:i+ServoState.numValues()]))
			i += ServoState.numValues()		
		self.batt = []
		self.batt.append(BatteryState(t[i:i+BatteryState.numValues()]))
		i += BatteryState.numValues()		
		self.batt.append(BatteryState(t[i:i+BatteryState.numValues()]))
		i += BatteryState.numValues()		
		self.batt.append(BatteryState(t[i:i+BatteryState.numValues()]))
