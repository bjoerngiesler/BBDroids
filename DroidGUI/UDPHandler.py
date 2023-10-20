import socket

from PacketIndices import *
from LargeStatePacket import LargeStatePacket

STATE_PORTNUM = 3000

class UDPHandler:
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind(('', STATE_PORTNUM))
		self.sock.setblocking(0)
		self.cmdqueue = []
		self.states = {}
		self.address = None
		self.broadcast = False
		self.seqnum = 0
		self.newDroidDiscoveredCB = None

	def setNewDroidDiscoveredCB(self, cb):
		self.newDroidDiscoveredCB = cb

	def addressesDiscovered(self):
		return list(self.states.keys())

	def selectedAddress(self):
		return self.address

	def selectAddress(self, address):
		self.address = address

	def doesUseBroadcast(self):
		return self.broadcast

	def useBroadcast(self, b):
		self.broadcast = b

	def readIfAvailable(self):
		try:
			buf = self.sock.recvfrom(1024)
			address = buf[1][0]
			if address not in self.states.keys() and self.newDroidDiscoveredCB:
				shouldCallCallback = True
			else:
				shouldCallCallback = False
			packet = LargeStatePacket(buf[0])

			self.states[address] = packet
			if shouldCallCallback:
				self.newDroidDiscoveredCB(address)
			return packet
		except BlockingIOError:
			return None

	def removeAndQueueNew(self, key, bytes):
		i = 0
		while i<len(self.cmdqueue):
			if(self.cmdqueue[i][0] == key):
				del self.cmdqueue[i]
			else:
				i = i+1
		self.cmdqueue.append((key, bytes))

	def queueFloatCommand(self, command, floatarg):
		bytes = struct.pack(PACK_FORMAT_FLOATARG, self.seqnum, command, floatarg)
		self.seqnum = (self.seqnum+1)%256
		self.removeAndQueueNew(command, bytes)

	def queueIndexedFloatCommand(self, command, index, floatarg):
		bytes = struct.pack(PACK_FORMAT_INDEXEDFLOATARG, self.seqnum, command, index, floatarg)
		self.seqnum = (self.seqnum + 1)%256
		self.removeAndQueueNew((command, index), bytes)

	def queueFloatListCommand(self, command, float0, float1, float2, float3):
		bytes = struct.pack(PACK_FORMAT_FLOATLISTARG, self.seqnum, command, float0, float1, float2, float3)
		self.seqnum = (self.seqnum + 1)%256
		self.removeAndQueueNew(command, bytes)

	def sendCommandQueue(self):
		for key, bytes in self.cmdqueue:
			self.sock.sendto(bytes, (self.address, CMD_PORTNUM))
		self.cmdqueue = []

	def getFloatVal(self, index, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return -1
		return self.states[address][index]

	def getSeqNum(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return -1
		return self.states[address][0]

	def getIMUsOK(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return False
		return self.states[address][1]

	def getMotorsOK(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return False
		return self.states[address][2]

	def getServosOK(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return False
		return self.states[address][3]

	def getDomeIMUData(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return (0, 0, 0)
		return self.states[address][4], self.states[address][5], self.states[address][6]

	def getBodyIMUData(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return (0, 0, 0)
		return self.states[address][7], self.states[address][8], self.states[address][9]

	def getServoData(self, address = None):
		if address == None:
			address = self.address
		if address not in self.states:
			return (0, 0, 0, 0)
		return self.states[address][10], self.states[address][11], self.states[address][12], self.states[address][13]
