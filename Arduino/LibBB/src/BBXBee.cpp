#include <limits.h> // for ULONG_MAX
#include <inttypes.h> // for uint64_t format string
#include <vector>

#include "BBXBee.h"
#include "BBError.h"
#include "BBConsole.h"
#include "BBRunloop.h"

bb::XBee bb::XBee::xbee;

static std::vector<unsigned int> baudRatesToTry = { 115200, 9600, 57600, 19200, 28800, 38400, 76800 }; // start with 115200, then try 9600

bb::XBee::XBee() {
	uart_ = &Serial1;
	debug_ = (XBee::DebugFlags)(DEBUG_PROTOCOL);
	timeout_ = 1000;
	atmode_ = false;
	atmode_millis_ = 0;
	atmode_timeout_ = 10000;
	currentBPS_ = 0;
	memset(packetBuf_, 0, sizeof(packetBuf_));
	packetBufPos_ = 0;
	apiMode_ = false;

	name_ = "xbee";
	description_ = "Communication via XBee 802.5.14";
	help_ = "In order for communication to work, the PAN and channel numbers must be identical.\r\n" \
	"Available commands:\r\n" \
	"\tpacket_mode on|off: Switch to packet mode\r\n" \
	"\tapi_mode on|off: Enter / leave API mode\r\n" \
	"\tsend_api_packet <dest>: Send zero control packet to destination\r\n";

	addParameter("channel", "Communication channel (between 11 and 26, usually 12)", params_.chan, 11, 26);
	addParameter("pan", "Personal Area Network ID (16bit, 65535 is broadcast)", params_.pan, 0, 65535);
	addParameter("station", "Station ID (MY) for this device (16bit)", params_.station, 0, 65535);
	addParameter("bps", "Communication bps rate", params_.bps, 0, 200000);
}

bb::XBee::~XBee() {
}

bb::Result bb::XBee::initialize(uint8_t chan, uint16_t pan, uint16_t station, uint32_t bps, HardwareSerial *uart) {
	if(operationStatus_ != RES_SUBSYS_NOT_INITIALIZED) return RES_SUBSYS_ALREADY_INITIALIZED;

	paramsHandle_ = ConfigStorage::storage.reserveBlock("xbee", sizeof(params_), (uint8_t*)&params_);
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
		Console::console.printfBroadcast("XBee: Storage block is valid\n");
		ConfigStorage::storage.readBlock(paramsHandle_);
	} else {
		Console::console.printfBroadcast("XBee: Storage block is invalid, using passed parameters\n");
		memset(&params_, 0, sizeof(params_));
		params_.chan = chan;
		params_.pan = pan;
		params_.station = station;
		params_.bps = bps;
	}

	uart_ = uart;

	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	return Subsystem::initialize();	
}

bb::Result bb::XBee::start(ConsoleStream *stream) {
	if(isStarted()) return RES_SUBSYS_ALREADY_STARTED;
	if(NULL == uart_) return RES_SUBSYS_HW_DEPENDENCY_MISSING;

	// empty uart
	while(uart_->available()) uart_->read();

	if(stream == NULL) stream = Console::console.serialStream();

	currentBPS_ = 0;
	if(currentBPS_ == 0) {
		if(stream) stream->printf("auto-detecting BPS... ");
		
		for(size_t i=0; i<baudRatesToTry.size(); i++) {
			if(stream) {
				stream->printf("%d...", baudRatesToTry[i]); 
			}
			
			uart_->end();
			delay(100);
			uart_->begin(baudRatesToTry[i]);
			
			if(enterATModeIfNecessary() == RES_OK) {
				Console::console.printfBroadcast("Success.\n");
				currentBPS_ = baudRatesToTry[i];
				break; 
			}
		}
		if(currentBPS_ == 0) {
			if(stream) stream->printf("failed.\n");
			operationStatus_ = RES_SUBSYS_HW_DEPENDENCY_MISSING;
			return RES_SUBSYS_HW_DEPENDENCY_MISSING;
		}
	} else {
		if(stream) stream->printf("Using %dbps.", currentBPS_);
		uart_->begin(currentBPS_);
		if(enterATModeIfNecessary() != RES_OK) {
			operationStatus_ = RES_SUBSYS_HW_DEPENDENCY_MISSING;
			return RES_SUBSYS_HW_DEPENDENCY_MISSING;
		}
	}

	String addrH = sendStringAndWaitForResponse("ATSH"); addrH.trim();
	String addrL = sendStringAndWaitForResponse("ATSL"); addrL.trim();
	String fw = sendStringAndWaitForResponse("ATVR"); fw.trim();
	if(addrH == "" || addrL == "" || fw == "") return RES_SUBSYS_COMM_ERROR;

	hwAddress_ = {uint32_t(strtol(addrH.c_str(), 0, 16)), uint32_t(strtol(addrL.c_str(), 0, 16))};
	if(stream) {
		stream->printf("Found XBee at address: %s:%s (0x%lx:%lx) Firmware version: %s\n", addrH.c_str(), addrL.c_str(), hwAddress_.addrHi, hwAddress_.addrLo, fw.c_str());
	} else {
		Console::console.printfBroadcast("Found XBee at address: %s:%s (0x%lx:%lx) Firmware version: %s\n", addrH.c_str(), addrL.c_str(), hwAddress_.addrHi, hwAddress_.addrLo, fw.c_str());
	}	

	String retval = sendStringAndWaitForResponse("ATCT"); 
	if(retval != "") {
		atmode_timeout_ = strtol(retval.c_str(), 0, 16) * 100;
	}

	retval = sendStringAndWaitForResponse("ATMM");
	if(retval != "") {
		Console::console.printfBroadcast("MM response: %s\n", retval.c_str());
	}
	retval = sendStringAndWaitForResponse("ATMM=3");
	if(retval != "") {
		Console::console.printfBroadcast("MM=3 response: %s\n", retval.c_str());
	}
	retval = sendStringAndWaitForResponse("ATRR");
	if(retval != "") {
		Console::console.printfBroadcast("RR response: %s\n", retval.c_str());
	}

	if(setConnectionInfo(params_.chan, params_.pan, params_.station, true) != RES_OK) {
		if(stream) stream->printf("Setting connection info failed.\n");
		return RES_SUBSYS_COMM_ERROR;
	}

	uint8_t chan; uint16_t pan; uint16_t station; 
	if(getConnectionInfo(chan, pan, station, true) != RES_OK) {
		if(stream) stream->printf("Getting connection info failed.\n");
		return RES_SUBSYS_COMM_ERROR;
	}
	params_.chan = chan;
	params_.pan = pan;
	params_.station = station;

	if(strlen(params_.name) != 0) {
		String str = String("ATNI") + params_.name;
		if(sendStringAndWaitForOK(str) == false) {
			Console::console.printfBroadcast("Error setting name \"%s\"\n", params_.name);
			return RES_SUBSYS_COMM_ERROR;
		}
	} else {
		Console::console.printfBroadcast("Zero-length name, not setting\n");
	} 

	if(sendStringAndWaitForOK("ATNT=64") == false) {
		Console::console.printfBroadcast("Error setting node discovery timeout\n");
		return RES_SUBSYS_COMM_ERROR;
	} 

	bool changedBPS = false;

	if(params_.bps != currentBPS_) {
		Console::console.printfBroadcast("Changing BPS from current %d to %d...\n", currentBPS_, params_.bps);
		if(changeBPSTo(params_.bps, stream, true) != RES_OK) {
			if(stream) stream->printf("Setting BPS failed.\n"); 
			else Console::console.printfBroadcast("Setting BPS failed.\n"); 
			currentBPS_ = 0;
			return RES_SUBSYS_COMM_ERROR;
		} else {
			Console::console.printfBroadcast("Setting BPS to %d successful.\n", params_.bps); 
			changedBPS = true;
		}
	}

	if(sendStringAndWaitForOK("ATWR") == false) {
		if(stream) stream->printf("Couldn't write config!\n");
		return RES_SUBSYS_COMM_ERROR;
	}

	// we have changed the BPS successfully?
	if(changedBPS) {
		if(stream) stream->printf("Closing and reopening serial at %dbps\n", params_.bps);
		leaveATMode();
		uart_->end();
		uart_->begin(params_.bps);
		enterATModeIfNecessary();
		if(sendStringAndWaitForOK("AT") == false) return RES_SUBSYS_COMM_ERROR;
		currentBPS_ = params_.bps;
	}

	setAPIMode(true);

	leaveATMode();

	operationStatus_ = RES_OK;
	started_ = true;

	return RES_OK;
}

bb::Result bb::XBee::stop(ConsoleStream *stream) {
	if(stream) stream = stream; // make compiler happy
	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	currentBPS_ = 0;
	started_ = false;
	return RES_OK;
}

bb::Result bb::XBee::step() {
	int packetsHandled = 0;
	while(available()) {
		if(apiMode_) {
			HWAddress srcAddr;
			uint8_t rssi;
			Packet packet;
			Result retval = receiveAPIMode(srcAddr, rssi, packet);
			if(retval != RES_OK) {
				//Console::console.printfBroadcast("receiveAPIMode(): %s\n", errorMessage(retval));
				continue;
			}
			//Console::console.printfBroadcast("Received packet from %lx:%lx type %d\n", srcAddr.addrHi, srcAddr.addrLo, packet.type);
			for(auto& r: receivers_) {
				r->incomingPacket(srcAddr, rssi, packet);
			}
			packetsHandled++;
		} else {
			bb::Console::console.printfBroadcast("%s\n", receive().c_str());
		}
	}

	return RES_OK;
}

bb::Result bb::XBee::parameterValue(const String& name, String& value) {
	if(name == "channel") { 
		value = String(params_.chan); return RES_OK; 
	} else if(name == "pan") {
		value = String(params_.pan); return RES_OK;
	} else if(name == "station") {
		value = String(params_.station); return RES_OK;
	} else if(name == "bps") {
		value = String(params_.bps); return RES_OK;
	} 

	return RES_PARAM_NO_SUCH_PARAMETER;
}

bb::Result bb::XBee::setParameterValue(const String& name, const String& value) {
	Result res = RES_PARAM_NO_SUCH_PARAMETER;

	if(name == "channel") { 
		params_.chan = value.toInt();
		res = setConnectionInfo(params_.chan, params_.pan, params_.station, false);
	} else if(name == "pan") {
		params_.pan = value.toInt();
		res = setConnectionInfo(params_.chan, params_.pan, params_.station, false);
	} else if(name == "station") {
		params_.station = value.toInt();
		res = setConnectionInfo(params_.chan, params_.pan, params_.station, false);
	} else if(name == "bps") {
		params_.bps = value.toInt();
		res = RES_OK;
	}

	if(res == RES_OK) {
		ConfigStorage::storage.writeBlock(paramsHandle_);
	}

	return res;
}

bb::Result bb::XBee::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
	if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
	
	if(words[0] == "send") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		return send(words[1]);
	} 

	else if(words[0] == "api_mode") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		else {
			if(words[1] == "on" || words[1] == "true") {
				return setAPIMode(true);
			}
			else if(words[1] == "off" || words[1] == "false") {
				return setAPIMode(false);
			}
			else return RES_CMD_INVALID_ARGUMENT;	
		}		
	}

	return bb::Subsystem::handleConsoleCommand(words, stream);
}

bb::Result bb::XBee::setAPIMode(bool onoff) {
	Result res;
	if(onoff == true) {
		if(apiMode_ == true) {
			return RES_CMD_INVALID_ARGUMENT;
		}

		enterATModeIfNecessary();
		if(sendStringAndWaitForOK("ATAP=2") == true) {
			apiMode_ = true;
			leaveATMode();
			return RES_OK;
		} else {
			leaveATMode();
			return RES_SUBSYS_COMM_ERROR;
		}
	} else {
		if(apiMode_ == false) {
			Console::console.printfBroadcast("Not in API mode\n");
			return RES_CMD_INVALID_ARGUMENT;			
		}

		uint32_t arg = 0;
		res = sendAPIModeATCommand(1, "AP", arg);
		if(res == RES_OK) apiMode_ = false;
		return res;
	}
}


bb::Result bb::XBee::addPacketReceiver(PacketReceiver *receiver) {
	for(size_t i=0; i<receivers_.size(); i++)
		if(receivers_[i] == receiver)
			return RES_COMMON_DUPLICATE_IN_LIST;
	receivers_.push_back(receiver);
	return RES_OK;
}

bb::Result bb::XBee::removePacketReceiver(PacketReceiver *receiver) {
	for(size_t i=0; i<receivers_.size(); i++)
		if(receivers_[i] == receiver) {
			receivers_.erase(receivers_.begin()+i);
			return RES_OK;
		}
	return RES_COMMON_NOT_IN_LIST;
}


bb::Result bb::XBee::enterATModeIfNecessary(ConsoleStream *stream) {
	debug_ = DEBUG_PROTOCOL;
	if(isInATMode()) {
		return RES_OK;
	}

	Console::console.printfBroadcast("Entering AT mode.\n");
	//Console::console.printfBroadcast("Wait 1s... ");

	int numDiscardedBytes = 0;

	for(int timeout = 0; timeout < 1000; timeout++) {
		if(uart_->available()) {
			Console::console.printfBroadcast("Discarding ");
			while(uart_->available())  {
				uart_->read();
				Console::console.printfBroadcast(".");
				numDiscardedBytes++;
			}
			Console::console.printfBroadcast("\n");
		}
		delay(1);
	}

	uart_->write("+++");
	delay(1000);

	bool success = false;

	for(int timeout = 0; timeout < 1200 && !success; timeout++) {
		unsigned char c;

		while(uart_->available()) {
			c = uart_->read();
			if(c == 'O') {
				if(!uart_->available()) delay(1);
				if(!uart_->available()) continue;
				c = uart_->read();
				if(c == 'K') {
					if(!uart_->available()) delay(1);
					if(!uart_->available()) continue;
					c = uart_->read();
					if(c == '\r') {
						success = true;
						break;
					} else {
						numDiscardedBytes++;
					}
				} else {
					numDiscardedBytes++;
				}
			} else {
				numDiscardedBytes++;
			}
		}

		if(!success) delay(1);
	}


	if(success) {
		Console::console.printfBroadcast("Successfully entered AT Mode\n");
		if(numDiscardedBytes) {
			Console::console.printfBroadcast("Discarded %d bytes while entering AT mode.\n", numDiscardedBytes);
		}
		atmode_millis_ = millis();
		atmode_ = true;
				
		return RES_OK;
	}

	Console::console.printfBroadcast("no response to +++\n");
	return RES_COMM_TIMEOUT;
}

bool bb::XBee::isInATMode() {
	if(atmode_ == false) return false;
	unsigned long m = millis();
	if(atmode_millis_ < m) {
		if(m - atmode_millis_ < atmode_timeout_) return true;
	} else {
		if(ULONG_MAX - m + atmode_millis_ < atmode_timeout_) return true;
	}
	atmode_ = false;
	return false;
}

bb::Result bb::XBee::leaveATMode(ConsoleStream *stream) {
	if(!isInATMode()) {
		return RES_OK;
	}
	if((debug_ & DEBUG_PROTOCOL) && stream!=NULL) stream->printf("Sending ATCN to leave AT Mode\n");
	if(sendStringAndWaitForOK("ATCN") == true) {
		if((debug_ & DEBUG_PROTOCOL) && stream!=NULL) stream->printf("Successfully left AT Mode\n");
		atmode_ = false;
		delay(1100);
		return RES_OK;
	}

	if((debug_ & DEBUG_PROTOCOL) && stream!=NULL) stream->printf("no response to ATCN\n");
	return RES_COMM_TIMEOUT;
}

bb::Result bb::XBee::changeBPSTo(uint32_t bps, ConsoleStream *stream, bool stayInAT) {
	uint32_t paramVal;
	switch(bps) {
	case 1200:   paramVal = 0x0; break;
	case 2400:   paramVal = 0x1; break;
	case 4800:   paramVal = 0x2; break;
	case 9600:   paramVal = 0x3; break;
	case 19200:  paramVal = 0x4; break;
	case 38400:  paramVal = 0x5; break;
	case 57600:  paramVal = 0x6; break;
	case 115200: paramVal = 0x7; break;
	case 230400: paramVal = 0x8; break;
	case 460800: paramVal = 0x9; break;
	case 921600: paramVal = 0xa; break;
	default:     paramVal = bps; break;
	}

#if 0
	String retval = sendStringAndWaitForResponse("ATBD");

	if((unsigned)retval.toInt() == paramVal) {
		if(stream) stream->printf("XBee says it's already running at %d bps\n", bps);
		return RES_OK;
	}
#endif

	if(stream) stream->printf("Sending ATBD=%x\n", paramVal);
	if(sendStringAndWaitForOK(String("ATBD=")+String(paramVal, HEX)) == false) return RES_SUBSYS_COMM_ERROR;

	if(stayInAT == false) leaveATMode();
	currentBPS_ = bps;

	return RES_OK;
}

bb::Result bb::XBee::setConnectionInfo(uint8_t chan, uint16_t pan, uint16_t station, bool stayInAT) {
	params_.chan = chan;
	params_.pan = pan;
	params_.station = station;

	enterATModeIfNecessary();

	if(sendStringAndWaitForOK(String("ATCH=")+String(chan, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printfBroadcast("ERROR: Setting channel to 0x%x failed!\n", chan);
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	} 
	if(sendStringAndWaitForOK(String("ATID=")+String(pan, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printfBroadcast("ERROR: Setting PAN to 0x%x failed!\n", pan);
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	}
	if(sendStringAndWaitForOK(String("ATMY=")+String(station, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printfBroadcast("ERROR: Setting MY to 0x%x failed!\n", station);
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	}

	if(!stayInAT) leaveATMode();

	return RES_OK;
}

bb::Result bb::XBee::getConnectionInfo(uint8_t& chan, uint16_t& pan, uint16_t& station, bool stayInAT) {
	enterATModeIfNecessary();

	String retval;

	retval = sendStringAndWaitForResponse("ATCH");
	if(retval == "") return RES_COMM_TIMEOUT;
	chan = strtol(retval.c_str(), 0, 16);
	delay(1);

	retval = sendStringAndWaitForResponse("ATID");
	if(retval == "") return RES_COMM_TIMEOUT;
	pan = strtol(retval.c_str(), 0, 16);
	delay(1);

	retval = sendStringAndWaitForResponse("ATMY");
	if(retval == "") return RES_COMM_TIMEOUT;
	station = strtol(retval.c_str(), 0, 16);
	delay(1);

	if(!stayInAT) leaveATMode();
	return RES_OK;
}

bb::Result bb::XBee::discoverNodes(std::vector<bb::XBee::Node>& nodes) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(apiMode_ == false) return RES_SUBSYS_WRONG_MODE;

	nodes = std::vector<bb::XBee::Node>();

	Result res;
	APIFrame request = APIFrame::atRequest(0x5, ('N'<<8 | 'D'));
	res = send(request);
	if(res != RES_OK) return res;

	int timeout = 10000;

	while(timeout > 0) {
		delay(1);
		timeout--;

		if(available()) {
			Result res;
			APIFrame response;
			res = receive(response);
			if(res != RES_OK) {
				Console::console.printfBroadcast("Error receiving response: %s\n", errorMessage(res));
				continue;
			}

			uint8_t frameID, status;
			uint16_t command, length;
			uint8_t *data;

			res = response.unpackATResponse(frameID, command, status, &data, length);
			if(res != RES_OK) {
				continue;
			}

			if(status != 0) {
				Console::console.printfBroadcast("Response with status %d\"", status);
				continue;
			}	

			if(length < APIFrame::ATResponseNDMinLength) {
				Console::console.printfBroadcast("Expected >=%d bytes, found %d\n", APIFrame::ATResponseNDMinLength, length);
				continue;
			}

			APIFrame::ATResponseND *r = (APIFrame::ATResponseND*)data;
			Node n;
			n.stationId = r->my;
			n.address = {r->addrHi, r->addrLo};
			n.rssi = r->rssi;
			memset(n.name, 0, sizeof(n.name));
			if(length > APIFrame::ATResponseNDMinLength) {
				strncpy(n.name, r->name, strlen(r->name));
			}

			Console::console.printfBroadcast("Discovered station 0x%x at address 0x%lx:%lx, RSSI %d, name \"%s\"\n", n.stationId, n.address.addrHi, n.address.addrLo, n.rssi, n.name);
			nodes.push_back(n);	
		}
	}

	return RES_OK;
}

void bb::XBee::setDebugFlags(DebugFlags debug) {
	debug_ = debug;
}

bb::Result bb::XBee::send(const String& str) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(isInATMode()) leaveATMode();
	uart_->write(str.c_str(), str.length());
	return RES_OK;
}

bb::Result bb::XBee::send(const uint8_t *bytes, size_t size) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(isInATMode()) leaveATMode();
	uart_->write(bytes, size);
	return RES_OK;
}

bb::Result bb::XBee::send(const bb::Packet& packet) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(isInATMode()) leaveATMode();

	bb::PacketFrame frame;
	frame.packet = packet;

	uint8_t *buf = (uint8_t*)&frame.packet;
	for(size_t i=0; i<sizeof(frame.packet)-1; i++) {
		if(buf[i] & 0x80) {
			Console::console.printfBroadcast("ERROR: Byte %d of packet has highbit set! Not sending.\n", i);
			return RES_PACKET_INVALID_PACKET;
		}
	}

	frame.packet.seqnum = Runloop::runloop.getSequenceNumber() % MAX_SEQUENCE_NUMBER;
	frame.crc = frame.packet.calculateCRC();

	uart_->write((uint8_t*)&frame, sizeof(frame));

	return RES_OK;
}

bb::Result bb::XBee::sendToXBee3(const HWAddress& dest, const bb::Packet& packet, bool ack) {
	uint8_t buf[14+sizeof(packet)];

	buf[0] = 0x10; // transmit request
	buf[1] = 0x0;  // no response frame
	buf[2] = (dest.addrHi >> 24) & 0xff;
	buf[3] = (dest.addrHi >> 16) & 0xff;
	buf[4] = (dest.addrHi >> 8) & 0xff;
	buf[5] = dest.addrHi & 0xff;
	buf[6] = (dest.addrLo >> 24) & 0xff;
	buf[7] = (dest.addrLo >> 16) & 0xff;
	buf[8] = (dest.addrLo >> 8) & 0xff;
	buf[9] = dest.addrLo & 0xff;
	//Console::console.printfBroadcast("Sending to 0x%lx:%lx - ", dest.addrHi, dest.addrLo);
	//for(int i=2; i<=9; i++) Console::console.printfBroadcast("%02x", buf[i]);
	//Console::console.printfBroadcast("\n");
	buf[10] = 0xff;
	buf[11] = 0xfe;
	buf[12] = 0; 							// broadcast radius - unused
	if(ack == false) {
		buf[13] = 1;						// disable ACK
	} else {
		buf[13] = 0;						// Use default value of TO
	}

	memcpy(&(buf[14]), &packet, sizeof(packet));
	
	APIFrame frame(buf, 14+sizeof(packet));
	return send(frame);
}

bb::Result bb::XBee::sendToXBee(const HWAddress& dest, const bb::Packet& packet, bool ack) {
	uint8_t buf[11+sizeof(packet)];

	buf[0] = 0x0;  // transmit request - 64bit frame. This is deprecated.
	buf[1] = 0x0;  // no response frame
	buf[2] = (dest.addrHi >> 24) & 0xff;
	buf[3] = (dest.addrHi >> 16) & 0xff;
	buf[4] = (dest.addrHi >> 8) & 0xff;
	buf[5] = dest.addrHi & 0xff;
	buf[6] = (dest.addrLo >> 24) & 0xff;
	buf[7] = (dest.addrLo >> 16) & 0xff;
	buf[8] = (dest.addrLo >> 8) & 0xff;
	buf[9] = dest.addrLo & 0xff;
#if 0
	Console::console.printfBroadcast("Sending with 0x%x to 0x%lx:%lx - ", buf[0], dest.addrHi, dest.addrLo);
	for(int i=2; i<=9; i++) Console::console.printfBroadcast("%02x", buf[i]);
	Console::console.printfBroadcast("\n");
#endif
	if(ack == false) {
		buf[10] = 1;						// disable ACK
	} else {
		buf[10] = 0;						// Use default value of TO
	}

	memcpy(&(buf[11]), &packet, sizeof(packet));
	
	APIFrame frame(buf, 11+sizeof(packet));
	return send(frame);
}

bb::Result bb::XBee::sendConfigPacket(const HWAddress& dest,  
                                      bb::PacketSource src, const ConfigPacket& cfg, ConfigPacket::ConfigReplyType& replyType,
									  uint8_t seqnum, bool waitForReply) {
	bb::Packet sPacket(bb::PACKET_TYPE_CONFIG, src, seqnum);
	sPacket.payload.config = cfg;
	if(waitForReply == false) sPacket.payload.config.reply = ConfigPacket::CONFIG_TRANSMIT_NOREPLY;
	else sPacket.payload.config.reply = ConfigPacket::CONFIG_TRANSMIT_REPLY;

	Result res = sendTo(dest, sPacket, false);
	if(res != RES_OK) {
		Console::console.printfBroadcast("sendConfigPacket(): sendTo(): %s\n", errorMessage(res));
		return res;
	}

	if(waitForReply == false) return res;

	int timeout = 500;
	while(true) {
		while(uart_->available() == false) {
			timeout--;
			delay(1);
			if(timeout < 0) {
				Console::console.printfBroadcast("Timeout.\n");
				return RES_COMM_TIMEOUT;
			}
		}
		HWAddress srcAddr;
		uint8_t rssi;
		Packet rPacket;
		Result res = receiveAPIMode(srcAddr, rssi, rPacket);
		if(res != RES_OK) {
			Console::console.printfBroadcast("sendConfigPacket(): receiveAPIMode(): %s\n", errorMessage(res));
			return res;
		}

		if(rPacket.type != PACKET_TYPE_CONFIG) {
			Console::console.printfBroadcast("sendConfigPacket(): Discarding packet of type %d while waiting for reply\n", rPacket.type);
			continue;
		}
		if(rPacket.seqnum != sPacket.seqnum) {
			Console::console.printfBroadcast("sendConfigPacket(): Discarding packet with seqnum %d while waiting for %d\n", rPacket.seqnum, sPacket.seqnum);
			continue;
		}
		if(rPacket.payload.config.type != sPacket.payload.config.type) {
			Console::console.printfBroadcast("sendConfigPacket(): Discarding packet with type %d while waiting for %d\n", 
			rPacket.payload.config.type, sPacket.payload.config.type);
			continue;
		}

		replyType = rPacket.payload.config.reply;
		return RES_OK;
	}
}

int bb::XBee::numFailedACKs() {
	uint32_t failedAcks;

	if(sendAPIModeATCommand(0x17, "EA", failedAcks, true) != RES_OK) return -1;
	return failedAcks;
}


bool bb::XBee::available() {
	if(operationStatus_ != RES_OK) return false;
	if(isInATMode()) leaveATMode();
	return uart_->available();
}

String bb::XBee::receive() {
	if(operationStatus_ != RES_OK) return "";
	if(isInATMode()) leaveATMode();

	String retval;
	while(uart_->available()) {
		retval += (char)uart_->read();
	}
	return retval;
}

bb::Result bb::XBee::receiveAPIMode(HWAddress& srcAddr, uint8_t& rssi, Packet& packet) {
	if(!apiMode_) {
		bb::Console::console.printfBroadcast("Wrong mode.\n");
		return RES_SUBSYS_WRONG_MODE;
	} 

	APIFrame frame;
	Result retval;

	retval = receive(frame);
	if(retval != RES_OK) return retval;

	//Console::console.printfBroadcast("Received frame of length %d, first char 0x%x\n", frame.length(), frame.data()[0]);

	if(frame.is16BitRXPacket()) { // 16bit address frame
		if(frame.length() != sizeof(bb::Packet) + 5) {
			Console::console.printfBroadcast("Invalid API Mode 16bit addr packet size %d (expected %d)\n", frame.length(), sizeof(bb::Packet) + 5);
			return RES_SUBSYS_COMM_ERROR;
		}
		srcAddr = {0, uint32_t(frame.data()[1] << 8) | frame.data()[2]};
		rssi = frame.data()[3];
		memcpy(&packet, &(frame.data()[5]), sizeof(packet));
	} else if(frame.is64BitRXPacket()) { // 64bit address frame
		if(frame.length() != sizeof(bb::Packet) + 11) {
			Console::console.printfBroadcast("Invalid API Mode 64bit addr packet size %d (expected %d)\n", frame.length(), sizeof(bb::Packet) + 11);
			return RES_SUBSYS_COMM_ERROR;
		}
		srcAddr.addrHi = (uint32_t(frame.data()[1]) << 24) | (uint32_t(frame.data()[2]) << 16) |
				         (uint32_t(frame.data()[3]) <<  8) | uint32_t(frame.data()[4]);
		srcAddr.addrLo = (uint32_t(frame.data()[5]) << 24) | (uint32_t(frame.data()[6]) << 16) |
				         (uint32_t(frame.data()[7]) <<  8) | uint32_t(frame.data()[8]);
		rssi = frame.data()[9];
		memcpy(&packet, &(frame.data()[11]), sizeof(packet));
#if 0
		Console::console.printfBroadcast("Source addr: 0x%0lx:%0lx \n", srcAddr.addrHi, srcAddr.addrLo);
		Console::console.printfBroadcast("Source: %d Type: %d Seqnum: %d\n", packet.source, packet.type, packet.seqnum);
#endif
	} else {
		Console::console.printfBroadcast("Unknown frame type 0x%x\n", frame.data()[0]);
		return RES_SUBSYS_COMM_ERROR;
	}

	return RES_OK;
}

String bb::XBee::sendStringAndWaitForResponse(const String& str, int predelay, bool cr) {
  	if(debug_ & DEBUG_XBEE_COMM) {
    	Console::console.printfBroadcast("Sending \"%s\"...", str.c_str());
  	}

    uart_->print(str);
  	if(cr) {
  		uart_->print("\r");
  	}


    if(predelay > 0) delay(predelay);

    String retval;
    if(readString(retval)) {
    	if(debug_ & DEBUG_XBEE_COMM) {
    		Console::console.printfBroadcast(retval.c_str());
    		Console::console.printfBroadcast(" ");
    	}
    	return retval;
    }

    if(debug_ & DEBUG_PROTOCOL) {
    	Console::console.printfBroadcast("Nothing.\n");
    }
    return "";
}
  
bool bb::XBee::sendStringAndWaitForOK(const String& str, int predelay, bool cr) {
	Console::console.printfBroadcast("Sending \"%s\"\n", str.c_str());
  	String result = sendStringAndWaitForResponse(str, predelay, cr);
  	if(result.equals("OK\r")) return true;
      
  	if(debug_ & DEBUG_PROTOCOL) {
    	Console::console.printfBroadcast("Expected \"OK\", got \"%s\"... \n", result.c_str());
  	}
  return false;
}

bool bb::XBee::readString(String& str, unsigned char terminator) {
	while(true) {
		int to = 0;
		while(!uart_->available()) {
			delay(1);
			to++;
			if(debug_ & DEBUG_XBEE_COMM) Console::console.printfBroadcast(".");
			if(to >= timeout_) {
				if(debug_ & DEBUG_PROTOCOL) Console::console.printfBroadcast("Timeout!\n");
				return false;
			}
		}
		unsigned char c = (unsigned char)uart_->read();
		str += (char)c;
		if(c == terminator) return true;
	}
}

bb::Result bb::XBee::sendAPIModeATCommand(uint8_t frameID, const char* cmd, uint32_t& argument, bool request) {
	if(apiMode_ == false) return RES_CMD_INVALID_ARGUMENT;
	if(strlen(cmd) != 2) return RES_CMD_INVALID_ARGUMENT;

	uint8_t len = request ? 4 : 5;
	uint8_t buf[5];
	buf[0] = 0x08; // AT command
	buf[1] = frameID;
	buf[2] = cmd[0];
	buf[3] = cmd[1];
	if(!request)
		buf[4] = uint8_t(argument);

	APIFrame frame(buf, len);
	
	if(send(frame) != RES_OK) return RES_SUBSYS_COMM_ERROR;

	bool received = false;
	for(int i=0; i<10 && received == false; i++) {
		if(uart_->available()) {
			if(receive(frame) != RES_OK) return RES_SUBSYS_COMM_ERROR;
			else if(!frame.isATResponse()) {
				Console::console.printfBroadcast("Ignoring frame of type 0x%x while waiting for AT response\n", frame.data()[0]);
				continue;
			} 
			received = true;
		} else delay(1);
	}
	if(!received) {
		Console::console.printfBroadcast("Timed out waiting for frame reply\n");
		return RES_SUBSYS_COMM_ERROR;
	}

	uint16_t length = frame.length();
	const uint8_t *data = frame.data();

	Console::console.printfBroadcast("API reply (%d bytes): ", length);
	for(unsigned int i=0; i<length; i++) Console::console.printfBroadcast("%x ", data[i]);
	Console::console.printfBroadcast("\n");

	if(length < 5 || (request == true && length < 6)) return RES_SUBSYS_COMM_ERROR;
	if(data[0] != 0x88 || data[1] != frameID || data[2] != cmd[0] || data[3] != cmd[1]) return RES_SUBSYS_COMM_ERROR;

	Result res = RES_OK;
	switch(data[4]) {
	case 0:
		Console::console.printfBroadcast("API Response: OK.\n");
		break;
	case 1:
		Console::console.printfBroadcast("API Response: ERROR.\n");
		res=RES_SUBSYS_COMM_ERROR;
		break;
	case 2:
		Console::console.printfBroadcast("API Response: Invalid Command.\n");
		res=RES_SUBSYS_COMM_ERROR;
		break;
	case 3:
		Console::console.printfBroadcast("API Response: Invalid Parameter.\n");
		res=RES_SUBSYS_COMM_ERROR;
		break;
	default:
		Console::console.printfBroadcast("API Response: Unknown error.\n");
		res=RES_SUBSYS_COMM_ERROR;
		break;
	}

	if(request) {
		argument = 0;
		for(unsigned int i=5; i<5+sizeof(argument)&&i<length; i++) {
			argument <<= 8;
			argument |= data[i];
		}
	}
	return res;
}

static uint32_t total = 0;

//#define MEMDEBUG

static uint8_t *allocBlock(uint32_t size, const char *loc="unknown") {
#if defined(MEMDEBUG)
	bb::Console::console.printfBroadcast("Allocing block size %d from \"%s\"...", size, loc);
#endif
	uint8_t *block = new uint8_t[size];
	total += size;
#if defined(MEMDEBUG)
	bb::Console::console.printfBroadcast("result: 0x%x, total %d.\n", block, total);
#endif
	return block;
}

static void freeBlock(uint8_t *block, uint32_t size, const char *loc="unknown") {
	delete block;
	total -= size;
#if defined(MEMDEBUG)
	bb::Console::console.printfBroadcast("Deleted block 0x%x, size %d, total %d from \"%s\"\n", block, size, total, loc);
#endif
}

bb::XBee::APIFrame::APIFrame() {
	data_ = NULL;
	length_ = 0;
	calcChecksum();
}

bb::XBee::APIFrame::APIFrame(const uint8_t *data, uint16_t length) {
	//data_ = new uint8_t[length];
	data_ = allocBlock(length, "APIFrame(const uint8_t*,uint16_t)");
	length_ = length;
	memcpy(data_, data, length);
	calcChecksum();
}

bb::XBee::APIFrame::APIFrame(const bb::XBee::APIFrame& other) {
	//Console::console.printfBroadcast("APIFrame 2\n");
	if(other.data_ != NULL) {
		length_ = other.length_;
		data_ = allocBlock(length_, "APIFrame(const APIFrame&)");
//		data_ = new uint8_t[length_];
		memcpy(data_, other.data_, length_);
	} else {
		data_ = NULL;
		length_ = 0;
	}
	checksum_ = other.checksum_;
}

bb::XBee::APIFrame::APIFrame(uint16_t length) {
	//Console::console.printfBroadcast("APIFrame 1\n");
	//data_ = new uint8_t[length];
	data_ = allocBlock(length, "APIFrame(uint16_t)");
	length_ = length;
}

bb::XBee::APIFrame& bb::XBee::APIFrame::operator=(const APIFrame& other) {
	//Console::console.printfBroadcast("operator=\n");
	if(other.data_ == NULL) {           	// they don't have data
		if(data_ != NULL) {
			freeBlock(data_, length_, "operator=(const APIFrame&) 1");
			//delete data_; 	// ...but we do - delete
		}
		data_ = NULL;						// now we don't have data either
		length_ = 0;
	} else {  								// they do have data
		if(data_ != NULL) {					// and so do we
			if(length_ != other.length_) {	// but it's of a different size
				//delete data_;				// reallocate
				freeBlock(data_, length_, "operator=(const APIFrame&) 2");
				length_ = other.length_;
				//data_ = new uint8_t[length_];
				data_ = allocBlock(length_, "operator=(const APIFrame&) 3");
			}
			memcpy(data_, other.data_, length_); // and copy
		} else {							// and we don't
			length_ = other.length_;		
			//data_ = new uint8_t[length_];
			data_ = allocBlock(length_, "operator=(const APIFrame&) 4");
			memcpy(data_, other.data_, length_);
		}
	}
	checksum_ = other.checksum_;

	return *this;
}

bb::XBee::APIFrame::~APIFrame() {
	if(data_ != NULL) {
		//delete data_;
		freeBlock(data_, length_, "~APIFrame");
	}
}

void bb::XBee::APIFrame::calcChecksum() {
	checksum_ = 0;
	if(data_ != NULL) {
		for(uint16_t i=0; i<length_; i++) {
			checksum_ += data_[i];
		}
	}

	checksum_ = 0xff - (checksum_ & 0xff);
}

bb::XBee::APIFrame bb::XBee::APIFrame::atRequest(uint8_t frameID, uint16_t command) {
	APIFrame frame(4);

	frame.data_[0] = ATREQUEST; // local AT request
	frame.data_[1] = frameID;
	frame.data_[2] = (command>>8) & 0xff;
	frame.data_[3] = command & 0xff;
	frame.calcChecksum();

	return frame;
}

bool bb::XBee::APIFrame::isATRequest() {
	return data_[0] == ATREQUEST && length_ > 4;
}

bool bb::XBee::APIFrame::isATResponse() {
	return data_[0] == ATRESPONSE && length_ > 4;
}

bool bb::XBee::APIFrame::is16BitRXPacket() {
	return data_[0] == RECEIVE16BIT && length_ > 5;
}

bool bb::XBee::APIFrame::is64BitRXPacket() {
	return data_[0] == RECEIVE64BIT && length_ > 11;
}

bb::Result bb::XBee::APIFrame::unpackATResponse(uint8_t &frameID, uint16_t &command, uint8_t &status, uint8_t** data, uint16_t &length) {
	if(data_[0] != ATRESPONSE) {
		return RES_SUBSYS_COMM_ERROR;
	} 
	if(length_ < 5) {
		Console::console.printfBroadcast("AT response too short (%d, need min 5)\n", length_);
		return RES_SUBSYS_COMM_ERROR;
	}

	frameID = data_[1];
	command = (data_[2]<<8)|data_[3];
	status = data_[4];

	if(length_ > 5) {
		*data = &(data_[5]);
		length = length_-5;
	} else {
		*data = NULL;
		length = 0;
	}

	return RES_OK;
}

static inline int writeEscapedByte(HardwareSerial* uart, uint8_t byte) {
	int sent = 0;
	if(byte == 0x7d || byte == 0x7e || byte == 0x11 || byte == 0x13) {
		sent += uart->write(0x7d);
		sent += uart->write(byte ^ 0x20);
	} else {
		sent += uart->write(byte);
	}
	return sent;
}

static inline uint8_t readEscapedByte(HardwareSerial* uart) {
	uint8_t byte = uart->read();
	if(byte != 0x7d) return byte;
	return (uart->read()^0x20);
}

bb::Result bb::XBee::send(const APIFrame& frame) {
	if(apiMode_ == false) return RES_SUBSYS_WRONG_MODE;
	
	uint16_t length = frame.length();
	const uint8_t *data = frame.data();

	uint8_t lengthMSB = (length >> 8) & 0xff;
	uint8_t lengthLSB = length & 0xff;
	
	uart_->write(0x7e); // start delimiter
	writeEscapedByte(uart_, lengthMSB);
	writeEscapedByte(uart_, lengthLSB);

#if 0
	Console::console.printfBroadcast("Writing %d bytes: ", length);
	for(uint16_t i=0; i<length; i++) {
		Console::console.printfBroadcast("%x ", data[i]);
	}
	Console::console.printfBroadcast("\n");
	Console::console.printfBroadcast("Writing checksum %x\n", frame.checksum());
#endif

	for(uint16_t i=0; i<length; i++) {
		writeEscapedByte(uart_, data[i]);
	}
	writeEscapedByte(uart_, frame.checksum());

	return RES_OK;
}

bb::Result bb::XBee::receive(APIFrame& frame) {
	uint8_t byte = 0xff;

	while(uart_->available()) {
		byte = uart_->read();
		if(byte == 0x7e) {
			break;
		} else {
			// Console::console.printfBroadcast("Read garbage '%c' 0x%x\n", byte, byte);
		}
	}
	if(byte != 0x7e) {
		return RES_SUBSYS_COMM_ERROR;
	}
//	Console::console.printfBroadcast("Start delimiter found\n");

	if(!uart_->available()) delayMicroseconds(200);
	if(!uart_->available()) return RES_SUBSYS_COMM_ERROR;	
	uint8_t lengthMSB = readEscapedByte(uart_);
	if(!uart_->available()) delayMicroseconds(200);
	if(!uart_->available()) return RES_SUBSYS_COMM_ERROR;	
	uint8_t lengthLSB = readEscapedByte(uart_);

	if(lengthLSB == 0x7e || lengthMSB == 0x7e) {
		Console::console.printfBroadcast("Extra start delimiter found\n");
		return RES_SUBSYS_COMM_ERROR;		
	}

	uint16_t length = (lengthMSB << 8) | lengthLSB;
	frame = APIFrame(length);
	uint8_t *buf = frame.data();
	for(uint16_t i=0; i<length; i++) {
		//Console::console.printfBroadcast("Reading byte %d of %d\n", i, length);
		if(!uart_->available()) delayMicroseconds(200);
		buf[i] = readEscapedByte(uart_);
	}
	frame.calcChecksum();

	if(!uart_->available()) delayMicroseconds(200);
	uint8_t checksum = readEscapedByte(uart_);

	if(frame.checksum() != checksum) {
		//Console::console.printfBroadcast("Checksum invalid - expected 0x%x, got 0x%x\n", frame.checksum(), checksum);
		return RES_SUBSYS_COMM_ERROR;
	}

	return RES_OK;
}