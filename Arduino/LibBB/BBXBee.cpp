#include <limits.h> // for ULONG_MAX
#include <vector>

#include "BBXBee.h"
#include "BBError.h"
#include "BBConsole.h"
#include "BBRunloop.h"

bb::XBee bb::XBee::xbee;

static std::vector<int> baudRatesToTry = { 115200, 9600, 19200, 28800, 38400, 57600, 76800 }; // start with 115200, then try 9600

bb::XBee::XBee() {
	uart_ = &Serial1;
	debug_ = (XBee::DebugFlags)(DEBUG_PROTOCOL | DEBUG_XBEE_COMM);
	timeout_ = 1000;
	atmode_ = false;
	atmode_millis_ = 0;
	atmode_timeout_ = 10000;
	currentBPS_ = 0;
	memset(packetBuf_, 0, sizeof(packetBuf_));
	packetBufPos_ = 0;

	name_ = "xbee";
	description_ = "Communication via XBee 802.5.14";
	help_ = "In order for communication to work, the PAN and channel numbers must be identical.\r\n" \
	"Available commands:\r\n" \
	"\tsend <string>:      Send <string> to partner\r\n"\
	"\tsend_cmd_packet:    Send a zero command packet (with sequence number and CRC set) to partner\r\n"
	"\tcontinuous on|off:  Start or stop sending a continuous stream of numbers (or zero command packets when in packet mode)\r\n"\
	"\tpacket_mode on|off: Switch to packet mode\r\n";

	addParameter("channel", "Communication channel (between 11 and 26, usually 12)", chan_, 11, 26);
	addParameter("pan", "Personal Area Network ID (16bit, 65535 is broadcast)", pan_, 0, 65535);
	addParameter("station", "Station ID (MY) for this device (16bit)", station_, 0, 65535);
	addParameter("partner", "Partner ID this device should talk to (16bit)", partner_, 0, 65535);
	addParameter("bps", "Communication bps rate", bps_, 0, 200000);

	paramsHandle_ = ConfigStorage::storage.reserveBlock(sizeof(XBeeParams));
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
}

bb::XBee::~XBee() {
}

bb::Result bb::XBee::initialize(uint8_t chan, uint16_t pan, uint16_t station, uint16_t partner, uint32_t bps, HardwareSerial *uart) {
	if(operationStatus_ != RES_SUBSYS_NOT_INITIALIZED) return RES_SUBSYS_ALREADY_INITIALIZED;

	paramsHandle_ = ConfigStorage::storage.reserveBlock(sizeof(params_));
	//Console::console.printBroadcast(String("Handle: ") + String(paramsHandle_, HEX));
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
		//Console::console.printlnBroadcast("Block valid, reading config");
		ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
	} else {
		//Console::console.printlnBroadcast("Block invalid, initializing");
		memset(&params_, 0, sizeof(params_));
		params_.chan = chan;
		params_.pan = pan;
		params_.station = station;
		params_.partner = partner;
		params_.bps = bps;
	}

	uart_ = uart;

	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	return Subsystem::initialize();	
}

bb::Result bb::XBee::start(ConsoleStream *stream) {
	if(isStarted()) return RES_SUBSYS_ALREADY_STARTED;
	if(NULL == uart_) return RES_SUBSYS_HW_DEPENDENCY_MISSING;

	currentBPS_ = 0;
	if(currentBPS_ == 0) {
		if(stream) stream->print("auto-detecting BPS... ");
		
		for(size_t i=0; i<baudRatesToTry.size(); i++) {
			if(stream) {
				stream->print(baudRatesToTry[i]); 
				stream->print("... ");
			}
			
			uart_->begin(baudRatesToTry[i]);
			
			if(enterATModeIfNecessary() == RES_OK) {
				currentBPS_ = baudRatesToTry[i];
				break; 
			}
		}
		if(currentBPS_ == 0) {
			if(stream) stream->println("failed.");
			operationStatus_ = RES_SUBSYS_HW_DEPENDENCY_MISSING;
			return RES_SUBSYS_HW_DEPENDENCY_MISSING;
		}
	} else {
		if(stream) stream->print(String("Using ") + currentBPS_ + "bps.");
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

	if(stream) {
		stream->print("Found XBee at address: "); stream->print(addrH);
		stream->print(":"); stream->print(addrL);
		stream->print(" Firmware version: "); stream->println(fw);
	}

	String retval = sendStringAndWaitForResponse("ATCT"); 
	if(retval != "") {
		atmode_timeout_ = strtol(retval.c_str(), 0, 16) * 100;
		if(stream!=NULL) stream->println(String("Command timeout: ") + atmode_timeout_);
	}
	if(atmode_timeout_ != 1000) {
		if(sendStringAndWaitForOK("ATCT=a") == false) {
			if(stream) stream->println("Couldn't set AT Mode Timeout");
			return RES_SUBSYS_COMM_ERROR;
		}

		// need to leave and jump back in here to make sure the new timeout is used
		leaveATMode();
		enterATModeIfNecessary();
		atmode_timeout_ = 1000;
	}

	if(setConnectionInfo(params_.chan, params_.pan, params_.station,params_.partner, true) != RES_OK) {
		if(stream) stream->println("Setting connection info failed.");
		return RES_SUBSYS_COMM_ERROR;
	} else {
		if(stream) stream->println("Setting connection info successful.");
	}

	if(params_.bps != currentBPS_) {
		if(changeBPSTo(params_.bps, stream, true) != RES_OK) {
			if(stream) stream->println("Setting BPS failed.");
			return RES_SUBSYS_COMM_ERROR;
		}
	}

	if(sendStringAndWaitForOK("ATWR") == false) {
		if(stream) stream->println("Couldn't write config!");
		return RES_SUBSYS_COMM_ERROR;
	}

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
	while(available()) {
		if(packetMode_) {
			Result retval = receiveAndHandlePacket();
			if(retval != RES_OK) return retval;
		} else {
			bb::Console::console.printlnBroadcast(receive());
		}
	}

	if(sendContinuous_) {
		if(packetMode_) {

			Packet packet;
			memset(&packet, 0, sizeof(packet));
			packet.type = PACKET_TYPE_COMMAND;
			packet.source = PACKET_SOURCE_TEST_ONLY;

			return send(packet);
		} else {
			String str(continuous_++);
			bb::Console::console.printBroadcast("Sending ");
			bb::Console::console.printlnBroadcast(str);
			send(str);			
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
	} else if(name == "partner") {
		value = String(params_.partner); return RES_OK;
	} else if(name == "bps") {
		value = String(params_.bps); return RES_OK;
	} 

	return RES_PARAM_NO_SUCH_PARAMETER;
}

bb::Result bb::XBee::setParameterValue(const String& name, const String& value) {
	Result res = RES_PARAM_NO_SUCH_PARAMETER;

	if(name == "channel") { 
		params_.chan = value.toInt();
		res = RES_OK;
	} else if(name == "pan") {
		params_.pan = value.toInt();
		res = RES_OK;
	} else if(name == "station") {
		params_.station = value.toInt();
		res = RES_OK;
	} else if(name == "partner") {
		params_.partner = value.toInt();
		res = RES_OK;
	} else if(name == "bps") {
		params_.bps = value.toInt();
		res = RES_OK;
	} 

	if(res == RES_OK) ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);

	return res;
}

bb::Result bb::XBee::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
	if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
	
	if(words[0] == "send") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		return send(words[1]);
	} 

	else if(words[0] == "send_cmd_packet") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT;

		Packet packet;
		memset(&packet, 0, sizeof(packet));
		packet.type = PACKET_TYPE_COMMAND;
		packet.source = PACKET_SOURCE_TEST_ONLY;

		return send(packet);
	} 

	else if(words[0] == "continuous") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		else {
			if(words[1] == "on" || words[1] == "true") {
				sendContinuous_ = true;
				continuous_ = 0;
				return RES_OK;
			}
			else if(words[1] == "off" || words[1] == "false") {
				sendContinuous_ = false;
				return RES_OK;
			}
			else return RES_CMD_INVALID_ARGUMENT;	
		}
	} 

	else if(words[0] == "packet_mode") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		else {
			if(words[1] == "on" || words[1] == "true") {
				setPacketMode(true);
				sendContinuous_ = false;
				return RES_OK;
			}
			else if(words[1] == "off" || words[1] == "false") {
				setPacketMode(false);
				return RES_OK;
			}
			else return RES_CMD_INVALID_ARGUMENT;	
		}
	} 

	return bb::Subsystem::handleConsoleCommand(words, stream);
}

void bb::XBee::setPacketMode(bool onoff) {
	packetMode_ = onoff;
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
	if(isInATMode()) {
		return RES_OK;
	}

	if(debug_ && stream!=NULL) {
		stream->println("Entering AT mode.");
		stream->print("Wait 1s... ");
	}

	int numDiscardedBytes = 0;

	for(int timeout = 0; timeout < 1000; timeout++) {
		while(uart_->available())  {
			uart_->read();
			numDiscardedBytes++;
		}
		delay(1);
	}

	if(debug_ && stream!=NULL) 
		stream->println("Sending +++... ");

	uart_->write("+++");

	bool success = false;

	for(int timeout = 0; timeout < 1200; timeout++) {
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
					} else numDiscardedBytes++;
				} else numDiscardedBytes++;
			} else numDiscardedBytes++;
		}

		delay(1);
	}


	if(success) {
		if(debug_ && stream!=NULL) {
			stream->println("Successfully entered AT Mode");
		}
		if(numDiscardedBytes) {
			Console::console.printBroadcast(String("Discarded ") + numDiscardedBytes + " bytes while entering AT mode.");
		}
		atmode_millis_ = millis();
		atmode_ = true;
				
		return RES_OK;
	}

	if(debug_ && stream!=NULL) stream->println("no response to +++");
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
	if((debug_ & DEBUG_PROTOCOL) && stream!=NULL) stream->println("Sending ATCN to leave AT Mode");
	if(sendStringAndWaitForOK("ATCN") == true) {
		if((debug_ & DEBUG_PROTOCOL) && stream!=NULL) stream->println("Successfully left AT Mode");
		atmode_ = false;
		delay(1100);
		return RES_OK;
	}

	if((debug_ & DEBUG_PROTOCOL) && stream!=NULL) stream->println("no response to ATCN");
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

	String retval = sendStringAndWaitForResponse("ATBD");

	if((unsigned)retval.toInt() == paramVal) {
		if(stream) stream->println(String("XBee says it's already running at ") + bps + "bps");
		return RES_OK;
	}

	if(stream) stream->println(String("Sending ATBD=")+String(paramVal, HEX));
	if(sendStringAndWaitForOK(String("ATBD=")+String(paramVal, HEX)) == false) return RES_SUBSYS_COMM_ERROR;

	if(stream) stream->println(String("Closing and reopening serial at ") + bps + "bps");
	uart_->end();	
	uart_->begin(bps);

	if(stream) stream->println(String("Waiting for ") + atmode_timeout_ + "ms to drop out of AT mode");
	delay(atmode_timeout_+10);

	enterATModeIfNecessary();
	if(sendStringAndWaitForOK("AT") == false) return RES_SUBSYS_COMM_ERROR;

	if(stayInAT == false) leaveATMode();
	currentBPS_ = bps;

	return RES_OK;
}

bb::Result bb::XBee::setConnectionInfo(uint8_t chan, uint16_t pan, uint16_t station, uint16_t partner, bool stayInAT) {
	params_.chan = chan;
	params_.pan = pan;
	params_.station = station;
	params_.partner = partner;

	enterATModeIfNecessary();

	if(sendStringAndWaitForOK(String("ATCH=")+String(chan, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printlnBroadcast("ERROR: Setting Channel failed!");
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	} 
	delay(10);
	if(sendStringAndWaitForOK(String("ATID=")+String(pan, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printlnBroadcast("ERROR: Setting PAN failed!");
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	}
	delay(10);
	if(sendStringAndWaitForOK(String("ATMY=")+String(station, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printlnBroadcast("ERROR: Setting MY failed!");
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	}
	delay(10);
	if(sendStringAndWaitForOK(String("ATDL=")+String(partner, HEX)) == false) {
		if(debug_ & DEBUG_PROTOCOL) Console::console.printlnBroadcast("ERROR: Setting Destination Low failed!");
		if(!stayInAT) leaveATMode();
		return RES_COMM_TIMEOUT;
	}
	delay(10);

	if(!stayInAT) leaveATMode();

	return RES_OK;
}

bb::Result bb::XBee::getConnectionInfo(uint8_t& chan, uint16_t& pan, uint16_t& station, uint16_t& partner, bool stayInAT) {
	enterATModeIfNecessary();

	String retval;

	retval = sendStringAndWaitForResponse("ATCH");
	if(retval == "") return RES_COMM_TIMEOUT;
	chan = strtol(retval.c_str(), 0, 16);
	delay(10);

	retval = sendStringAndWaitForResponse("ATID");
	if(retval == "") return RES_COMM_TIMEOUT;
	pan = strtol(retval.c_str(), 0, 16);
	delay(10);

	retval = sendStringAndWaitForResponse("ATDL");
	if(retval == "") return RES_COMM_TIMEOUT;
	partner = strtol(retval.c_str(), 0, 16);
	delay(10);

	retval = sendStringAndWaitForResponse("ATMY");
	if(retval == "") return RES_COMM_TIMEOUT;
	station = strtol(retval.c_str(), 0, 16);
	delay(10);

	if(!stayInAT) leaveATMode();
	return RES_OK;
}

bb::Result bb::XBee::discoverNetworks() {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;

	enterATModeIfNecessary();

	Console::console.printlnBroadcast("Discover Networks");
	uart_->print("ATND\r");
	uart_->flush();
	int to=13;
	while(to>0) {
		if(uart_->available()) {
			Console::console.printlnBroadcast(uart_->readString());
		}
		delay(1000);
		to--;
	}
	if(to>0) return RES_OK;
	return RES_COMM_TIMEOUT;
}

void bb::XBee::setDebugFlags(DebugFlags debug) {
	debug_ = debug;
}

bb::Result bb::XBee::send(const String& str) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(packetMode_) return RES_SUBSYS_WRONG_MODE;
	if(isInATMode()) leaveATMode();
	uart_->write(str.c_str(), str.length());
	uart_->flush();
	return RES_OK;
}

bb::Result bb::XBee::send(const uint8_t *bytes, size_t size) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(packetMode_) return RES_SUBSYS_WRONG_MODE;
	if(isInATMode()) leaveATMode();
	uart_->write(bytes, size);
	uart_->flush();
	return RES_OK;
}

bb::Result bb::XBee::send(const bb::Packet& packet) {
	if(operationStatus_ != RES_OK) return RES_SUBSYS_NOT_OPERATIONAL;
	if(!packetMode_) return RES_SUBSYS_WRONG_MODE;
	if(isInATMode()) leaveATMode();

	bb::PacketFrame frame;
	frame.packet = packet;

	uint8_t *buf = (uint8_t*)&frame.packet;
	for(size_t i=0; i<sizeof(frame.packet)-1; i++) {
		if(buf[i] & 0x80) {
			Console::console.printlnBroadcast(String("ERROR: Byte ") + i + " of packet has highbit set! Not sending.");
			return RES_PACKET_INVALID_PACKET;
		}
	}

	frame.packet.seqnum = Runloop::runloop.getSequenceNumber() % MAX_SEQUENCE_NUMBER;
	frame.crc = calculateCRC(frame.packet);
	frame.highbit = 1;

#if 0
	buf = (uint8_t*)&frame;
	for(size_t i=0; i<sizeof(frame); i++) {
		Console::console.printBroadcast(String(buf[i], HEX) + " ");
	}
	Console::console.printlnBroadcast();
#endif

	uart_->write((uint8_t*)&frame, sizeof(frame));
	uart_->flush();

	return RES_OK;
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

bb::Result bb::XBee::receiveAndHandlePacket() {
	if(!packetMode_) {
		bb::Console::console.printlnBroadcast("Wrong mode.");
		return RES_SUBSYS_WRONG_MODE;
	} 

	while(available()) {
		packetBuf_[packetBufPos_] = uart_->read();
		//bb::Console::console.printBroadcast(String(packetBuf_[packetBufPos_], HEX) + " ");
		if(packetBuf_[packetBufPos_] & 0x80) {
			if(packetBufPos_ == sizeof(PacketFrame)-1) {
				PacketFrame frame;
				memcpy(&frame, packetBuf_, sizeof(PacketFrame));

				memset(packetBuf_, 0, sizeof(packetBuf_));
				packetBufPos_ = 0;

				if(frame.crc != calculateCRC(frame.packet)) {
					bb::Console::console.printlnBroadcast(String("CRC incorrect - ") + String(frame.crc, HEX) + 
						                                    String(" instead of ") + String(calculateCRC(frame.packet), HEX));
					return RES_PACKET_INVALID_PACKET;
				} 

				for(size_t i=0; i<receivers_.size(); i++) {
					receivers_[i]->incomingPacket(frame.packet);
				}

				return RES_OK;
			} else {
				Console::console.printlnBroadcast(String("Wrong packet size: ") + String(packetBufPos_+1) + String(" bytes instead of ") + sizeof(PacketFrame));
				for(size_t i=0; i<=packetBufPos_; i++) {
					Console::console.printBroadcast(String(packetBuf_[i], HEX) + " ");
				}
				Console::console.printlnBroadcast("");
				memset(packetBuf_, 0, sizeof(packetBuf_));
				packetBufPos_ = 0;
				return RES_PACKET_TOO_SHORT;
			}
		} else {
			packetBufPos_++;
			if(packetBufPos_ >= sizeof(packetBuf_)) {
				bb::Console::console.printlnBroadcast(String("Packet too long: ") + String(packetBufPos_+1) + String(" bytes instead of ") + sizeof(PacketFrame));

				memset(packetBuf_, 0, sizeof(packetBuf_));
				packetBufPos_ = 0;

				return RES_PACKET_TOO_LONG;
			}
		}
	}

	return RES_PACKET_TOO_SHORT;
}

String bb::XBee::sendStringAndWaitForResponse(const String& str, int predelay, bool cr) {
  	if(debug_ & DEBUG_XBEE_COMM) {
    	Console::console.printBroadcast("Sending \"");
    	Console::console.printBroadcast(str);
    	Console::console.printBroadcast("\"... ");
  	}

    uart_->print(str);
  	if(cr) {
  		uart_->print("\r");
  	}
    uart_->flush();


    if(predelay > 0) delay(predelay);

    String retval;
    if(readString(retval)) {
    	if(debug_ & DEBUG_XBEE_COMM) {
    		Console::console.printBroadcast(retval);
    		Console::console.printlnBroadcast(" ");
    	}
    	return retval;
    }

    if(debug_ & DEBUG_PROTOCOL) {
    	Console::console.printlnBroadcast("Nothing.");
    }
    return "";
}
  
bool bb::XBee::sendStringAndWaitForOK(const String& str, int predelay, bool cr) {
  	String result = sendStringAndWaitForResponse(str, predelay, cr);
  	if(result.equals("OK\r")) return true;
      
  	if(debug_ & DEBUG_PROTOCOL) {
    	Console::console.printBroadcast("Expected \"OK\", got \"");
    	Console::console.printBroadcast(result);
    	Console::console.printlnBroadcast("\"... ");
  	}
  return false;
}

bool bb::XBee::readString(String& str, unsigned char terminator) {
	while(true) {
		int to = 0;
		while(!uart_->available()) {
			delay(1);
			to++;
			if(debug_ & DEBUG_XBEE_COMM) Console::console.printBroadcast(".");
			if(to >= timeout_) {
				if(debug_ & DEBUG_PROTOCOL) Console::console.printlnBroadcast("Timeout!");
				return false;
			}
		}
		unsigned char c = (unsigned char)uart_->read();
		str += (char)c;
		if(c == terminator) return true;
	}
}