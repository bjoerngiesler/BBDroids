#include <BBWifiServer.h>
#include <BBRunloop.h>
#if !defined(ARDUINO_PICO_VERSION_STR)
#include <ArduinoOTA.h>
#endif

String IPAddressToString(const IPAddress& addr) {
	return String(addr[0]) + "." + addr[1] + "." + addr[2] + "." + addr[3];
}

bb::WifiServer bb::WifiServer::server;

bb::WifiConsoleStream::WifiConsoleStream() {
}

void bb::WifiConsoleStream::setClient(const WiFiClient& client) {
	client_ = client;
	printGreeting();
}

bool bb::WifiConsoleStream::available() {
	if(client_.available()) {
		return true;
	} else {
		return false;
	}
}

bool bb::WifiConsoleStream::readStringUntil(unsigned char c, String& str) {
	int timeout = 0;
	while(true) {
		if(client_.available()) {
			unsigned char data = client_.read();
			str += (char)data;
			if(data == c) return true;
		} else {
			delay(1);
			timeout++;
			if(timeout >= 2) return false;
		}
	}
}

void bb::WifiConsoleStream::printfFinal(const char* str) {
	client_.print(str);
}

void bb::WifiConsoleStream::print(size_t val) {
	client_.print(val);
}

void bb::WifiConsoleStream::print(int val) {
	client_.print(val);
}

void bb::WifiConsoleStream::print(float val) {
	client_.print(val);
}

void bb::WifiConsoleStream::print(const String& val) {
	client_.print(val);
}

void bb::WifiConsoleStream::println(int val) {
	client_.println(val);
}

void bb::WifiConsoleStream::println(float val) { 
	client_.println(val);
}

void bb::WifiConsoleStream::println(const String& val) {
	client_.println(val);
}

void bb::WifiConsoleStream::println() {
	client_.println();
}

bb::WifiServer::WifiServer(): tcp_(DEFAULT_TCP_PORT) {
	name_ = "wifi";
	help_ = "Creates an AP or joins a network. Starts a shell on TCP.\r\nSSID and WPA Key replacements: $MAC - Mac address";
	description_ = "Wifi comm module (uninitialized)";
	
	addParameter("ssid", "SSID", ssid_);
	addParameter("wpa_key", "WPA Key", wpaKey_);
	addParameter("ap", "Access Point Mode", params_.ap);
	addParameter("terminal_port", "TCP port to use for terminal access", params_.tcpPort, 0, 32767);
	addParameter("remote_port", "UDP port to use for remote packet publishing", params_.udpPort, 0, 32767);
}

bb::Result bb::WifiServer::initialize(const String& ssid, const String& wpakey, bool apmode, uint16_t udpPort, uint16_t tcpPort) {
	if(operationStatus_ != RES_SUBSYS_NOT_INITIALIZED) return RES_SUBSYS_ALREADY_INITIALIZED;

	paramsHandle_ = ConfigStorage::storage.reserveBlock(sizeof(params_));
	Serial.print("Handle: "); Serial.println(paramsHandle_);
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
		Serial.println("Block valid, reading config");
		ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
	} else {
		Serial.println("Block invalid, initializing");
		memset(&params_, 0, sizeof(params_));
		strncpy(params_.ssid, ssid.c_str(), MAX_STRLEN);
		strncpy(params_.wpaKey, wpakey.c_str(), MAX_STRLEN);
		params_.ap = apmode;
		params_.udpPort = udpPort;
		params_.tcpPort = tcpPort;
	}

	byte mac[6];
	WiFi.macAddress(mac);
	macStr_ = String(mac[5], HEX) + ":" + String(mac[4], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[0], HEX);

	setOTANameAndPassword(ssid, wpakey);

	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	return Subsystem::initialize();
}

bb::Result bb::WifiServer::setOTANameAndPassword(const String& name, const String& password) {
#if !defined(ARDUINO_PICO_VERSION_STR)
	if(WiFi.status() == WL_NO_MODULE) return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;

	otaName_ = name;
	otaName_.replace("$MAC", macStr_);
	otaPassword_ = password;
	otaPassword_.replace("$MAC", macStr_);

	return RES_OK;
#else
	return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
#endif
}


bb::Result bb::WifiServer::start(ConsoleStream* stream) {
	if(WiFi.status() == WL_NO_MODULE) {
		if(stream) stream->println("WL_NO_MODULE!");
		return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
	} 

	WiFi.noLowPowerMode();

	ssid_ = params_.ssid;
	wpaKey_ = params_.wpaKey;
	ssid_.replace("$MAC", macStr_);
	wpaKey_.replace("$MAC", macStr_);

	if(params_.ap == true) { // Start Access Point
		if(stream) {
			stream->print(String("Start AP \"") + ssid_ + "\"... ");
		}

		uint8_t retval = WiFi.beginAP(ssid_.c_str(), wpaKey_.c_str());
		if(retval == WL_AP_LISTENING) {
			if(stream) stream->print("success. ");
		} else {
			if(stream) stream->println("failure.");
			return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
		}
	} else { // Connect as client
		if(stream) { 
			stream->print(String("Connect to \"") + ssid_ + "\"... ");
		}

		uint8_t retval = WiFi.begin(ssid_.c_str(), wpaKey_.c_str());
		if(retval == WL_CONNECTED) {
			if(stream) stream->print("success. ");
		} else {
			if(stream) stream->println("failure.");
			return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
		}
	}

	ArduinoOTA.begin(WiFi.localIP(), otaName_.c_str(), otaPassword_.c_str(), InternalStorage);
	tcp_ = WiFiServer(params_.tcpPort);
	tcp_.begin();
	udp_.begin(params_.udpPort);

	operationStatus_ = RES_OK;
	started_ = true;

	return RES_OK;
}

bb::Result bb::WifiServer::stop(ConsoleStream* stream) {
	if(stream) stream = stream; // make compiler happy

	WiFiClient client = tcp_.available();
	client.stop();
	bb::Console::console.removeConsoleStream(&consoleStream_);

	udp_.stop();
	ArduinoOTA.end();
	WiFi.end();

	started_ = false;
	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	return RES_OK;
}

bb::Result bb::WifiServer::step() {
	int status = WiFi.status();
	static int seqnum = 0;
	static int dummy = 0;

	if(seqnum == (1e6/Runloop::runloop.cycleTimeMicros())/4) {
#if !defined(ARDUINO_PICO_VERSION_STR)
		unsigned long m = micros();
		ArduinoOTA.poll();
#endif
		seqnum = 0;
	}

	seqnum++;

	if(client_ == true && client_.status() == 0) {
		client_.stop();
		consoleStream_.setClient(client_);
		Console::console.removeConsoleStream(&consoleStream_);
	}

	WiFiClient c = tcp_.available();
	if(client_ == false && c == true) {
		client_ = c;
		consoleStream_.setClient(client_);
		Console::console.addConsoleStream(&consoleStream_);
	}

	return RES_OK;
}

bb::Result bb::WifiServer::setParameterValue(const String& name, const String& value) {
	Result res = RES_PARAM_NO_SUCH_PARAMETER;

	if(name == "ssid") { 
		strncpy(params_.ssid, value.c_str(), MAX_STRLEN);
		res = RES_OK;
	} else if(name == "wpa_key") {
		strncpy(params_.wpaKey, value.c_str(), MAX_STRLEN);
		res = RES_OK;
	} else if(name == "ap") {
		if(value == "1") { params_.ap = true; res = RES_OK; }
		else if(value == "0") { params_.ap = false; res = RES_OK; }
		else res = RES_PARAM_INVALID_TYPE;
	} else if(name == "terminal_port") {
		int v = value.toInt();
		if(v < 0 || v > 65536) return RES_PARAM_INVALID_VALUE;
		params_.tcpPort = v;
		res = RES_OK;
	} else if(name == "remote_port") {
		int v = value.toInt();
		if(v < 0 || v > 65536) return RES_PARAM_INVALID_VALUE;
		params_.udpPort = v;
		res = RES_OK;
	} 

	if(res == RES_OK) ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);

	return res;
}

bb::Result bb::WifiServer::operationStatus() {
	if(!started_) return RES_SUBSYS_NOT_STARTED;
	return RES_OK;
}

bool bb::WifiServer::isAPStarted() {
  return WiFi.status() == WL_AP_LISTENING || WiFi.status() == WL_AP_CONNECTED;
}

bool bb::WifiServer::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool bb::WifiServer::broadcastUDPPacket(const uint8_t* packet, size_t len) {
	IPAddress ip = WiFi.localIP();
	ip[3] = 0xff;

	return sendUDPPacket(ip, packet, len);
}

bool bb::WifiServer::sendUDPPacket(const IPAddress& addr, const uint8_t* packet, size_t len) {
	static unsigned int failures = 0;

	if(WiFi.status() != WL_CONNECTED && WiFi.status() != WL_AP_CONNECTED) return false;
	if(udp_.beginPacket(addr, params_.udpPort) == false) {
		Console::console.printlnBroadcast("beginPacket() failed!");
		return false;
	}

	if(udp_.write(packet, len) != len) {
		Console::console.printlnBroadcast("write() failed");
		return false;
	}

	if(udp_.endPacket() == false) {
		failures++;
		if(failures > 10) {
			Console::console.printlnBroadcast(String("endPacket() failed ") + failures + " in a row!");
			failures = 0;
		}

		return false;
	}

	failures = 0;
	return true;
}


unsigned int bb::WifiServer::readDataIfAvailable(uint8_t *buf, unsigned int maxsize, IPAddress& remoteIP) {
	unsigned int len = udp_.parsePacket();
	if(!len) return 0;
	remoteIP = udp_.remoteIP();
	if(len > maxsize) return len;
	if((unsigned int)(udp_.read(buf, maxsize)) != len) { 
		Serial.println("Huh? Differing sizes?!"); 
		return 0;
	} else {
		return len;
	}
}

void bb::WifiServer::updateDescription() {
	if(WiFi.status() == WL_NO_MODULE) {
		description_ = "No Wifi module installed.";
		return;
	}

	description_ = String("Wifi module ") + macStr_ + ", status: ";
	
	IPAddress ip;

	switch(WiFi.status()) {
    case WL_IDLE_STATUS:
    	description_ += "idle";
    	break;
    case WL_NO_SSID_AVAIL:
    	description_ += "no SSID available";
    	break;
    case WL_SCAN_COMPLETED:
    	description_ += "scan completed";
    	break;
    case WL_CONNECTED:
    	description_ += "connected as client, IP " + IPAddressToString(WiFi.localIP());
    	break;
    case WL_CONNECT_FAILED:
    	description_ += "conn failed";
    	break;
    case WL_CONNECTION_LOST:
    	description_ += "conn lost";
    	break;
    case WL_DISCONNECTED:
    	description_ += "disconnected";
    	break;
    case WL_AP_LISTENING:
    	description_ += "AP listening";
    	break;
    case WL_AP_CONNECTED:
    	description_ += "connected as AP, IP " + IPAddressToString(WiFi.localIP());
    	break;
    case WL_AP_FAILED:
    	description_ += "AP setup failed";
    	break;
    default:
    	description_ += "unknown";
    	break;
	}

#if 0
	if(client_ == true) {
		description_ += ", client ";
		ip = client_.remoteIP();
  	description_ += ip[0];
  	description_ += ".";
  	description_ += ip[1];
  	description_ += ".";
  	description_ += ip[2];
  	description_ += ".";
  	description_ += ip[3];
		description_ += " connected";
	}
#endif
}