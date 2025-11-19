#include "RemoteSubsys.h"
#include "nvs_flash.h"
#include "nvs.h"

extern bool isLeftRemote;

using namespace bb::rmt;


RemoteSubsys RemoteSubsys::inst;

static const char* STORAGE_NAMESPACE = "rss";

bool RemoteSubsys::memoryRead(ProtocolStorage& storage) {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        bb::printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return false;
    }

    // Write blob
    bb::printf("Retrieving protocol storage data blob...\n");
    size_t size;
    err = nvs_get_blob(handle, "proto_s", (void*)&storage, &size);
    if (err != ESP_OK) {
        bb::printf("Failed to read protocol storage data blob!\n");
        nvs_close(handle);
        return false;
    }

    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        bb::printf("Failed to commit data\n");
    }

    nvs_close(handle);
    return true;
}

bool RemoteSubsys::memoryWrite(const ProtocolStorage& storage) {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        bb::printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return false;
    }

    // Write blob
    bb::printf("Saving %d bytes of protocol storage data blob (name max %d)...\n", sizeof(ProtocolStorage), NVS_KEY_NAME_MAX_SIZE);
    err = nvs_set_blob(handle, "proto_s", &storage, sizeof(ProtocolStorage));
    if (err != ESP_OK) {
        bb::printf("Failed to write protocol storage data blob (%s)!\n", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    // Commit
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        bb::printf("Failed to commit data\n");
    }

    nvs_close(handle);
    return true;
}


Result RemoteSubsys::initialize() {
    ProtocolFactory::setMemoryReadFunction(memoryRead);
    ProtocolFactory::setMemoryWriteFunction(memoryWrite);

    interremote = ProtocolFactory::createProtocol(MONACO_XBEE);
    interremote->init(isLeftRemote ? "LeftRemote" : "RightRemote");
    current = interremote;

    name_ = "rss";
    description_ = "Remote Subsystem";
    help_ = "This subsystem encapsulates the BBRemote library.\n" \
"Commands:\n" \
"\tlist                           List all stored protocols\n"\
"\tcreate <type>                  Shutdown current protocol (if not interremote) and create new of type <type> (X/E/B/U/S/D/d)\n"\
"\tload {current|inter} <name>    Shutdown current protocol and load the named one\n"\
"\tstore {current|inter} <name>   Store current protocol under the given name\n"\
"\tprint_storage                  Print a description of storage contents\n"\
"\tdiscover {current|inter}       Run discovery on current\n";

    return Subsystem::initialize();
}

Result RemoteSubsys::start() {
    return Subsystem::start();
}

Result RemoteSubsys::stop() {
    return Subsystem::stop();
}

Result RemoteSubsys::step() {
    interremote->step();
    if(current != nullptr && current != interremote) current->step();
    return RES_OK;
}

Result RemoteSubsys::handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream) {
    if(words.size() == 0) return RES_CMD_INVALID_ARGUMENT;

    if(words[0] == "list") {
        std::vector<std::string> storedNames = ProtocolFactory::storedProtocols();
        stream->printf("Stored protocols: %d\n", storedNames.size());
        for(auto s: storedNames) stream->printf("\t\"%s\"\n", s.c_str());
        return RES_OK;
    }

    if(words[0] == "print_storage") {
        if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;

        ProtocolFactory::printStorage();
        return RES_OK;
    }

    if(words[0] == "create") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

        ProtocolType type = (ProtocolType)words[1][0];
        if(type == interremote->protocolType()) {
            stream->printf("Refuse to create new protocol of type '%c' as it's also the interremote protocol.\n", type);
            return RES_CMD_INVALID_ARGUMENT;
        }
        Protocol *proto = ProtocolFactory::createProtocol(type);
        if(proto == nullptr) {
            stream->printf("createProtocol() returns nullptr\n");
            return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
        }
        if(current != nullptr && current != interremote) {
            delete current;
        }
        current = proto;
        current->init(isLeftRemote ? "LeftRemote" : "RightRemote");
        return RES_OK;
    }

    if(words[0] == "load") {
        if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
        
        Protocol *proto = (words[1]=="current" ? current : (words[1]=="inter" ? interremote : nullptr));
        const String& name = words[2];
        if(proto == nullptr) {
            stream->printf("Protocol is null\n");
            return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
        }

        return RES_OK;
    }

    if(words[0] == "store") {
        Protocol *proto = (words[1]=="current" ? current : (words[1]=="inter" ? interremote : nullptr));
        const String& name = words[2];
        if(proto == nullptr) {
            stream->printf("Protocol is null\n");
            return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
        }

        if(ProtocolFactory::storeProtocol(name.c_str(), proto) == true) return RES_OK;

        return RES_CMD_FAILURE;
    }

    if(words[0] == "discover") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

        Protocol *proto = (words[1]=="current" ? current : (words[1]=="inter" ? interremote : nullptr));

        if(proto == nullptr) {
            stream->printf("Protocol is null\n");
            return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
        }
        proto->discoverNodes();
        stream->printf("%d nodes discovered\n", proto->numDiscoveredNodes());
        for(int i=0; i<current->numDiscoveredNodes(); i++) {
            const NodeDescription& descr = proto->discoveredNode(i);
            stream->printf("    Name: %s Addr: %s (configurator: %s receiver: %s transmitter: %s)\n", 
                descr.getName().c_str(), descr.addr.toString().c_str(),
                descr.isConfigurator ? "true" : "false",
                descr.isReceiver ? "true" : "false",
                descr.isTransmitter ? "true" : "false");
        }
        return RES_OK;
    }

    return Subsystem::handleConsoleCommand(words, stream);
}
