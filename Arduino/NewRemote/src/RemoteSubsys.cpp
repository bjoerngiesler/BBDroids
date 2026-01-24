#include "RemoteSubsys.h"
#include "nvs_flash.h"
#include "nvs.h"

extern bool isLeftRemote;

using namespace bb::rmt;

RemoteSubsys RemoteSubsys::inst;

const std::string RemoteSubsys::INTERREMOTE_PROTOCOL_NAME = "default";
static const char* STORAGE_NAMESPACE = "rss";

static const char *HELP = 
"This subsystem encapsulates the BBRemote library.\n" \
"Commands:\n" \
"\tlist                                 List all stored protocols\n"\
"\tcreate <type>                        Shutdown current protocol (if not interremote) and create new of type <type> (X/E/B/U/S/D/d)\n"\
"\tload {current|inter} <name>          Shutdown current protocol and load the named one\n"\
"\tstore {current|inter} <name>         Store current protocol under the given name\n"\
"\terase {all|<name>}                   Erase storage under the given name\n"\
"\tdiscover {current|inter}             Run discovery on current\n"\
"\tinfo {current|inter}                 Print information on the given protocol (paired nodes, etc)\n"\
"\tpair {current|inter} {<name>|<addr>} Pair the given protocol with the given name/addr (must have been discovered earlier)\n"
"\tprint_storage                        Print a description of storage contents\n"\
"\tcommit_storage                       Write storage to flash\n";


bool RemoteSubsys::memoryRead(ProtocolStorage& storage) {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        bb::printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return false;
    }

    // Write blob
    bb::printf("Retrieving protocol storage data blob... ");
    size_t size;
    err = nvs_get_blob(handle, "proto_s", (void*)&storage, &size);
    if (err != ESP_OK) {
        bb::printf("Error %s!\n", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    bb::printf("OK.\n");
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
    if(operationStatus() != RES_SUBSYS_NOT_INITIALIZED) {
        bb::printf("Already initialized!\n");
        return RES_SUBSYS_ALREADY_INITIALIZED;
    }

    ProtocolFactory::setMemoryReadFunction(memoryRead);
    ProtocolFactory::setMemoryWriteFunction(memoryWrite);
    Protocol::setTransmitFrequencyHz(25);

    interremote_ = nullptr;
    current_ = nullptr;

    // first setup interremote
    bb::printf("Loading protocol \"%s\"\n", INTERREMOTE_PROTOCOL_NAME);
    interremote_ = ProtocolFactory::loadProtocol(INTERREMOTE_PROTOCOL_NAME);
    if(interremote_ == nullptr) {
        bb::printf("Loading failed, creating new\n");
        interremote_ = ProtocolFactory::getOrCreateProtocol(MONACO_XBEE);
        interremote_->setPairingCallback([this](Protocol* p,const NodeDescription& n) {this->protocolPairedCB(p, n);});
    }

    if(interremote_ == nullptr) {
        bb::printf("Couldn't load, get, or create protocol of type %c\n", MONACO_XBEE);
        return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    } 
        
    bb::printf("Initializing interremote protocol at 0x%x\n", interremote_);
    interremote_->init(isLeftRemote ? "LeftRemote" : "RightRemote");


    if(ProtocolFactory::lastUsedProtocolName() != INTERREMOTE_PROTOCOL_NAME) {
        bb::printf("Loading protocol %s\n", ProtocolFactory::lastUsedProtocolName().c_str());
        current_ = ProtocolFactory::loadLastUsedProtocol();
        if(current_ == nullptr) {
            bb::printf("Loading failed, using interremote\n");
            current_ = interremote_;
        }
    } else {
        bb::printf("Last stored protocol is interremote, so just using that.\n");
        current_ = interremote_;
    }

    initialized_ = true;

    return Subsystem::initialize("rss", "Remote Subsystem",  HELP);
}

Result RemoteSubsys::start() {
    return Subsystem::start();
}

Result RemoteSubsys::stop() {
    return Subsystem::stop();
}

bool RemoteSubsys::storeCurrent(const std::string& name) {
    currentName_ = name;
    return ProtocolFactory::storeProtocol(name, current_);
}

bool RemoteSubsys::storeInterremote() {
    return ProtocolFactory::storeProtocol(INTERREMOTE_PROTOCOL_NAME, interremote_);
}

Protocol* RemoteSubsys::createProtocol(ProtocolType type) {
    if(type == interremote_->protocolType()) {
        return nullptr;
    }
    Protocol *proto = ProtocolFactory::getOrCreateProtocol(type);
    if(proto == nullptr) {
        return nullptr;
    }
    if(current_ != nullptr && current_ != interremote_) {
        delete current_;
    }
    current_ = proto;
    current_->init(isLeftRemote ? "LeftRemote" : "RightRemote");
    current_->setStorageName(nextProtocolName());
    current_->setPairingCallback([this](Protocol* p,const NodeDescription& n) {this->protocolPairedCB(p, n);});

    if(currentChangedCB_ != nullptr) {
        currentChangedCB_(current_);
    }

    return current_;
}

std::string RemoteSubsys::nextProtocolName() {
    int idx = 1;
    std::string name;
    while(true) {
        name = "Config" + std::to_string(idx);
        std::vector<std::string> storedNames = ProtocolFactory::storedProtocolNames();
        bool found = false;
        for(auto n: storedNames) {
            if(n == name) {
                found = true;
                break;
            } 
        }
        if(current_->storageName() == name) {
            found = true;
        }
        if(!found) {
            return name;
        }
        idx++;
    }
}



Result RemoteSubsys::step() {
    if(interremote_ == nullptr) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    interremote_->step();
    if(current_ != nullptr && current_ != interremote_) {
        current_->step();
    }
    return RES_OK;
}

Result RemoteSubsys::handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream) {
    if(words.size() == 0) return RES_CMD_INVALID_ARGUMENT;

    if(words[0] == "list") {
        std::vector<std::string> storedNames = ProtocolFactory::storedProtocolNames();
        stream->printf("Stored protocols: %d\n", storedNames.size());
        for(auto s: storedNames) stream->printf("\t\"%s\"\n", s.c_str());
        return RES_OK;
    }

    if(words[0] == "print_storage") {
        if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;

        ProtocolFactory::printStorage();
        return RES_OK;
    }

    if(words[0] == "commit_storage") {
        if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;

        ProtocolFactory::commit();
        return RES_OK;
    }

    if(words[0] == "erase") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

        if(words[1] == "all") {
            if(ProtocolFactory::eraseAll() == true) {
                return RES_OK;
            } else {
                return RES_SUBSYS_HW_DEPENDENCY_MISSING;
            }
        } else {
            if(ProtocolFactory::eraseProtocol(words[1].c_str()) == true) {
                return RES_OK;
            } else {
                return RES_SUBSYS_HW_DEPENDENCY_MISSING;
            }
        }
    }

    if(words[0] == "create") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

        ProtocolType type = (ProtocolType)words[1][0];
        if(type == interremote_->protocolType()) {
            stream->printf("Refuse to create new protocol of type '%c' as it's also the interremote protocol.\n", type);
            return RES_CMD_INVALID_ARGUMENT;
        }
        Protocol *proto = createProtocol(type);
        if(proto == nullptr) {
            stream->printf("createProtocol() returns nullptr\n");
            return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
        }
        return RES_OK;
    }

    if(words[0] == "load") {
        if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
        const String& name = words[2];
        
        if(words[1] == "current") {
            Protocol *proto = ProtocolFactory::loadProtocol(name.c_str());
            if(proto == nullptr) {
                stream->printf("Protocol is null\n");
                return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
            }

            if(current_ != proto && current_ != interremote_) {
                stream->printf("Destroying current\n");
                ProtocolFactory::destroyProtocol(&current_);
            } else {
                if(current_ == proto) { stream->printf("not deleting, the new one is the current_\n"); }
                if(current_ == interremote_) { stream->printf("not deleting, the old one is the interremote_\n"); }
            }
            current_ = proto;
            
            if(currentChangedCB_ != nullptr) currentChangedCB_(current_);
        } else {
            stream->printf("Changing inter currently not implemented\n");
        }

        return RES_OK;
    }

    if(words[0] == "store") {
        if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;

        Protocol *proto = (words[1]=="current" ? current_ : (words[1]=="inter" ? interremote_ : nullptr));
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

        Protocol *proto = (words[1]=="current" ? current_ : (words[1]=="inter" ? interremote_ : nullptr));

        if(proto == nullptr) {
            stream->printf("Protocol is null\n");
            return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
        }
        proto->discoverNodes();
        stream->printf("%d nodes discovered\n", proto->numDiscoveredNodes());
        for(int i=0; i<current_->numDiscoveredNodes(); i++) {
            const NodeDescription& descr = proto->discoveredNode(i);
            stream->printf("    Name: %s Addr: %s (configurator: %s receiver: %s transmitter: %s)\n", 
                std::string(descr.name).c_str(), descr.addr.toString().c_str(),
                descr.isConfigurator ? "true" : "false",
                descr.isReceiver ? "true" : "false",
                descr.isTransmitter ? "true" : "false");
        }
        return RES_OK;
    }

    if(words[0] == "pair") {
        if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;

        Protocol *proto = (words[1]=="current" ? current_ : (words[1]=="inter" ? interremote_ : nullptr));
        if(proto == nullptr) {
            stream->printf("%s is nullptr.\n", words[1].c_str());
            return RES_OK;
        }

        if(proto->numDiscoveredNodes() == 0) {
            stream->printf("%s has no discovered nodes.\n", words[1].c_str());
            return RES_OK;
        }

        NodeDescription nd; bool found = false;
        for(int i=0; i<proto->numDiscoveredNodes(); i++) {
            nd = proto->discoveredNode(i);
            if(nd.name == words[2] || nd.addr.toString() == words[2].c_str()) {
                found = true;
                break;
            }
        }
        if(found == false) {
            stream->printf("Couldn't find %s in list of discovered nodes - try \"info\".", words[2].c_str());
            return RES_OK;
        }
        stream->printf("Pairing with %s... ", words[2].c_str());
        if(proto->pairWith(nd) == true) {
            stream->printf("Success.\n");
            return RES_OK;
        } else {
            stream->printf("Error.\n");
            return RES_SUBSYS_COMM_ERROR;
        }
    }

    if(words[0] == "info") {
        if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

        Protocol *proto = (words[1]=="current" ? current_ : (words[1]=="inter" ? interremote_ : nullptr));
        if(proto == nullptr) {
            stream->printf("%s is nullptr.\n", words[1].c_str());
            return RES_OK;
        }
        proto->printInfo();
        return RES_OK;
    }

    return Subsystem::handleConsoleCommand(words, stream);
}


void RemoteSubsys::setCurrentChangedCB(std::function<void(Protocol*)> currentChangedCB) {
    currentChangedCB_ = currentChangedCB;
}

bool RemoteSubsys::loadCurrent(const std::string& name) {
    Protocol *proto = ProtocolFactory::loadProtocol(name.c_str());
    if(proto == nullptr) {
        bb::printf("Protocol is null\n");
        return false;
    }

    if(current_ != proto && current_ != interremote_) {
        bb::printf("Destroying current\n");
        ProtocolFactory::destroyProtocol(&current_);
    } else {
        if(current_ == proto) { bb::printf("not deleting, the new one is the current_\n"); }
        if(current_ == interremote_) { bb::printf("not deleting, the old one is the interremote_\n"); }
    }
    current_ = proto;
            
    if(currentChangedCB_ != nullptr) currentChangedCB_(current_);

    return true;
}

void RemoteSubsys::protocolPairedCB(Protocol* proto, const NodeDescription& node) {
    if(proto == interremote_) {
        bb::printf("New node paired with interremote: %s\n", node.addr.toString().c_str());
        storeInterremote();
        ProtocolFactory::commit();
    } else if(proto == current_) {
        bb::printf("New node paired with current: %s\n", node.addr.toString().c_str());
        if(current_->storageName() == "") {
            bb::printf("Can't store current as storage name is \"\"\n");
        } else {
            storeCurrent(current_->storageName());
            ProtocolFactory::commit();
        }   
    } else {
        bb::printf("Received pairing CB from protocol %x, which is neither current nor inter - ignoring\n", proto);
    }
}
