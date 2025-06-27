#include <pasta_agent/pasta_interface.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <fmt/core.h>
#include <fmt/args.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>
#include <map>
#include <sstream>
#include <pasta_agent/ros2_agent.hpp>

using namespace std::chrono_literals;

std::shared_ptr<pasta_interface> pasta_interface::currentAgent;

const std::set<std::string> pasta_interface::pasta_formats_listings(
{
    "serial_format",
    "can_format"
});

pasta_interface::pasta_interface(std::string format, std::string path) 
{
    load_CAN_ID(path, format);
}

pasta_interface::~pasta_interface()
{

}

std::shared_ptr<pasta_interface> pasta_interface::getAgent()
{
    return currentAgent;
}

void pasta_interface::createAgent(std::string format, std::string path)
{
    currentAgent = std::make_shared<pasta_interface>(format, path);
}

void pasta_interface::destroyAgent()
{
    currentAgent.reset();
}

void pasta_interface::set_net(int port, std::string addr)
{
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&clientaddr, 0, sizeof(clientaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port + 1);
    
    clientaddr.sin_family = AF_INET;
    clientaddr.sin_port = htons(port);
    clientaddr.sin_addr.s_addr = inet_addr(addr.c_str());

    std::cout << "Setting PASTA interface to " << addr << ":" << port << std::endl;
}

void pasta_interface::start()
{
    // Creating socket file descriptor
    if ( (sockfd_send = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        std::cerr << "send socket creation failed with " << errno << std::endl;
        throw("send socket creation failed");
    }

    // Creating socket file descriptor
    if ( (sockfd_recv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
        std::cerr << "recv socket creation failed with " << errno << std::endl;
        throw("recv socket creation failed");
    }
    
    if (bind(sockfd_recv, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        std::cerr << "socket bind failed with " << errno << std::endl;
        throw("socket bind failed");
    }

    loopRuntime = 0;

    running = true;

    loopReceiveThread = std::thread(&pasta_interface::loopReceive, this);
    loopSendThread = std::thread(&pasta_interface::loopSend, this);
    loopTimingThread = std::thread(&pasta_interface::loopTiming, this);

    std::cout << "Starting pasta..." << std::endl;
}

void pasta_interface::loopSend()
{
    std::unique_lock<std::mutex> flck(frame_lock);
    while(running)
    {
        frame_cv.wait_for(flck, 60s);

        if (!running)
        {
            break;
        }

        for(int loop = 0; loop < idCount; loop++)
        {
            std::shared_ptr<PastaPublish> item = setValues[loop];
            
            //Check is item has been set
            if (!item)
            {
                continue;
            }

            //Check if item is empty
            if (!item->active)
            {
                continue;
            }

            //Mode check if allowed to send
            if (item->pastaEntry->modeSend.find(pasta_mode) == item->pastaEntry->modeSend.end())
            {
                continue;
            }

            if (item->pastaEntry->period == 0)
            {
                sendto(sockfd_send, item->value.c_str(), item->value.size(), 0, (sockaddr*)&clientaddr, sizeof(clientaddr));
                item->active = false;
                apiRPC.can_process(item->pastaEntry->pastaId, item->data, item->pastaEntry->totalSize);
            }
            else if (loopRuntime % item->pastaEntry->period == 0)
            {
                sendto(sockfd_send, item->value.c_str(), item->value.size(), 0, (sockaddr*)&clientaddr, sizeof(clientaddr));
                apiRPC.can_process(item->pastaEntry->pastaId, item->data, item->pastaEntry->totalSize);
            }
        }

        loopRuntime += loopInterval;

        //Send can message from api modules 
        {
            std::queue<std::tuple<uint16_t, uint64_t, int>> send_data;

            apiRPC.can_process_send(send_data);

            while(!send_data.empty())
            {
                auto msg = send_data.front();
                send_data.pop();

                int id = std::get<0>(msg);
                int data = std::get<1>(msg);
                int size = std::get<2>(msg);

                uint32_t datamask = 0;
                for (int i = 0; i < size; i++)
                {
                    datamask <<= 8;
                    datamask |= 0xff;
                }

                std::string wire_msg = fmt::format(
                    fmt::runtime(pasta_format), 
                    data & datamask,
                    fmt::arg("id", id), 
                    fmt::arg("datasize", size * 2));

                sendto(sockfd_send, wire_msg.c_str(), wire_msg.size(), 0, (sockaddr*)&clientaddr, sizeof(clientaddr));
            }
        }
    }
}

void pasta_interface::loopReceive()
{
    char buffer[128];

    while(running)
    {
        ssize_t msg_len;

        msg_len = recvfrom(sockfd_recv, buffer, sizeof(buffer), 0, nullptr, 0);

        if (msg_len == 1)
        {
            continue;
        }

        std::stringstream ss(buffer);
        
        int id;
        int size;
        int data;
        
        ss >> std::hex >> id;

        //Check if in mapped entries
        if (idEntries.find(id) == idEntries.end())
        {
            continue;
        }

        auto item = idEntries[id];

        if (item->modeReceive.find(pasta_mode) == item->modeReceive.end())
        {
            continue;
        }

        ss >> std::hex >> size;
        ss >> std::hex >> data;

        //processes data through modules
        apiRPC.can_process(id, data, size);

        //Storage for control command
        static uint16_t breakCmd;
        static uint16_t throttleCmd;
        static uint16_t steerCmd;

        //Check that rcpcpp is alive
        if (!rclcpp::ok())
        {
            return;
        }

        uint8_t shiftCmd;
        uint8_t turnLights;
        uint8_t headlights;
        switch(id)
        {
            case 0x1a:  //breakCmd
                breakCmd = data & 0xffff;
                Ros2Agent::getAgent()->pub_control_command(breakCmd, throttleCmd, steerCmd);
                break;
            case 0x2f:  //throttleCmd
                throttleCmd = data & 0xffff;
                Ros2Agent::getAgent()->pub_control_command(breakCmd, throttleCmd, steerCmd);
                break;
            case 0x58:  //steerCmd
                steerCmd = data & 0xffff;
                Ros2Agent::getAgent()->pub_control_command(breakCmd, throttleCmd, steerCmd);
                break;
            case 0x6d:  //shiftCmd
                shiftCmd = (data & 0xff00) >> 8;
                Ros2Agent::getAgent()->pub_shift_command(shiftCmd);
                break;
            case 0x83:  //turnLightsCmd
                turnLights = (data & 0xff00) >> 8;
                Ros2Agent::getAgent()->pub_turnsignal_command(turnLights);
                break;
            case 0x1c9: //HandbrakeCmd
                Ros2Agent::getAgent()->pub_handbrake_command(data & 0xff00);
                break;
            case 0x1a7: //HeadlightsCmd
                headlights = (data & 0xff00) >> 8;
                Ros2Agent::getAgent()->pub_headlights_command(headlights);
                break;
        }
    }
}

void pasta_interface::loopTiming()
{
    std::chrono::milliseconds waitTime = std::chrono::milliseconds(loopInterval);

    while (running)
    {
        std::this_thread::sleep_for(waitTime);

        frame_cv.notify_all();
    }
}

void pasta_interface::stop()
{
    running = false;
    setPastaValues.clear();

    shutdown(sockfd_send, SHUT_RDWR);
    shutdown(sockfd_recv, SHUT_RDWR);

    loopReceiveThread.join();
    loopSendThread.join();
    loopTimingThread.join();
    
    sockfd_send = 0;
    sockfd_recv = 0;
}

void pasta_interface::set(std::string id, u_int32_t value1, u_int32_t value2, u_int32_t value3 )
{
    using namespace fmt::literals;

    auto entry = std::make_shared<PastaPublish>();

    try 
    {
        int lookup = setValuesNamedMap[id];
        entry->pastaEntry = namedEntries[id];
        uint32_t datamask = entry->pastaEntry->datamask;
        uint16_t id;
        if (auto search = entry->pastaEntry->sendPastaIdOverride.find(pasta_mode); search != entry->pastaEntry->sendPastaIdOverride.end()) {
            id = entry->pastaEntry->sendPastaIdOverride[pasta_mode];
        } else {
            id =entry->pastaEntry->pastaId;
        }
        entry->value = fmt::format(
            fmt::runtime(entry->pastaEntry->format), 
            value1 & datamask, value2 & datamask, value3 & datamask,
            fmt::arg("id", id), 
            fmt::arg("datasize", entry->pastaEntry->datasize * 2));
        entry->active = true;
        entry->data = (value1 & datamask) | ((value2 & datamask) << entry->pastaEntry->datasize) | ((value3 & datamask) << entry->pastaEntry->datasize*2);

        if (!setValues[lookup] || entry->value != setValues[lookup]->value)
        {
            setValues[lookup].swap(entry);
        }
    }
    catch(...)
    {
        throw("Error setting " + id);
    }

    // setPastaValues[id] = entry;
}


void pasta_interface::unset(std::string id)
{
    try 
    {
        int lookup = setValuesNamedMap[id];
        setValues[lookup].reset();
    }
    catch(...)
    {
        throw("Error unsetting " + id);
    }

    // setPastaValues.erase(id);
}

inline bool pasta_interface::load_CAN_ID(std::string path, std::string format)
{
    std::cout << "PASTA Config - format:" << format << " file:" << path << std::endl;

    using json = nlohmann::json;

    std::ifstream ifs(path);
    json objects = json::parse(ifs);

    //Sanity check
    if (!objects.contains("ids"))
    {
        std::cerr << "Error loading can_id from json " << path << " file" << std::endl;
        return false;
    }

    if (!objects.contains(format + "_format"))
    {
        std::cerr << "Error loading " << format << "_format from json " << path << " file, using defaults" << std::endl;
        pasta_format =  "{:03x} {:0{datasize * 2}x}";
    }
    else{
        pasta_format = objects[format + "_format"];
    }

    json ids = objects["ids"];

    idCount = -1;
    for(auto item : ids.items()) 
    {
        idCount++;

        std::string id = item.key();

        std::shared_ptr<PastaEntry> pastaEntry = std::make_shared<PastaEntry>();

        pastaEntry->id = id;

        pastaEntry->format = pasta_format;
        //Check if there is a override
        {
            std::string format_name = "override_" + format + "_format";
            if (item.value().contains(format_name))
            {
                if (item.value()[format_name].is_string())
                {
                    if (!item.value()[format_name].empty()) 
                    {
                        pastaEntry->format = item.value()[format_name];
                    }
                }    
            } 
        }

        //Populate CanEntry struct
        if (item.value().contains(format + "Id"))
        {
            if (item.value()[format + "Id"].is_string())
            {
                try {
                    std::string id = item.value()[format + "Id"];
                    pastaEntry->pastaId = std::stoi(id, nullptr, 0);
                } 
                catch(...) {
                    std::cerr << "Failed to parse " << format << "Id for " << id << ", skipping" << std::endl;
                    continue;
                }
            }
            else
            {
                pastaEntry->pastaId = item.value()[format + "Id"];
            }
        }

        if (item.value().contains("period") && item.value()["period"].is_number())
        {
            pastaEntry->period = item.value()["period"];
        }
        if (item.value().contains("datasize") && item.value()["datasize"].is_number())
        {
            pastaEntry->datasize = item.value()["datasize"];
            pastaEntry->datamask = 0;
            
            for (int i = 0; i < pastaEntry->datasize; i++)
            {
                pastaEntry->datamask <<= 8;
                pastaEntry->datamask |= 0xff;
            }
        }
        if (item.value().contains("totalsize") && item.value()["totalsize"].is_number())
        {
            pastaEntry->totalSize = item.value()["totalsize"];
        }
        else {
            pastaEntry->totalSize = pastaEntry->datasize;
        }
        if (item.value().contains("modeSend") && item.value()["modeSend"].is_array())
        {
            for(auto mode : item.value()["modeSend"])
            {
                pastaEntry->modeSend.insert((int)mode);
            }
        }
        if (item.value().contains("modeReceive") && item.value()["modeReceive"].is_array())
        {
            for(auto mode : item.value()["modeReceive"])
            {
                pastaEntry->modeReceive.insert((int)mode);
            }
        }

        if (item.value().contains("sendCanIdOverride") && item.value()["sendCanIdOverride"].is_array()) {
            for (auto modeAndId: item.value()["sendCanIdOverride"]) {
                if (modeAndId.contains("mode") && modeAndId.contains("id")) {
                    int      overrideMode = modeAndId["mode"];
                    uint16_t overrideId   = std::stoi((const std::string&)modeAndId["id"], nullptr, 0);
                    pastaEntry->sendPastaIdOverride[overrideMode] = overrideId;
                    pastaEntry->modeSend.insert(overrideMode);
                }
            }
        }

        //Allow lookup based on named entry
        namedEntries[id] = pastaEntry;

        //Allow lookup based on id, serial or can
        idEntries[pastaEntry->pastaId] = pastaEntry;

        //Create mapping for pasta sending and multithreading
        setValuesNamedMap[id] = idCount;
    }

    // setValues.resize(idCount + 1);
    idCount++;
    setValues = new std::shared_ptr<PastaPublish>[idCount];
    return true;
}

void pasta_interface::set_mode(int mode)
{
    std::cout << "Set Pasta Mode " << mode << std::endl;
    pasta_mode = mode;
}