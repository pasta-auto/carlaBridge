#ifndef CARLA_ROS2_AGENT_PASTA_AGENT_HPP
#define CARLA_ROS2_AGENT_PASTA_AGENT_HPP

#include <string>
#include <set>
#include <map>
#include <memory>
#include <netinet/in.h>
#include <fmt/core.h>
#include <fmt/args.h>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "server/remote_process.hpp"

struct PastaEntry
{
    std::string id;
    int period;
    int datasize;
    int totalSize;
    uint32_t datamask;
    uint16_t pastaId;
    std::string format;
    std::set<int> modeReceive;
    std::set<int> modeSend;
    std::map<int, uint16_t> sendPastaIdOverride;
};

struct PastaPublish
{
    std::string value;
    std::shared_ptr<PastaEntry> pastaEntry;
    bool active;
    uint32_t data;
};

enum PastaTurnSignals : uint8_t
{
    PASTA_TURN_NONE = 0,
    PASTA_TURN_LEFT = 1,
    PASTA_TURN_RIGHT = 2,
    PASTA_TURN_RPT_HAZARD = 3,
    PASTA_TURN_CMD_HAZARD = 4
};

enum PastaShiftSignals : uint8_t
{
    PASTA_SHIFT_NONE = 0,
    PASTA_SHIFT_UP = 1,
    PASTA_SHIFT_DOWN = 2
};

enum PastaModes : uint8_t
{
    PASTA_MODE_OFF = 20,
    PASTA_MODE_1 = 1,
    PASTA_MODE_2 = 2,
    PASTA_MODE_6 = 6,
};

enum PastaHeadLights : uint8_t
{
    PASTA_HEADLIGHT_OFF = 0,
    PASTA_HEADLIGHT_POSITION = 1,
    PASTA_HEADLIGHT_LOW = 2,
    PASTA_HEADLIGHT_PASS = 4
};

constexpr int PASTA_MAX_STEERING_MODE_6 = 45;

class pasta_interface{
public:
    ~pasta_interface();
    pasta_interface(std::string format, std::string path);

    int pasta_mode = PASTA_MODE_2;

    const static std::set<std::string> pasta_formats_listings;

    static std::shared_ptr<pasta_interface> currentAgent;
    static std::shared_ptr<pasta_interface> getAgent();
    static void createAgent(std::string format, std::string path);
    static void destroyAgent();

    void start();
    void stop();

    void set_net(int port, std::string address);

    void set(std::string id, u_int32_t value1, u_int32_t value2 = 0, u_int32_t value3 = 0 );
    
    void unset(std::string id);

    void set_mode(int mode);
private:
    const int loopInterval = 10;

    int sockfd_send;
    int sockfd_recv;

    struct sockaddr_in servaddr, clientaddr;

    bool running;
    uint64_t loopRuntime;
    std::mutex frame_lock;
    std::condition_variable frame_cv;

    std::thread loopSendThread;
    std::thread loopReceiveThread;
    std::thread loopTimingThread;

    int idCount;
    std::shared_ptr<PastaPublish>* setValues;
    std::map<std::string, int> setValuesNamedMap;

    std::map<std::string, std::shared_ptr<PastaEntry>> namedEntries;
    std::map<uint16_t, std::shared_ptr<PastaEntry>> idEntries;
    std::map<std::string, std::shared_ptr<PastaPublish>> setPastaValues;

    std::string pasta_format;

    bool load_CAN_ID(std::string path, std::string format);

    //Module processing system
    remote_process apiRPC; 

    void loopSend();
    void loopReceive();
    void loopTiming();
};

#endif