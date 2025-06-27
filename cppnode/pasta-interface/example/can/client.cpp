#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <fstream>
#include <unistd.h>

//#include "server/remote_process.hpp"
#include "../../server/include/remote_process.hpp"


//#include "lodepng/lodepng.h"

int main(int argc, const char *argv[]) {
    remote_process apiRPC; 

    int count = 0;

    int id = 1, data = 2, size = 1;

    while(true)
    {
      std::queue<std::tuple<uint16_t, uint64_t, int>> send_data;

      apiRPC.can_process(id, data, size);
      apiRPC.can_process_send(send_data);

      while(!send_data.empty())
      {
        auto msg = send_data.front();
        send_data.pop();
        std::cout << "SEND ID: " << std::get<0>(msg) << " data: " << std::get<1>(msg) << " size: " << std::get<2>(msg) << std::endl;
      }

      std::cout << "ID: " << id << " data: " << data << " size: " << size << std::endl;
      usleep(5*100000);
    }
}