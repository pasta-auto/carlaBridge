
#include <boost/python.hpp>
#include <mutex>
#include "can_module.hpp"
#include <memory>

using namespace boost::python;

//global in this file to access through dlsym
std::shared_ptr<std::mutex> can_filter_mutex;
std::shared_ptr<std::set<int>> can_id_filter;

std::shared_ptr<std::mutex> can_send_queue_mutex;
std::shared_ptr<std::queue<std::tuple<uint16_t, uint64_t, int>>> can_data_to_send;

extern "C" {
    void filter_add(int id)
    {
        const std::lock_guard<std::mutex> lock(*can_filter_mutex);
        can_id_filter->insert(id);
    }

    void filter_remove(int id)
    {
        const std::lock_guard<std::mutex> lock(*can_filter_mutex);
        can_id_filter->erase(id);
    }

    void filter_clear()
    {
        const std::lock_guard<std::mutex> lock(*can_filter_mutex);
        can_id_filter->clear();
    }

    void send_can(int id, int data, int size)
    {
        const std::lock_guard<std::mutex> lock(*can_send_queue_mutex);
        can_data_to_send->push(std::make_tuple(id, data, size));
    }

    std::set<int> filter_get()
    {
        const std::lock_guard<std::mutex> lock(*can_filter_mutex);
        return *can_id_filter;
    }

    bool filter_exists(int id)
    {
        const std::lock_guard<std::mutex> lock(*can_filter_mutex);
        return can_id_filter->count(id);
    }

    std::queue<std::tuple<uint16_t, uint64_t, int>> send_get(int max_msgs)
    {
        const std::lock_guard<std::mutex> lock(*can_filter_mutex);
        std::queue<std::tuple<uint16_t, uint64_t, int>> tmp = *can_data_to_send;

        //Clear the queue after we copied it over
        can_data_to_send = std::make_shared<std::queue<std::tuple<uint16_t, uint64_t, int>>>();
        
        return tmp;
    }

    void module_init()
    {
        can_filter_mutex = std::make_shared<std::mutex>();
        can_send_queue_mutex = std::make_shared<std::mutex>();
        can_id_filter = std::make_shared<std::set<int>>();
        can_data_to_send = std::make_shared< std::queue<std::tuple<uint16_t, uint64_t, int>> >();
    }
}

BOOST_PYTHON_MODULE(can_module)
{
    class_<PythonCANModuleWrapper, boost::noncopyable>("PythonCANModuleWrapper")
    .def("filter_add", filter_add)
    .def("filter_remove", filter_remove)
    .def("filter_clear", filter_clear)
    .def("filter_exists", filter_exists)
    .def("send_can", send_can);
}

