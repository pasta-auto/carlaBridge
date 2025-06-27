#include "pasta_interface.hpp"
#include <boost/python/numpy.hpp>
namespace np = boost::python::numpy;
using namespace boost::python;

// TODO what should a high level module look like?

////////////////////////////////////////
// I expect this portion to be w/e c++ API will include here 
// then will add on the python specific things needed to get the python working
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
struct lidar_module {
    virtual void run(np::ndarray & arr) const {}
};
#pragma GCC diagnostic pop
//////////////////////////////////////////

// This portion is where we would put the python API wrapper around above
struct PythonLiDARModuleWrapper : lidar_module, wrapper<lidar_module>
{
    virtual void run(np::ndarray & arr, np::ndarray & header) const
    {
        if (override n = this->get_override("run")) {
            n(arr, header);
        }
    }
};

// For anything using above API need to have line like this matching the base class
extern "C" PyObject* PyInit_lidar_module();
