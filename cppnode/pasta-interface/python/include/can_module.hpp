#include "pasta_interface.hpp"
#include <boost/python/numpy.hpp>
#include <set>
#include <queue>
#include <tuple>
namespace np = boost::python::numpy;
using namespace boost::python;


////////////////////////////////////////
// I expect this portion to be w/e c++ API will include here 
// then will add on the python specific things needed to get the python working
// TODO is there 
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
struct can_module {
    virtual void run(np::ndarray & arr) const {}
};

void can_module_init();
#pragma GCC diagnostic pop
//////////////////////////////////////////

// This portion is where we would put the python API wrapper around above
struct PythonCANModuleWrapper : can_module, wrapper<can_module>
{
    virtual void run(np::ndarray & arr) const
    {
        if (override n = this->get_override("run")) {
            n(arr);
        }
    }
};

// For anything using above API need to have line like this matching the base class
extern "C" PyObject* PyInit_can_module();
