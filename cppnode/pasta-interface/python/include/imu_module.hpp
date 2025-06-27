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
struct imu_module {
    virtual void run(np::ndarray& gyro, np::ndarray& accel, np::ndarray& location, np::ndarray& rotation) const {}
};
#pragma GCC diagnostic pop
//////////////////////////////////////////

// This portion is where we would put the python API wrapper around above
struct PythonIMUModuleWrapper : imu_module, wrapper<imu_module>
{
    virtual void run(np::ndarray& gyro, np::ndarray& accel, np::ndarray& location, np::ndarray& rotation) const
    {
        if (override n = this->get_override("run")) {
            n(gyro, accel, location, rotation);
        }
    }
};

// For anything using above API need to have line like this matching the base class
extern "C" PyObject* PyInit_imu_module();
