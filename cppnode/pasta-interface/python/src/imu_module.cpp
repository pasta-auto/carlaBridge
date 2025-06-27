
#include <boost/python.hpp>
#include "imu_module.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(imu_module)
{
    class_<PythonIMUModuleWrapper, boost::noncopyable>("PythonIMUModuleWrapper")
    ;

}

