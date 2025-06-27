
#include <boost/python.hpp>
#include "lidar_module.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(lidar_module)
{
    class_<PythonLiDARModuleWrapper, boost::noncopyable>("PythonLiDARModuleWrapper")
    ;

}

