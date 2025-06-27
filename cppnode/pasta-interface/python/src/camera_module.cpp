
#include <boost/python.hpp>
#include "camera_module.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(camera_module)
{
    class_<PythonCameraModuleWrapper, boost::noncopyable>("PythonCameraModuleWrapper")
    ;

}

