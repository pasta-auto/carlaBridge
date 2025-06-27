
#include <boost/python.hpp>
#include "radar_module.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(radar_module)
{
    class_<PythonRadarModuleWrapper, boost::noncopyable>("PythonRadarModuleWrapper")
    ;

}

