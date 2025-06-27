
#include <boost/python.hpp>
#include "gnss_module.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(gnss_module)
{
    class_<PythonGNSSModuleWrapper, boost::noncopyable>("PythonGNSSModuleWrapper")
    ;

}

