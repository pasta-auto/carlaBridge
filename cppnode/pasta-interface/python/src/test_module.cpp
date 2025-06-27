
#include <boost/python.hpp>
#include "test_module.hpp"

using namespace boost::python;

BOOST_PYTHON_MODULE(test_module)
{
    class_<PythonTestModuleWrapper, boost::noncopyable>("PythonTestModuleWrapper")
    ;

}

