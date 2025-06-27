#include "../python/include/gil_guard.hpp"
#include "../python/include/camera_module.hpp"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <Python.h>
#include "numpy/arrayobject.h" // Include any other Numpy headers, UFuncs for example.
#include <mutex>

using namespace boost::python;
namespace np = boost::python::numpy;

#define INIT_MODULE PyInit_camera_module
extern "C" PyObject* INIT_MODULE();

//#define INIT_IN_THREAD


constexpr size_t width  = 640;
constexpr size_t height = 480;

std::mutex m;
std::mutex runLock;
int testNoSOLoad(PyInterpreterState* state)
{
  try {
    m.lock();
    #ifdef INIT_IN_THREAD
        PyImport_AppendInittab((char*)"camera_module", INIT_MODULE);
        Py_Initialize();
        np::initialize();

        
        // Initialise Numpy
        //import_array();
    #endif
  uint8_t rawData[width*height*4];
  //gil_guard gil;
  //PyGILState_STATE state = PyGILState_Ensure();
  
  PyThreadState* m_state = PyThreadState_New(state);
  PyEval_RestoreThread(m_state);
  //PyGILState_STATE gilState = PyGILState_Ensure();
  m.unlock();

  tuple shape  = make_tuple(height, width, 4);
  //tuple shape  = make_tuple(303, 384, 4);
  tuple stride = make_tuple(width*4*sizeof(uint8_t),4*sizeof(uint8_t),1*sizeof(uint8_t));
  object own;
  np::dtype dt = np::dtype::get_builtin<uint8_t>();
  np::ndarray fromDarray = np::from_data(rawData, dt, shape, stride, object());

  object main_module = import("__main__");
  dict main_namespace = extract<dict>(main_module.attr("__dict__"));
  object python_module = import("camera_module"); // TODO name of c++ base class needs to be generated
  dict python_module_namespace = extract<dict>(python_module.attr("__dict__"));

  // TODO this has an awful segfault if file doesn't exist
  //exec_file( str("/pasta-interface/example/loadData.py"), main_namespace, main_namespace);
  exec_file( str("/pasta-interface/example/rotate.py"), main_namespace, main_namespace);
  object BaseClass = python_module_namespace["PythonCameraModuleWrapper"]; // TODO name of wrapper class needs to be generated
  PyTypeObject* base_class = reinterpret_cast<PyTypeObject*>(BaseClass.ptr());

  // TODO this section probably changes depending on above class
  list keys = main_namespace.keys();
  for (unsigned int i = 0; i<len(keys) ; ++i) {
      object k = keys[i];
      object item = main_namespace[k];
      PyObject* item_ptr = item.ptr();
      if ( PyType_Check(item_ptr) != 0 ) {
          PyTypeObject* type_obj = reinterpret_cast<PyTypeObject*>(item_ptr);
          if ( ( type_obj != base_class) && ( PyType_IsSubtype( type_obj,  base_class) > 0)) {
              object obj = item();
              const PythonCameraModuleWrapper& base_obj = extract<PythonCameraModuleWrapper>(obj);
              runLock.lock();
              base_obj.run(fromDarray); // TODO exact function names will need to be generated
              runLock.unlock();
              std::cout << "Returned from python function" << std::endl;
          }
      } 
    }
    //PyGILState_Release(gilState);
    //PyThreadState* state_ = PyEval_SaveThread();
  } catch(error_already_set &){ // failed to create array
        PyObject *ptype, *pvalue, *ptraceback;
        PyErr_Fetch(&ptype, &pvalue, &ptraceback);
        std::string strErrorMessage = extract<std::string>(pvalue);
        std::cerr << "Could not process pipeline:\n" << strErrorMessage << std::endl;
  }
}

int main()
{
    #ifndef INIT_IN_THREAD
    PyImport_AppendInittab((char*)"camera_module", INIT_MODULE);
    Py_Initialize();
    np::initialize();
    PyThreadState* state = PyThreadState_Get();
    PyEval_SaveThread();
    #endif
    std::thread t1(testNoSOLoad, state->interp);
    //std::thread t2(testNoSOLoad, state->interp);
    t1.join();
    //t2.join();
}