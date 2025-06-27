#include <dlfcn.h>
#include <stdlib.h>
#include <stdio.h>
#include <boost/dll.hpp>
#include <iostream>
#include <filesystem>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "camera_module.hpp"
#include <thread>

using namespace boost::python;
namespace np = boost::python::numpy;

#define INIT_MODULE PyInit_camera_module
extern "C" PyObject* INIT_MODULE();

bool inited = false;
np::ndarray initBoostPython() {
     // When you load a NumPy-dependent plugin, do the following just before that.
    //dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
    // also need to add our global python module
    //dlopen("camera_module.so", RTLD_LAZY | RTLD_GLOBAL);
    // and initialize our global python module and initialize python and boost
    PyImport_AppendInittab((char*)"camera_module", INIT_MODULE);
    Py_Initialize();
    np::initialize();

    // --------- specific to example maybe try moving to a C++ so to try and simulating loading w/e data
    tuple shape = make_tuple(303, 384, 4); // shape of scikit's built in coins dataset
    np::dtype dtype = np::dtype::get_builtin<uint8_t>();
    return std::move(np::zeros(shape, dtype));
    // ----------------------------------------
}

void runSO(const std::string& soFile, boost::python::numpy::ndarray& arr) {
    void *handle;
    void (*run)(np::ndarray & arr); //TODO func signature dependent on module type
    char *error;
    handle = dlopen(soFile.c_str(), RTLD_NOW);
    if (!handle) {
        fprintf(stderr, "%s\n", dlerror());
        exit(EXIT_FAILURE);
    }

    dlerror();    /* Clear any existing error */

    module_info* info = (module_info*)dlsym(handle, "info");
    if (info) {
        std::cout << *info << std::endl;
    } else {
        std::cout << "Could not load module info" << std::endl;
    }
    

    run = (void (*)(boost::python::numpy::ndarray&))dlsym(handle, "run");

    if ((error = dlerror()) != NULL)  {
        boost::dll::library_info inf(soFile.c_str());
        std::vector<std::string> exports = inf.symbols();
        for (std::size_t j = 0; j < exports.size(); ++j) {
            std::cout << exports[j] << std::endl;
        }
    }

    run(arr);
    dlclose(handle);
}

int main(int argc, char** argv)
{
    std::filesystem::path cwd = std::filesystem::current_path();
    std::cout << "Running test from: " << cwd << std::endl;
    
    auto runThread = std::thread([] {
    auto arr = initBoostPython();
    //runSO("example/exPyModule.so", arr);
    runSO("example/loadData.so", arr);
    runSO("example/printShape.so", arr);
    runSO("example/scale.so", arr);
    runSO("example/show.so", arr);
    runSO("example/setBtoZero.so", arr);
    runSO("example/show.so", arr);
    });
    runThread.join();
    
    exit(EXIT_SUCCESS);
}