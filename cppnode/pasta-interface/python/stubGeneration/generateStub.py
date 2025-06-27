#!/usr/bin/env python3
import argparse
import sys
import os
import yaml
import logging
import json
from pathlib import Path
import glob
import textwrap
import tempfile
import contextlib
import shutil

Args = None

class CONST_STRS(object):
    __slots__ = ()
    # TODO will neeed to alter this if reorganize the repo (which is likely when merge branches) 
    COMPILE_START = "g++ --shared --std=c++17 -fPIC -DBOOST_ALLOW_DEPRECATED_HEADERS -DBOOST_DISABLE_PRAGMA_MESSAGE "
    # need the python3.10 installing from repo should symlink or something for dev image
    COMPILE_END = \
        "-I"+os.path.abspath(os.path.join(Path(__file__).parent.parent.parent.resolve(), "include ")) + \
        "-I"+os.path.abspath(os.path.join(Path(__file__).parent.parent.resolve()       , "include ")) + \
        "-I/usr/include/ " + \
        "-I/usr/include/python3.10 " + \
        "-lpython3.10 -lcrypt -ldl  -lm -lm " + \
        "-lboost_python3 -lboost_numpy " +  \
        "-L" + os.path.abspath(os.path.join(Path(__file__).parent.parent.parent.resolve(), "build")) +  \
        " -L/autoware/install/pasta-interface/lib/" +  \
    " "
    # python -l from going: python3-config --libs --embed TODO should probably run that and get result in this

    # 1 arg header for c++ type
    HEADER = \
    """
    #include <boost/python.hpp>
    #include <boost/python/numpy.hpp>
    #include <iostream>
    #include <fstream>
    #include <sstream>

    #if __has_include(<experimental/filesystem>)
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
    #elif __has_include(<filesystem>)
    #include <filesystem>
    using fs = std::filesystem;
    #endif
    #include "pasta_interface.hpp"
    #include "{:s}.hpp"

    using namespace boost::python;
    namespace np = boost::python::numpy;
    """

    # 5 args
    # 1 type:
    #   enum class module_type {
    #    Test_Module,
    #    CAN_Data,
    #    AutoEther,
    #    Sensor_GNSS,
    #    Sensor_LiDAR,
    #    Senosr_Radar,
    #    Sensor_CMOS,
    #    UNKNOWN,
    #   }; 
    # 2 api version
    # 3 module version
    # 4 name
    # 5 description
    MODULE_INFO = \
    """
    module_info info(
        module_lang::Python// module_lang _language      
       ,{:s} // module_type _type          
       ,{:d} // uint8_t     _api_version   
       ,{:d} // uint8_t     _module_version
       ,R"({:s})" // std::string _name          
       ,R"({:s})" // std::string _description   
    );
    """

    # TODO need to propbably change out the function signature and can't easily use for example templates as need
    #      extern "C" to not mangle the name (as will be read by dlopen)
    RUN_FUNC_START = \
    """
    extern "C" int run(np::ndarray & arr)
    {
        try {
    """
    CAM_RUN_FUNC_START = \
    """
    #define INIT_MODULE PyInit_camera_module
    extern "C" PyObject* INIT_MODULE();
    extern "C" int run(uint8_t * rawData, size_t width, size_t height)
    {
        try {
            PyImport_AppendInittab((char*)"camera_module", INIT_MODULE);
            Py_Initialize();
            np::initialize();

            tuple shape  = make_tuple(height, width, 4);
            tuple stride = make_tuple(width*4*sizeof(uint8_t),4*sizeof(uint8_t),1*sizeof(uint8_t));
            object own;
            np::dtype dt = np::dtype::get_builtin<uint8_t>();
            np::ndarray arr = np::from_data(rawData, dt, shape, stride, object());
    """

    GNSS_RUN_FUNC_START = \
    """
    #define INIT_MODULE PyInit_gnss_module
    extern "C" PyObject* INIT_MODULE();
    extern "C" int run(double * rawData)
    {
        try {
            PyImport_AppendInittab((char*)"gnss_module", INIT_MODULE);
            Py_Initialize();
            np::initialize();
            list dtype_list;
            dtype_list.append(make_tuple("latitude" , np::dtype::get_builtin<double>()));
            dtype_list.append(make_tuple("longitude", np::dtype::get_builtin<double>()));
            dtype_list.append(make_tuple("altitude" , np::dtype::get_builtin<double>()));
            np::dtype format = np::dtype(dtype_list);
            tuple shape = make_tuple(1);
            tuple stride = make_tuple(1);
            np::ndarray arr = np::from_data(rawData, format, shape, stride, object());
    """

    CAN_RUN_FUNC_START = \
    """
    #define INIT_MODULE PyInit_can_module
    extern "C" PyObject* INIT_MODULE();
    extern "C" int run(uint64_t * rawData)
    {
        try {
            PyImport_AppendInittab((char*)"can_module", INIT_MODULE);
            Py_Initialize();
            np::initialize();
            list dtype_list;
            dtype_list.append(make_tuple("id"  , np::dtype::get_builtin<uint64_t>()));
            dtype_list.append(make_tuple("data", np::dtype::get_builtin<uint64_t>()));
            dtype_list.append(make_tuple("size", np::dtype::get_builtin<uint64_t>()));
            np::dtype format = np::dtype(dtype_list);
            tuple shape = make_tuple(1);
            tuple stride = make_tuple(1);
            np::ndarray arr = np::from_data(rawData, format, shape, stride, object());
    """

    IMU_RUN_FUNC_START = \
    """
    #define INIT_MODULE PyInit_imu_module
    extern "C" PyObject* INIT_MODULE();
    extern "C" int run(float * rawGyr, float * rawAcc, float * rawLoc, float * rawRot)
    {
        try {
            PyImport_AppendInittab((char*)"imu_module", INIT_MODULE);
            Py_Initialize();
            np::initialize();
            list dtype_list;
            dtype_list.append(make_tuple("x", np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("y", np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("z", np::dtype::get_builtin<float>()));
            np::dtype format = np::dtype(dtype_list);
            tuple shape = make_tuple(1);
            tuple stride = make_tuple(1);
            // run(gyr, acc, loc, rot)
            np::ndarray gyr = np::from_data(rawGyr, format, shape, stride, object());
            np::ndarray acc = np::from_data(rawAcc, format, shape, stride, object());
            np::ndarray loc = np::from_data(rawLoc, format, shape, stride, object());
            list dtype_list_rot;
            dtype_list_rot.append(make_tuple("yaw"  , np::dtype::get_builtin<float>()));
            dtype_list_rot.append(make_tuple("pitch", np::dtype::get_builtin<float>()));
            dtype_list_rot.append(make_tuple("roll" , np::dtype::get_builtin<float>()));
            np::dtype rotFormat = np::dtype(dtype_list_rot);
            np::ndarray rot = np::from_data(rawRot, rotFormat, shape, stride, object());
    """

    LIDAR_RUN_FUNC_START = \
    """
    #define INIT_MODULE PyInit_lidar_module
    extern "C" PyObject* INIT_MODULE();
    extern "C" int run(float * rawData, size_t numPoints, float& horizontalAngle, size_t& channels)
    {
        try {
            PyImport_AppendInittab((char*)"lidar_module", INIT_MODULE);
            Py_Initialize();
            np::initialize();
            list dtype_list;
            dtype_list.append(make_tuple("x"        , np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("y"        , np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("z"        , np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("intensity", np::dtype::get_builtin<float>()));
            np::dtype format = np::dtype(dtype_list);

            tuple shape  = make_tuple(numPoints);
            tuple stride = make_tuple(4*sizeof(float));
            object own;
            np::ndarray arr = np::from_data(rawData, format, shape, stride, object());

            list header_dtype_list;
            header_dtype_list.append(make_tuple("horizontalAngle", np::dtype::get_builtin<float>()));
            header_dtype_list.append(make_tuple("channels", np::dtype::get_builtin<size_t>()));
            np::dtype header_format = np::dtype(header_dtype_list);
            tuple header_shape  = make_tuple(1);
            np::ndarray header = np::zeros(header_shape,header_format);
            header["horizontalAngle"] = horizontalAngle;
            header["channels"] = channels;
    """

    RADAR_RUN_FUNC_START = \
    """
    #define INIT_MODULE PyInit_radar_module
    extern "C" PyObject* INIT_MODULE();
    extern "C" int run(float * rawData, size_t numPoints)
    {
        try {
            PyImport_AppendInittab((char*)"radar_module", INIT_MODULE);
            Py_Initialize();
            np::initialize();
            list dtype_list;
            dtype_list.append(make_tuple("altitude", np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("azimuth" , np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("depth"   , np::dtype::get_builtin<float>()));
            dtype_list.append(make_tuple("velocity", np::dtype::get_builtin<float>()));
            np::dtype format = np::dtype(dtype_list);
            tuple shape = make_tuple(numPoints);
            tuple stride = make_tuple(4*sizeof(float));
            np::ndarray arr = np::from_data(rawData, format, shape, stride, object());
    """

    INDENTATION = "        "

    # 6 params to this:
    #  1 base c++ class name
    #  2 full path to python file NOTE: MAKE SURE THIS EXISTS execfile has bad seg fault otherwise
    #  3 boost python wrapper class
    #  4 className
    #  5/6 boost python wrapper class
    #  7 python function to call (should be the one with the override in wrapper class)
    RUN_FUNC_MID = \
    """
            object main_module = import("__main__");
            dict main_namespace = extract<dict>(main_module.attr("__dict__"));
            object python_module = import("{:s}"); // TODO name of c++ base class needs to be generated
            dict python_module_namespace = extract<dict>(python_module.attr("__dict__"));

            // TODO this has an awful segfault if file doesn't exist
            dict newNamespace;
            std::ifstream t("{:s}");
            std::stringstream buf;
            buf << t.rdbuf();
            exec( buf.str().c_str(), newNamespace, newNamespace);
            object BaseClass = python_module_namespace["{:s}"]; // TODO name of wrapper class needs to be generated
            PyTypeObject* base_class = reinterpret_cast<PyTypeObject*>(BaseClass.ptr());

            // TODO this section probably changes depending on above class
            list keys = newNamespace.keys();
            for (unsigned int i = 0; i<len(keys) ; ++i) {{
                object k = keys[i];
                //char const * objName = extract<char const *>(str(k));
                //std::cout << "Name of key: " << std::string(objName) << std::endl;
                object item = newNamespace[k];
                PyObject* item_ptr = item.ptr();
                if ( PyType_Check(item_ptr) != 0 ) {{
                    PyTypeObject* type_obj = reinterpret_cast<PyTypeObject*>(item_ptr);
                    if ( ( type_obj != base_class) && ( PyType_IsSubtype( type_obj,  base_class) > 0) ) {{
                        object obj = item();
                        const {:s}& base_obj = extract<{:s}>(obj);
                        base_obj.{:s}; // TODO exact function names will need to be generated
                    }}
                }}
            }}
    """

    RUN_FUNC_END = \
    """
        } catch (error_already_set& e) {
            PyErr_PrintEx(0);
            return 1;
        }
        return 0;
    }
    """
CONST_STRS = CONST_STRS()

moduleAPI = {}
# TODO pretty hacky to have mix of load form json and not but for quick testing
runFuncDict = {
    "Camera Module":CONST_STRS.CAM_RUN_FUNC_START,
    "GNSS Module"  :CONST_STRS.GNSS_RUN_FUNC_START,
    "IMU Module"   :CONST_STRS.IMU_RUN_FUNC_START,
    "LiDAR Module" :CONST_STRS.LIDAR_RUN_FUNC_START,
    "Radar Module" :CONST_STRS.RADAR_RUN_FUNC_START,
    "CAN Module"   :CONST_STRS.CAN_RUN_FUNC_START,
}
libDict = {
    "Camera Module": " -Wl,--whole-archive -l:camera_module.a -Wl,--no-whole-archive ",
    "GNSS Module"  : " -Wl,--whole-archive -l:gnss_module.a   -Wl,--no-whole-archive ",
    "IMU Module"   : " -Wl,--whole-archive -l:imu_module.a    -Wl,--no-whole-archive ",
    "LiDAR Module" : " -Wl,--whole-archive -l:lidar_module.a  -Wl,--no-whole-archive ",
    "Radar Module" : " -Wl,--whole-archive -l:radar_module.a  -Wl,--no-whole-archive ",
    "CAN Module"   : " -Wl,--whole-archive -l:can_module.a  -Wl,--no-whole-archive ",
}
def loadAPIList(filename='Module_APIs.json'):
    global moduleAPI
    try:
        data = ''
        with open(os.path.join(sys.path[0],filename), 'r') as file:
            data = file.read()
        moduleAPI = json.loads(data)
    except Exception as e:
        logging.error("Could not parse " + filename)
        logging.error(e)

def main():
    argparser = argparse.ArgumentParser(
        description='C++ Stub Generation for Python Module')
    #argparser.add_argument(
    #    '-p', '--port',
    #    metavar='P',
    #    default=3000,
    #    type=int,
    #    help='UDP port to listen to (default: 3000)'
    #)
    argparser.add_argument(
        '--module_apis',
        default='Module_APIs.json',
        help='Path to file containing API information (default Module_APIs.json in current directory)'
    )
    argparser.add_argument(
        '--target_dir',
        default=Path.cwd(),
        help='Directory to read containing module descriptions'
    )
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information'
    )
    argparser.add_argument(
        '-o',
        default=None,
        help='Directory to output .so to'
    )
    argparser.add_argument(
        '--debug_source',
        action='store_true',
        help='Saves output to current directory'
    )
    global Args
    Args = argparser.parse_args()
    
    log_level = logging.DEBUG if Args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    Args.target_dir = os.path.abspath(Args.target_dir)
    loadAPIList(Args.module_apis)

    yamlFiles = glob.glob(os.path.join(Args.target_dir, "*.[yY][aA][mM][lL]")) + glob.glob(os.path.join(Args.target_dir, "*.[yY][mM][lL]"))

    numFiles = len(yamlFiles)
    if numFiles == 0:
        logging.error("No YAML files found in " + Args.target_dir)
        logging.error("Specify directory with --target_dir option")
        quit()

    logging.info("Found " + str(numFiles) + " .yaml files in " + Args.target_dir)
    idx = 0
    for yf in yamlFiles:
        idx += 1
        logging.info("%02d/%02d Processing %s", idx, numFiles, yf)
        yfPath = Path(yf)
        pyFile = yfPath.with_suffix('.py')
        if not os.path.exists(pyFile):
            logging.warning("Could not find associated py file for "+yf+" . Skipping")
            continue
        outputFileName = ""
        if not Args.o:
            outputFileName = yfPath.with_suffix('.so')
        else:
            outputFileName = os.path.join(Args.o, os.path.basename(yfPath.with_suffix('.so')))
        if (os.path.isfile(outputFileName)):
            if (os.path.getmtime(outputFileName) > os.path.getmtime(yfPath)):
                logging.info("      %s exists and is newer; skipping", str(outputFileName))
                continue
        with open(yf, 'r') as stream:
            data = yaml.safe_load(stream)
        # TODO probably add version check as well
        try:
            if not data['type'] in moduleAPI:
                raise
        except Exception as e:
            logging.error("Could not parse type in "+ yf + ".\nThis YAML must contain a type field with one of these types: " + str(list(moduleAPI.keys())))
            continue
        
        name = yfPath.stem if 'name' not in data else data['name']
        desc = yfPath.stem if 'description' not in data else data['description']
        typeStr   = moduleAPI[data['type']]['type']
        header    = moduleAPI[data['type']]['baseClass']
        baseClass = moduleAPI[data['type']]['baseClass']
        wrapClass = moduleAPI[data['type']]['wrapperClass']
        apiVer    = moduleAPI[data['type']]['version']
        callFunc  = moduleAPI[data['type']]['callFunction']
        modVer    = 0 if 'moduleVersion' not in data else data['moduleVersion']

        with  tempfile.TemporaryDirectory() as td:
        #with  contextlib.nullcontext(tempfile.mkdtemp()) as td: #use this line if don't want directory to be auto deleted
            with open(os.path.join(td, 'tmp.cpp'), 'w') as f:
                f.write(textwrap.dedent(CONST_STRS.HEADER).format(header))
                f.write(textwrap.dedent(CONST_STRS.MODULE_INFO).format(typeStr, apiVer, modVer, name, desc))
                if data['type'] in runFuncDict:
                    f.write(textwrap.dedent(runFuncDict[data['type']]))
                else:
                    f.write(textwrap.dedent(CONST_STRS.RUN_FUNC_START)) #TODO this probably needs to be read from file but not sure how to put it in yet
                f.write(               (CONST_STRS.RUN_FUNC_MID).format(baseClass, str(pyFile), wrapClass, wrapClass, wrapClass, callFunc))
                f.write(textwrap.dedent(CONST_STRS.RUN_FUNC_END))
            
            compileLine = ""
            if not Args.o:
                compileLine = CONST_STRS.COMPILE_START + f.name + " -o " + str(str(yfPath.with_suffix('.so')) + " " + CONST_STRS.COMPILE_END)
            else:
                compileLine = CONST_STRS.COMPILE_START + f.name + " -o " + str(os.path.join(Args.o, os.path.basename(yfPath.with_suffix('.so')))) + " " + CONST_STRS.COMPILE_END
            if data['type'] in libDict:
                compileLine += libDict[data['type']]
            compileError = os.system(compileLine)
            if Args.debug_source:
                shutil.copy2(f.name, Path.cwd())
            if compileError != 0:
                print("Python stub compilation failed attempting: " + compileLine, file=sys.stderr)
                exit(1)
    # for yaml files


if __name__ == '__main__':
    main()