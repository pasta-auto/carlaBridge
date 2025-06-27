.. _moduleDoc:

Modules
==============

Modules in this system are shared object files specific to a given server type. When loaded by the API servers,
as new data is received from the CARLA simulator it is passed into these modules. Modules can read/write/modify the data before Autoware
receives it allowing for attacks on the autonomous driving agent to be performed. The process to build an SO useable by the API server from Python code is detailed in the following sections.


Writing a Pasta Interface Module
------------------------------------------
.. _modTypesTable:
.. list-table:: Available Module Types
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module Type
     - Wrapper Class
   * - CAN Module
     - can_module
     - PythonCANModuleWrapper
   * - Camera Module
     - camera_module
     - PythonCameraModuleWrapper
   * - GNSS Module
     - gnss_module
     - PythonGNSSModuleWrapper
   * - IMU Module
     - imu_module
     - PythonIMUModuleWrapper
   * - LiDAR Module
     - lidar_module
     - PythonLiDARModuleWrapper
   * - Radar Module
     - radar_module
     - PythonRadarModuleWrapper

Annotated Python Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following example code shows a very basic GNSS module which swaps the latitude and longitude of the input data.
The comments describe parts of the module relevant to other modules.

.. code-block:: Python
  :linenos:

    # Every Python module will have an import with the name of a base module type
    # that will import the Python wrapper class from (from base import wrapper)
    from gnss_module import PythonGNSSModuleWrapper 
    # Classes which will be called by the API should inherit from the same Python wrapper class
    # imported above. The name of the class doesn't matter (SwapLatLon in this case) 
    # only the wrapper class which is inherited from. 
    # There can be multiple of these classes which are called in order in which they appear
    class SwapLatLon(PythonGNSSModuleWrapper): 
        # Every class should implement a run method (here the name must be run)
        # whose arguments are dependant on the module type. Most will have a numpy array
        # with named columns relevant to the particular module type.
        def run(self, arr): 
            tmp = arr.copy()
            arr['latitude']  = tmp['longitude'] 
            arr['longitude'] = tmp['latitude']
            # GNSS modules also have arr['altitude']
            # When this function exits any values changed will be applied
            # to underlying data. Note modules cannot change the arr pointer and
            # reassigning it will have no effect (for example arr = tmp does nothing
            # but arr[:] = tmp[:] will overwrite contents of arr with tmp)

The core of every Python module will include a wrapper class import specific to the given module type. Every class which implements a run method and inherits from the wrapper class, will have its run method called (in order the classes appear in the file).
These classes and functions serve as the entry point for the API modules (like a "main" function) and other Python modules can be imported/used around these required classes/functions.
These API Python scripts must be paired with a corresponding YAML file describing basic properties of the module. The YAML and Python file pairs need to then be built into shared object files using the provided stub generation utility.

Apart from the specific import and run functions required, the code has few restrictions; it is possible to import and call other user defined or other general modules. One restriction however is that the module code cannot modify the input array pointer, only the underlying data. This means, for example, that the input data size cannot be changed directly.

Example YAML File
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

    name: "Swap Lat Lon"
    description: "Swaps the latitude and longitude of the input"
    type: "GNSS Module"
    moduleVersion: 0

An example GNSS module YAML file is shown above which is paired with the example code from the preceding section.
The fields that are read from the YAML file are :code:`name`, :code:`description`, :code:`type` and :code:`moduleVersion`. 
Of these, the only required field is type. Type is one of the Module Names listed in the :ref:`modTypesTable` table
(note name column not type column) and corresponds to which API server should load the module and thus what data the module receives as input.
The other fields are otherwise used to describe the module; this is mainly used to help identify the module running in an API server.

Shared Object Generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To convert a Python module into a shared object file a script is provided in the WSL2 image located at :code:`/root/apiInterface/stubGeneration/generateStub.py`. There are two relevant parameters:

* :code:`--target_dir` required parameter of a directory which contains at least one Python (.py) and YAML (.yaml) file pair with the same name
* :code:`-o` the directory where the shared object files will be generated. Defaults to :code:`--target_dir` if not specified

When run, for every Python file in the target directory which has a YAML with the same name, a shared object (.so) file also with the same name is created in the output directory. These can then be added to the API servers to be run.
It is important to note that this step does not be run every time want to modify the Python file; the stub generation only needs to be rerun if 

#. The file needs to be moved to a new location (including renamed) or
#. The type of the module or other information in the YAML file needs to be updated


Specific Module Notes
------------------------------------------
The following sections describe the shape and format of the input data received by each module type. For most of these, the columns are named accessible via :code:`array['columnName']`

CAN
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: 
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module
     - Wrapper Class
   * - CAN Module
     - can_module
     - PythonCANModuleWrapper

The CAN modules receive as input an array per CAN message sent by the system. This array includes the columns:

* :code:`id` integer denoting the type of CAN message
* :code:`size` integer describing the number of bits of data
* :code:`data` integer of the data in the CAN message

Additionally this class provides some built in functions which add functionality for filtering input CAN messages (so only messages with IDs in the filter set are received) as well as sending additional CAN messages in the system.
These functions are useable from Python module code by invoking :code:`PythonCANModuleWrapper.functionName(inputs)`. The available functions are:

* :code:`filter_add` takes 1 integer input which is added to the the filter. Only CAN messages with IDs in the filter are received (unless the filter is empty in which case all messages are received)
* :code:`filter_remove` 1 ID to be removed from the filter
* :code:`filter_clear` takes no inputs and removes all IDs from the filter (so all messages are received)
* :code:`filter_exists` checks if the given ID is already in the filter 
* :code:`send_can` takes 3 inputs: the id, data and size of a message to be sent to connected CAN devices

Note that the filters are unique per CAN module loaded and it is possible to have filters which mean the module will only be run once. 
If the only IDs in a module's filter are ones which are never sent by the system, the module will never be called again (as CAN modules are only run when an ID in the filter is received).
Modules start with no filtered IDs so will always be run at least once. If an unused ID is then added in the first invocation, the module will not be run again.
Using the module reload command will clear the filter allowing for updates to be performed.

Camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: 
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module
     - Wrapper Class
   * - Camera Module
     - camera_module
     - PythonCameraModuleWrapper

The input array to camera modules is of size 480,640,4 which represents a BGRA image. This means the image is height 480 pixels by 640 width pixels, each of which has 4, 1-byte components for the blue, green, red and alpha channels of the image.

GNSS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: 
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module
     - Wrapper Class
   * - GNSS Module
     - gnss_module
     - PythonGNSSModuleWrapper

The GNSS array consists of 3 large floating point values (:code:`double` type) named columns which are latitude, longitude, altitude.

IMU
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: 
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module
     - Wrapper Class
   * - IMU Module
     - imu_module
     - PythonIMUModuleWrapper

Example of a simple IMU module displaying the inputs.

.. code-block:: Python

    from imu_module import PythonIMUModuleWrapper
    
    class IMU(PythonIMUModuleWrapper):
        def run(self, gyro, accel, location, rotation):
            print("IMU module:")
            print("gyro: ", gyro['x'],gyro['y'],gyro['z'])
            print("acc: " ,accel['x'],accel['y'],accel['z'])
            print("loc: ",location['x'],location['y'],location['z'])
            print("rot: ",rotation['yaw'],rotation['pitch'],rotation['roll'])

The IMU modules receive 4 arrays as input representing the gyroscope, accelerometer, and relative location/orientation of the sensor. Each of the gyroscope, accelerometer and location sensor have 3 floating point values x, y, and z components. The rotation input has yaw, pitch, and roll values.

LiDAR
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: 
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module
     - Wrapper Class
   * - LiDAR Module
     - lidar_module
     - PythonLiDARModuleWrapper

The input array to LiDAR modules consists of some number of LiDAR detection points (varying between each module invocation). Each of these consist of 4 floating point elements: the relative x, y, and z position as well as the intensity of the detection.
The number of points can be queried using :code:`arr.shape[0]` and each element can accessed with index of detection which should be between 0 and the number of points (for example :code:`arr[0]["intensity"]` would get the intensity of the first detection point).

Radar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: 
   :widths: 25 25 25
   :header-rows: 1

   * - Module Name
     - Module
     - Wrapper Class
   * - Radar Module
     - radar_module
     - PythonRadarModuleWrapper

The input array to radar modules consists of some number of radar detections (varying between each module invocation). Each of these consist of 4 floating point elements: the altitude, azimuth, distance, and velocity toward the sensor.
The number of points can be queried using :code:`arr.shape[0]` and each element can accessed with index of detection which should be between 0 and the number of points (for example :code:`arr[0]["velocity"]` would get the incoming velocity of the first detection).
