.. _serverDoc:

Server
==========================


The API servers load and execute provided modules. Each has a list of the currently loaded modules which is in
execution order (so data will be processed CARLA server->module 1->module 2 etc.). The lists of modules are 0 indexed (first is index 0).

Server Interfaces
----------------------
In the WSL2 image located under :code:`/root/apiServers` there is an executable for interfacing with the API servers called :code:`cmd_interface`.

To use it:

#. Open a Powershell / or terminal window
#. Run :code:`wsl -d autoware` to start or login to the WSL2 image
#. Navigate to the root folder with :code:`cd /root`
#. :code:`./apiServers/cmd_interface COMMAND TYPE ARGUMENT` for example to load a GNSS module called :code:`swapLatLon.so` run :code:`./apiServers/cmd_interface add gnss swapLatLon.so`

Most of the arguments for the commands will be the server type and the path to a module SO generated as detailed in the :ref:`moduleDoc` section.
The subsequent sections detail each of the commands and are applicable to any API server. Which type specified should depend on what kind of module want to load or interact with.

Commands - Arguments
-----------------------
The following sections detail available commands and their arguments. After each of these is run,
the server reports the currently loaded modules in their execution order. All commands take a server type argument which is one of:

- can
- camera
- gnss
- imu
- lidar
- radar

For commands which take an index,
the index must be within bounds of the module list. Note the list is 0 indexed so with a single module loaded the only valid
index is 0 (and if no modules are loaded all of the commands which take an index parameter error).
For any of these commands :code:`--help` can be used to get reminder text on usage. As an example:

.. code-block::

  ./apiServers/cmd_interface add --help
  Add PASTA API module
  Usage: ./build/pasta-interface/cmd_interface add [OPTIONS] Server_Type file
  
  Positionals:
    Server_Type TEXT:{can,camera,gnss,imu,lidar,radar} REQUIRED
                                Which server to send to
    file TEXT:FILE REQUIRED     Module SO file to load
  
  Options:
    -h,--help                   Print this help message and exit



version - type
^^^^^^^^^^^^^^^^^^^^^^^^
Displays the module interface version of both the server and client. In the future, the module interface may
change so this was added as a way to query what API version the server is running to ensure compatibility.


add - type, 1 path to .so
^^^^^^^^^^^^^^^^^^^^^^^^^^
Adds the .so argument to be loaded by the server and
if the server can find and successfully open the module, the module is added to the list of loaded modules
last in execution order (all other loaded modules will be run first).

remove - type, 1 path to .so
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Removes the .so, if it is loaded, from the loaded module list. If one doesn't exist an error is shown.

list - type (optional)
^^^^^^^^^^^^^^^^^^^^^^^^
Returns the list of the loaded modules with just the name and path of each module (same list as when any other command is run).
Can also specify :code:`--verbose` to this command which shows all details about the module (name, description, type, etc.)
and is equivalent to running show on each index. This information is sourced from the YAML file associated with each module (when a module is created this information is stored in the SO).

This is the only command where server type is optional; if type is not set list will return the loaded modules for all servers rather than just the loaded modules for the specified API server.

reload - type, index of module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Removes and re-adds the module at the given index. This is useful for clearing state (such as the filter list from CAN modules).

show - type, index of module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Provides a detailed description of the module at the given index (name, description, type, etc.). This
information is sourced from the YAML file associated with each module (when a module is created this information is stored in the SO).

move - type, index of module, new index
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changes the module execution order such that the module in position at the first parameter is moved to the second parameter. 
All other modules will have their execution order changed to facilitate this move.

Given a loaded module list ordered :code:`0: a.so 1: b.so 2: c.so`, :code:`move TYPE 0 2` will yield:
:code:`0: b.so 1: c.so 2: a.so`.

swap - type, 2 indices of modules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changes the module execution order so that the two module swap positions in the order.

Given a loaded module list ordered :code:`0: a.so 1: b.so 2: c.so`, :code:`swap TYPE 0 2` will yield:
:code:`0: c.so 1: b.so 2: a.so`.