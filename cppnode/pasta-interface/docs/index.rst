.. PASTA Interface documentation master file, created by
   sphinx-quickstart on Tue Sep 17 11:14:12 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

PASTA Interface documentation
=============================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   Modules
   Server


The PASTA Interface API aims to provide a simple method for inserting code in the data path between CARLA simulator and autonomous driving solutions.
The code is inserted as modules which can alter senor data or add CAN messages so the effects on autonomous driving can be observed.
The API is implemented as a set of 6 servers (for CAN, camera, GNSS, IMU, Lidar and Radar) which load different types of shared object modules files that can be dynamically loaded, unloaded, and have their execution order changed etc.
This documentation has three main components: the :ref:`moduleDoc` section details how to write Python code that can be built into a shared object (SO) module file; and the :ref:`serverDoc` section which describes loading a module into one of the API servers and the following sections which detail simple installation and use.

Installation
-------------------

This document assumes the reader has read the CARLA - Autoware User Integration Manual as this release has the same setup for installing WSL and CARLA, configuring firewall etc.
The new installation uses a new WSL2 image but otherwise the steps remain the same.

To use the new WSL2 image (assuming have already installed WSL2), download and unzip the new :code:`pasta_autoware_wsl2.zip`. Inside this zip is a :code:`pasta_autoware.vhdx`
which can be turned into a WSL image by:

#. Opening a Powershell / or terminal window
#. Navigate to the folder containing the :code:`pasta_autoware.vhdx`
#. Run :code:`wsl -–list` to check if there is an existing :code:`autoware` image
#. If there is an existing image, uninstall the existing :code:`autoware` image. :red:`Warning this will remove the image and ALL contents are deleted`. 

    * run :code:`wsl --unregister autoware`
#. Install the new image: :code:`wsl -–import-in-place autoware pasta_autoware.vhdx`

Minimum Usage
-------------------
The basic functionality of this release with no modules loaded is similar to previous releases but now requires the API servers to be running.
Even with no loaded modules, data (like sensor data, CAN messages) is sent from CARLA to the API servers before being 
forwarded to Autoware. This means that if the API servers are not running, Autoware will never receive the sensor data it needs to function.

To start the API servers:

#. Opening a Powershell / or terminal window
#. Run :code:`wsl -d autoware` to start or login to the WSL2 image
#. Within the :code:`/root` folder there are two scripts :code:`startServers.sh` and :code:`stopServers.sh` which when run start and stop the 6 servers respectively

With the API servers started, the process to run the rest of the application is the same as described in the CARLA - Autoware User Integration Manual:
namely the application can be started using the Autoware using LFA6U in the CARLA - PASTA control GUI.

Note that the window where the servers were started from is also where standard output from the modules appears.
This means for example :code:`print` statements in the Python module code will appear here once the module is loaded.