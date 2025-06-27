#!/usr/bin/env python

# this is where we would define the API so could read as from API import Python bindings
# so in actual this would look like from CAN_DATA_API import CAN_DATA_Wrapper
# then like below could do:
#class PythonObject(CAN_DATA_Wrapper):
#   this.callSomeCFunction() like this.filterCAN(...)
from camera_module import PythonCameraModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

SHOW_IMAGE = True

def show(arr):
    imgplot = plt.imshow(arr)
    plt.show()

class Gen(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func gen data")
        arr[:] = ski.data.coins()
        #arr = ski.data.coins() # this doesn't work can't re assign the c++ allocated object
        if SHOW_IMAGE:
            show(arr)
        return

class doesNothing: # c++ only is currently looking for python classes which are derived from the c++ module
    def run(self, arr):
        arr[:] = np.zeros(arr.shape)

class Scale(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func scale")
        xSize = 150
        ySize = 190
        temp = ski.transform.resize(arr, (xSize, ySize)) # this returns float
        arr[0:xSize, 0:ySize] = temp * 255
        arr[xSize:, :] = 0
        arr[:, ySize:] = 0
        if SHOW_IMAGE:
            show(arr)

class Rotate(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func rotate")
        temp = ski.transform.rotate(arr, 90) # this returns float
        arr[0:temp.shape[0], 0:temp.shape[1]] = temp * 255
        if SHOW_IMAGE:
            show(arr)

#class Show(PythonCameraModuleWrapper):
#    def run(self, arr):
#        show(arr)