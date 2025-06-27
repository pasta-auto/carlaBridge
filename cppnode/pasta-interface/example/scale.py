from camera_module import PythonCameraModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Scale(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func scale")
        tempShape = arr.shape
        xSize = tempShape[0]//2
        ySize = tempShape[1]//2
        zSize = tempShape[2]
        temp = ski.transform.resize(arr, (xSize, ySize, zSize)) # this returns float
        arr[0:xSize, 0:ySize, 0:zSize] = temp * 255
        arr[xSize:, :, :] = 0
        arr[:, ySize:, :] = 0