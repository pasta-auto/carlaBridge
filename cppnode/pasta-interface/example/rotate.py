from camera_module import PythonCameraModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Rotate(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func rotate with shape:", arr.shape)
        temp = ski.transform.rotate(arr, 90, preserve_range=True)
        #print("Finished ski-rotate")
        arr[:] = temp[:]