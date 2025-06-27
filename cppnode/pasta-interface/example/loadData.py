from camera_module import PythonCameraModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Gen(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func gen data")
        arr[:,:,0] = ski.data.coins()
        arr[:,:,1] = ski.data.coins()
        arr[:,:,2] = ski.data.coins()
        arr[:,:,3] = 1