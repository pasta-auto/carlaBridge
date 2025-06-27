from camera_module import PythonCameraModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Show(PythonCameraModuleWrapper):
    def run(self, arr):
        print("func show")
        imgplot = plt.imshow(arr[:,:,0:3])
        plt.show()