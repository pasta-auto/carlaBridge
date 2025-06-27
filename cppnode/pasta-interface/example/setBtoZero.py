from camera_module import PythonCameraModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Set0(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func 0B")
        arr[:,:,0] = 0