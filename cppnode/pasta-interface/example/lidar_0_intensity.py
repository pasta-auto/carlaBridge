from lidar_module import PythonLiDARModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Lidar0Intensity(PythonLiDARModuleWrapper):
    def run(self, arr, header):
        #print("Test lidar module num points: ", arr.shape)
        arr[:]["x"] *= 1