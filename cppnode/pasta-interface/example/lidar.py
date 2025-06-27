from lidar_module import PythonLiDARModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Lidar(PythonLiDARModuleWrapper):
    def run(self, arr, header):
        print("Lidar module:")
        print("XYZI points array shape: ", arr.shape)
        print("Horizontal angle: ", header['horizontalAngle'], "Number of channels:", header['channels'])
        print("Example Detection:")
        print("x:        ", arr[0]["x"]) # same as arr[0][0]
        print("y:        ", arr[0]["y"])
        print("z:        ", arr[0]["z"])
        print("intensity:", arr[0]["intensity"])