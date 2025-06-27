from camera_module import PythonCameraModuleWrapper

import numpy as np

def show(arr):
    print("What?")

class Shape(PythonCameraModuleWrapper):
    def run(self, arr):
        print("Func shape")
        print(arr.shape)