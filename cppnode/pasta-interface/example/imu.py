from imu_module import PythonIMUModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class IMU(PythonIMUModuleWrapper):
    def run(self, gyro, accel, location, rotation):
        print("IMU module:")
        print("gyro: ", gyro['x'],gyro['y'],gyro['z'])
        print("acc: " ,accel['x'],accel['y'],accel['z'])
        print("loc: ",location['x'],location['y'],location['z'])
        print("rot: ",rotation['yaw'],rotation['pitch'],rotation['roll'])