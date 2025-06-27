from radar_module import PythonRadarModuleWrapper
import skimage as ski
import matplotlib.pyplot as plt
import numpy as np

class Radar(PythonRadarModuleWrapper):
    def run(self, arr):
        print("Radar module:")
        print("Number of detections: ", arr.shape[0])
        print("Example Detection:")
        print("altitude: ", arr[0]["altitude"])
        print("azimuth:  ", arr[0]["azimuth"])
        print("depth:    ", arr[0]["depth"])
        print("velocity: ", arr[0]["velocity"])
