from gnss_module import PythonGNSSModuleWrapper

import numpy as np

class SwapLatLon(PythonGNSSModuleWrapper):
    def run(self, arr):
        tmp = arr.copy()
        arr['latitude']  = tmp['longitude']
        arr['longitude'] = tmp['latitude']
        #print(tmp)