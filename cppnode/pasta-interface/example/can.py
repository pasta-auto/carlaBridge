#!/usr/bin/env python

# this is where we would define the API so could read as from API import Python bindings
# so in actual this would look like from CAN_DATA_API import CAN_DATA_Wrapper
# then like below could do:
#class PythonObject(CAN_DATA_Wrapper):
#   this.callSomeCFunction() like this.filterCAN(...)
from can_module import PythonCANModuleWrapper

from threading import Timer



class Gen(PythonCANModuleWrapper):
    def run(self, arr):
        
        PythonCANModuleWrapper.filter_add(131)
        #PythonCANModuleWrapper.filter_clear()
        print(arr)
        #arr['data'] = arr['data'] + 1

        #PythonCANModuleWrapper.send_can(5,200,20)
        #PythonCANModuleWrapper.send_can(2,500,50)

        return

    
    
