"""
The manager for the external modules which extend the NRP through ROS launch
mechanism.
"""

__author__ = 'Omer Yilmaz'

import concurrent.futures
import re
import rosservice
from hbp_nrp_cle.externalsim.ExternalModule import ExternalModule

class ExternalModuleManager(object):
    """
    This class automatically detects the external modules searching the ROS
    services available at the ROS server. It keeps and array of the external
    modules and calls initialize, run_step ans shutdown methods for each
    external module. One object of this class is used by the Deterministic
    Closed Loop Engine and is synchronized with it making every external module
    on the array also synchronized.
    """

    def __init__(self):
        self.module_names = []
        for service in rosservice.get_service_list():
            m = re.match(r"/emi/(\w+)_module/initialize", str(service))
            if m:
                module_name = m.group(1)
                self.module_names.append(module_name)

        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=max(len(self.module_names), 1))

        self.ema = []
        if self.module_names:
            future_results = [self.thread_pool.submit(ExternalModule, x) for x in self.module_names]
            concurrent.futures.wait(future_results)
            for future in future_results:
                self.ema.append(future.result())

    def exec_module_call(self, function):
        """
        Creates a thread for every module and executes the requested function in it
        :param function: Function to execute on every module
        """
        if self.module_names:
            future_results = [self.thread_pool.submit(function, x) for x in self.ema]
            concurrent.futures.wait(future_results)
            for future in future_results:
                while not future.result().done():
                    pass

    def initialize(self):
        """
        This method is used to run all initialize methods served at each external models at once.
        """
        self.exec_module_call(ExternalModule.initialize)

    def run_step(self):
        """
        This method is used to run all run_step methods served at each external models at once.
        """
        self.exec_module_call(ExternalModule.run_step)

    def shutdown(self):
        """
        This method is used to run all shutdown methods served at each external models at once.
        """
        self.exec_module_call(ExternalModule.shutdown)
