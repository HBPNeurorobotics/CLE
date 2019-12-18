"""
The manager for the external modules which extend the NRP through ROS launch
mechanism.
"""

__author__ = 'Omer Yilmaz'

import concurrent.futures
from multiprocessing import Process, Pool, cpu_count
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
            m = re.match(r"/emi/.*/initialize", str(service))
            if m:
                module_name = m.group(0)[5:-18]
                self.module_names.append(module_name)

        self.ema = []
        if len(self.module_names) is not 0:
            with concurrent.futures.ThreadPoolExecutor(max_workers=max(len(self.module_names), 1)) as executor:
                future_results = [executor.submit(ExternalModule, x) for x in self.module_names]
                concurrent.futures.wait(future_results)
                for future in future_results:
                    self.ema.append(future.result())


    def initialize(self):
        """
        This method is used to run all initialize methods served at each external models at once.
        """
        if len(self.module_names) is not 0:
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.ema)) as executor:
                future_results = [executor.submit(x.initialize) for x in self.ema]
                concurrent.futures.wait(future_results)
                for future in future_results:
                    while not future.result().done():
                        pass

    def run_step(self):
        """
        This method is used to run all run_step methods served at each external models at once.
        """
        if len(self.module_names) is not 0:
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.ema)) as executor:
                future_results = [executor.submit(x.run_step) for x in self.ema]
                concurrent.futures.wait(future_results)
                for future in future_results:
                    while not future.result().done():
                        pass

    def shutdown(self):
        """
        This method is used to run all shutdown methods served at each external models at once.
        """
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.ema)) as executor:
            future_results = [executor.submit(x.shutdown) for x in self.ema]
            concurrent.futures.wait(future_results)
            for future in future_results:
                while not future.result().done():
                    pass

