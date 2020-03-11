"""
The manager for the external modules which extend the NRP through ROS launch
mechanism.
"""

__author__ = 'Omer Yilmaz'

import concurrent.futures
import re
import rosservice
from hbp_nrp_cle.externalsim.ExternalModule import ExternalModule
from hbp_nrp_cle.robotsim.GazeboHelper import TIMEOUT


class ExternalModuleManager(object):
    """
    This class automatically detects the external modules searching the ROS
    services available at the ROS server. It keeps and array of the external
    modules and calls initialize, run_step ans shutdown methods for each
    external module. One object of this class is used by the Deterministic
    Closed Loop Engine and is synchronized with it making every external module
    on the array also synchronized.
    """

    def __init__(self, module_names=None):

        if module_names is not None:
            self.module_names = module_names
        else:
            # If no module_names were provided, check which modules are advertising their services
            self.module_names = []
            for service in rosservice.get_service_list():
                m = re.match(r"/emi/(\w+)_module/initialize", str(service))
                if m:
                    module_name = m.group(1)
                    self.module_names.append(module_name)

        self.thread_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=max(len(self.module_names), 1))

        self.ema = []
        if self.module_names:
            future_results = [self.thread_pool.submit(ExternalModule, x) for x in self.module_names]

            # Wait for modules, raise exception if module does not respond within given timeframe
            status = concurrent.futures.wait(future_results, timeout=TIMEOUT)
            if status.not_done:
                raise Exception("Could not start all modules")

            for future in future_results:
                self.ema.append(future.result())

    def _exec_module_call(self, function):
        """
        Creates a thread for every module and executes the requested function in it
        :param function: Function to execute on every module
        """
        if self.module_names:
            future_results = [self.thread_pool.submit(function, x) for x in self.ema]
            concurrent.futures.wait(future_results, return_when=concurrent.futures.ALL_COMPLETED)

    def initialize(self):
        """
        This method is used to run all initialize methods served at each external models at once.
        """
        self._exec_module_call(ExternalModule.initialize)

    def run_step(self):
        """
        This method is used to run all run_step methods served at each external models at once.
        """
        self._exec_module_call(ExternalModule.run_step)

    def shutdown(self):
        """
        This method is used to run all shutdown methods served at each external models at once.
        """
        self._exec_module_call(ExternalModule.shutdown)
