# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
Implementation of the closed loop engine.
"""

__author__ = 'Georg Hinkel'

import hbp_nrp_cle as cle
from hbp_nrp_cle.cle.DeterministicClosedLoopEngine import DeterministicClosedLoopEngine
import time
import logging
import threading
from hbp_nrp_cle.cle.CLEInterface import ForcedStopException

logger = logging.getLogger('hbp_nrp_cle')


# pylint: disable=R0902
# the attributes are reasonable in this case
class ClosedLoopEngine(DeterministicClosedLoopEngine):
    """
    Implementation of the closed loop engine that runs Transfer Functions, brain
    simulation and world simulation all in parallel for best effort performance
    """

    def __init__(self,
                 robot_control_adapter,
                 robot_comm_adapter,
                 brain_control_adapter,
                 brain_comm_adapter,
                 transfer_function_manager,
                 external_module_manager,
                 dt):
        """
        Create an instance of the cle.

        :param robot_control_adapter: an instance of IRobotContolAdapter
        :param robot_comm_adapter: an instance of IRobotCommunicationAdapter
        :param brain_control_adapter: an instance of IBrainContolAdapter
        :param brain_comm_adapter: an instance of IBrainCommunicationAdapter
        :param transfer_function_manager: an instance of ITransferFunctionManager
        :param dt: The CLE time step in seconds
        """
        super(ClosedLoopEngine, self).__init__(robot_control_adapter, robot_comm_adapter,
                                               brain_control_adapter, brain_comm_adapter,
                                               transfer_function_manager, external_module_manager,
                                               dt)

        self.__tf_thread = None
        self.__tf_start_event = threading.Event()
        self.__tf_done_event = threading.Event()

    def run_step(self, timestep):
        """
        Runs both simulations for the given time step in seconds.

        :param timestep: simulation time, in seconds
        :return: Updated simulation time, otherwise -1
        """
        clk = cle.clock

        self.__tf_start_event.set()

        # robot simulation
        logger.debug("Run step: Robot simulation.")
        self.rca_future = self.rca.run_step_async(timestep)
        self.rcm.refresh_buffers(clk)

        # brain simulation
        logger.debug("Run step: Brain simulation")
        start = time.time()
        self.bca.run_step(timestep * 1000.0)
        self.bcm.refresh_buffers(clk)
        self._bca_elapsed_time += time.time() - start

        # update clock
        cle.clock += timestep

        # wait for all thread to finish
        logger.debug("Run_step: waiting on Control thread")
        try:
            f = self.rca_future
            f.result()
            self._rca_elapsed_time += f.end - f.start
        except ForcedStopException:
            logger.warn("Simulation was brutally stopped.")

        self.__tf_done_event.wait()
        self.__tf_done_event.clear()

        logger.debug("Run_step: done !")
        return cle.clock

    def __run_tfs(self):
        """
        Runs the Transfer Functions. To be executed in a separate thread
        """
        clk = cle.clock
        while not self.stopped_flag.isSet():
            self.__tf_start_event.wait()
            self.__tf_start_event.clear()
            try:
                # transfer functions
                logger.debug("Run step: Transfer functions")
                self.tfm.run_robot_to_neuron(clk)
                self.tfm.run_neuron_to_robot(clk)
            finally:
                self.__tf_done_event.set()

    def start(self):
        """
        Starts the orchestrated simulations
        """
        if super(ClosedLoopEngine, self).start():
            self.__tf_thread = threading.Thread(target=self.__run_tfs)
            self.__tf_thread.setDaemon(True)
            self.__tf_thread.start()
