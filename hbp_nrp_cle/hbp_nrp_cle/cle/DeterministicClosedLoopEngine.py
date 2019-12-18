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

__author__ = 'LorenzoVannucci'

import time
import logging
import threading
from concurrent.futures import Future
from hbp_nrp_cle.cle.CLEInterface import IClosedLoopControl, ForcedStopException

from hbp_nrp_cle.cle.CLEInterface import BrainTimeoutException

from hbp_nrp_cle.cle.__helper import get_tf_elapsed_times
from hbp_nrp_cle.robotsim.GazeboHelper import GazeboHelper
import hbp_nrp_cle as cle

logger = logging.getLogger('hbp_nrp_cle')


# pylint: disable=R0902
# the attributes are reasonable in this case
class DeterministicClosedLoopEngine(IClosedLoopControl):
    """
    Implementation of the closed loop engine that runs transfer functions sequentially
    to brain simulation and world simulation. Therefore, we are to some degree sure that
    the TFs work on a reproducible state
    """

    # default simulation timestep in seconds (20 ms)
    DEFAULT_TIMESTEP = 0.02

    def __init__(self,
                 robot_control_adapter,
                 robot_comm_adapter,
                 brain_control_adapter,
                 brain_comm_adapter,
                 transfer_function_manager,
                 external_module_array,
                 dt
                 ):
        """
        Create an instance of the cle.

        :param robot_control_adapter: an instance of IRobotContolAdapter
        :param robot_comm_adapter: an instance of IRobotCommunicationAdapter
        :param brain_control_adapter: an instance of IBrainContolAdapter
        :param brain_comm_adapter: an instance of IBrainCommunicationAdapter
        :param transfer_function_manager: an instance of ITransferFunctionManager
        :param dt: The CLE time step in seconds
        """
        self.rca = robot_control_adapter
        self.rca_future = None

        self.rcm = robot_comm_adapter
        self.bca = brain_control_adapter
        self.bcm = brain_comm_adapter
        self.tfm = transfer_function_manager
        self.ema = external_module_array
        # default timestep
        self.timestep = dt

        # stop flag ask the main loop to stop
        self.stop_flag = threading.Event()
        self.stop_flag.clear()

        # stopped flag indicate if the main loop is stopped
        self.stopped_flag = threading.Event()
        self.stopped_flag.set()

        # global clock
        cle.clock = 0.0

        self.initialized = False

        # time measurements
        # The real time is measured this way: each time play is called
        # start_time is set to the current clock time. Each time the stop
        # function is called (pausing the simulation) elapsed_time is
        # incremented with (stop time - start_time). Thus, the real time is
        # elapsed_time                               if paused
        # elapsed_time + (current time - start_time) if running
        self.start_time = 0.0
        self.elapsed_time = 0.0

        self._rca_elapsed_time = 0.0
        self._bca_elapsed_time = 0.0
        self.__network_file = None
        self.__network_configuration = None

        self.__initial_robot_poses = None

        # This is to be eventually removed, as communications towards gazebo should
        # be done from inside the RosControlAdapter
        self.gazebo_helper = GazeboHelper()

        self.__start_thread = None
        self.__start_future = None
        self.start_cb = lambda f: None
        self.initial_models = None
        self.initial_lights = None

    def initialize(self, brain_file=None, **configuration):
        """
        Initializes the closed loop engine.
        :param brain_file: A python PyNN script containing the neural network definition
        :param configuration: A set of populations
        """
        self.rca.initialize()
        self.bca.initialize()
        self.tfm.initialize('tfnode')
        self.ema.initialize()
        cle.clock = 0.0
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self.initialized = True
        if brain_file:
            self.__network_file = brain_file
            self.__network_configuration = configuration
            try:
                self.load_brain(brain_file, **configuration)
            except BrainTimeoutException:
                logger.info(
                    "Timeout occurs during loading the brain:" + brain_file)
            except Exception as e:
                logger.exception(
                    "Compiling Error during loading the brain({0}): {1!r}".format(brain_file, e))

    @property
    def is_initialized(self):
        """
        Returns True if the simulation is initialized, False otherwise.
        """
        return self.initialized

    def load_populations(self, **populations):
        """
        load new populations into the brain

        :param populations: A dictionary indexed by population names and
        containing neuron indices. Neuron indices can be defined by
        lists of integers or slices. Slices are either python slices or
        dictionaries containing 'from', 'to' and 'step' values.
        """
        if self.initialized:
            if self.running:
                self.stop()
            if self.bca.is_alive():
                self.bca.shutdown()
            logger.info("Loading new populations")
            self.bca.load_populations(**populations)
            self.tfm.hard_reset_brain_devices()

    def load_brain(self, brain_file, **brain_populations):
        """
        Creates a new brain in the running simulation

        :param brain_file: A python PyNN script or an h5 file
        containing the neural network definition
        :param brain_populations: A (optional) dictionary indexed by population names and
        containing neuron indices. Neuron indices can be defined by
        lists of integers or slices. Slices are either python slices or
        dictionaries containing 'from', 'to' and 'step' values.
        """
        if self.initialized:
            if self.running:
                self.stop()
            if self.bca.is_alive():
                self.bca.shutdown()
            logger.info("Recreating brain from file " + brain_file)
            self.bca.load_brain(brain_file, **brain_populations)
            logger.info("Resetting TFs")
            self.tfm.hard_reset_brain_devices()

    def run_step(self, timestep):
        """
        Runs both simulations for the given time step in seconds.

        :param timestep: simulation time, in seconds
        :return: Updated simulation time, otherwise -1
        """
        clk = cle.clock

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

        # wait for all thread to finish
        logger.debug("Run_step: waiting on Control thread")
        try:
            f = self.rca_future
            f.result()
            self._rca_elapsed_time += f.end - f.start
        except ForcedStopException:
            logger.warn("Simulation was brutally stopped.")

        # transfer functions
        logger.debug("Run step: Transfer functions")
        self.tfm.run_robot_to_neuron(clk)
        self.tfm.run_neuron_to_robot(clk)

        self.ema.run_step()

        # update clock
        cle.clock += timestep

        logger.debug("Run_step: done !")
        return cle.clock

    def shutdown(self):
        """
        Shuts down both simulations.
        """
        self.stop_flag.set()
        self.stopped_flag.wait(5)
        self.rcm.shutdown()
        self.bcm.shutdown()
        self.rca.shutdown()
        self.bca.shutdown()
        self.ema.shutdown()

    def start(self):
        """
        Starts the orchestrated simulations
        """
        if self.__start_future is None:
            self.__start_future = Future()
            if self.start_cb is not None:
                self.start_cb(self.__start_future)
            self.__start_thread = threading.Thread(target=self.__loop)
            self.__start_thread.setDaemon(True)
            self.__start_future.set_running_or_notify_cancel()
            self.__start_thread.start()
            return True
        else:
            logger.warning("CLE is already running")
            return False

    def __loop(self):
        """
        Starts the orchestrated simulations.
        This function does not return (starts an infinite loop).
        """
        logger.info("Simulation loop started")
        try:
            try:
                self.stop_flag.clear()
                self.stopped_flag.clear()
                self.start_time = time.time()
                while not self.stop_flag.isSet():
                    self.run_step(self.timestep)
                self.__start_future.set_result(None)
            finally:
                logger.info("Simulation loop ended")
                self.elapsed_time += time.time() - self.start_time
                self.__start_thread = None
                self.stopped_flag.set()
        # pylint: disable=broad-except
        # we change the order intentionally (first finally then catch) because
        # before throwing an custom exception (brainRuntimeException), the stopped flag
        # has to be set in order to finish the stop function flow.
        except Exception as e:
            logger.exception(e)
            self.__start_future.set_exception(e)
        finally:
            self.__start_future = None

    def stop(self, forced=False):
        """
        Stops the orchestrated simulations. Also waits for the current
        simulation step to end.
        Must be called on a separate thread from the start one, as an example
        by a threading.Timer.

        :param forced: If set, the CLE instance cancels pending tasks
        :except Exception: throws an exception if the CLE was forced to stop but could not be
        stopped
        """
        self.stop_flag.set()
        if forced:
            if self.rca_future is not None and self.rca_future.running():
                self.stopped_flag.wait(5)
                if self.rca_future.running():
                    self.rca_future.set_exception(ForcedStopException())
            self.wait_step(timeout=self.bca.get_Timeout())
            if not self.stopped_flag.isSet():
                raise Exception("The simulation loop could not be completed")
        else:
            self.wait_step()

    def reset(self):
        """
        Reset the orchestrated simulations (stops them before resetting).
        """
        self.stop()
        self.rca.reset()
        self.bca.reset()
        self.tfm.reset()
        cle.clock = 0.0
        self.start_time = 0.0
        self.elapsed_time = 0.0
        self._rca_elapsed_time = 0.0
        self._bca_elapsed_time = 0.0
        logger.info("CLE reset")

    def reset_world(self, sdf_world_string=""):
        """
        Reset the world to a given configuration, or to the initial one if none is passed.

        :param sdf_world_string: the new environment to be set in the world simulator
            (default value is the empty string for compatibility with the ROS message).
        """
        if len(sdf_world_string) > 0:
            models, lights = self.gazebo_helper.parse_world_string(sdf_world_string)
        else:
            # backward compatibility
            # when working without collab, reset to the initial state
            models = self.initial_models
            lights = self.initial_lights

        self.stop()
        self.rca.reset_world(models, lights)
        logger.info("CLE world reset")

    def reset_brain(self, brain_file=None, populations=None):
        """
        Reloads the brain and resets the transfer function.
        If no parameter is specified, it reloads the initial brain.

        :param brain_file: A python PyNN script containing the neural network definition
        :param populations: A set of populations
        """
        if brain_file is not None and populations is not None:
            self.load_brain(brain_file, **populations)
        else:
            self.load_brain(self.__network_file, *self.__network_configuration)
        logger.info("CLE Brain reset")

    @property
    def simulation_time(self):  # -> float64
        """
        Get the current simulation time.
        """
        return cle.clock

    @property
    def running(self):
        """
        Gets a flag indicating whether the simulation is running
        """
        return self.__start_future is not None and self.__start_future.running()

    @property
    def real_time(self):  # -> float64
        """
        Get the total execution time.
        """
        if self.running:
            return self.elapsed_time + time.time() - self.start_time
        return self.elapsed_time

    @property
    def initial_robot_poses(self):
        """
        Get the robot pose at the beginning of the simulation.
        """
        return self.__initial_robot_poses

    @initial_robot_poses.setter
    def initial_robot_poses(self, value):
        """
        Set the robot pose at the beginning of the simulation.
        Usually performed after CLE initialization.
        :param value: The new value for the initial robot pose.
        """
        self.__initial_robot_poses = value

    def reset_robot_pose(self):
        """
        Set the robot in the simulation to its initial pose.
        """
        for rid, pose in self.__initial_robot_poses.iteritems():
            self.gazebo_helper.set_model_pose(str(rid), pose)
        self.tfm.reset()

    def tf_elapsed_time(self):
        """
        Gets the time share of the Transfer Functions
        """
        return get_tf_elapsed_times(self.tfm)

    def brainsim_elapsed_time(self):
        """
        Gets the time share of the brain simulation
        """
        return self._bca_elapsed_time

    def robotsim_elapsed_time(self):
        """
        Gets the time share of the robot simulation
        """
        return self._rca_elapsed_time

    def wait_step(self, timeout=None):
        """
        Wait for the currently running simulation step to end.

        :param timeout: The maximum amount of time (in seconds) to wait for the end of this step
        """
        self.stopped_flag.wait(timeout)
