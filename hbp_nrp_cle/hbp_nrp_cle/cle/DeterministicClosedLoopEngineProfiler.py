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
Specification of the closed loop engine with profiling capabilities.
"""

__author__ = 'Eloy Retamino'

from hbp_nrp_cle.tf_framework import CSVRecorder
from hbp_nrp_cle.brainsim.nest.NestCommunicationAdapter import NestCommunicationAdapter
from hbp_nrp_cle.brainsim.pynn_nest.PyNNNestCommunicationAdapter import PyNNNestCommunicationAdapter
from hbp_nrp_distributed_nest.cle.DistributedPyNNCommunicationAdapter import \
    DistributedPyNNCommunicationAdapter
from hbp_nrp_cle.cle.DeterministicClosedLoopEngine import DeterministicClosedLoopEngine
import logging
import cProfile
import nest
import hbp_nrp_cle as cle
import time
import os
from cle_ros_msgs import srv

logger = logging.getLogger('hbp_nrp_cle')


def log_brain_sim_info_nest(timestep, brain_load_time=0.0, csv_recorder=None, csv_filename=''):
    #  pylint: disable = W0702

    """
    Logs information from nest simulation into a CSVRecorder object

    :param timestep: simulation timestep
    :param brain_load_time: time taken to load the brain file in seconds
    :param csv_recorder:  instance of CSVRecorder to be used for logging. If None, a new one is
                          created
    :param csv_filename: name for the csv file where information will be logged. Just used if
                    csv_recorder is None.
    :return:  csv_recorder
    """

    # Log parallelization information
    num_processes = nest.NumProcesses()

    if num_processes == 1:
        k_status = nest.GetKernelStatus()
        n_status = nest.GetStatus(range(1, k_status['network_size']))
    else:
        # this is required because calling nest.GetKernelStatus() raises and error with MPI
        k_status = {'num_processes': num_processes, 'local_num_threads': 0,
                    'total_num_virtual_procs': 0, 'num_connections': 0}

        n = 1
        n_status = []
        # compile elements status one by one until it fails
        while True:
            try:
                n_status += nest.GetStatus([n])
                n += 1
            except:
                break

    if not csv_recorder:
        csv_recorder = CSVRecorder(csv_filename, ['timestep', 'local_num_threads', 'num_processes',
                                                   'total_num_virtual_procs', 'num_connections',
                                                   'network_size', 'num_neurons',
                                                   'num_local_nodes_rank_0', 'brain_load_time'])

    n_local = 0
    n_neuron = 0
    for n in n_status:
        n_neuron += 1 if n['element_type'].name == 'neuron' else 0
        c = n['local'] and n['element_type'].name == 'neuron'
        n_local += 1 if c else 0

    csv_recorder.record_entry(timestep, k_status['local_num_threads'],
                                   k_status['num_processes'], k_status['total_num_virtual_procs'],
                                   k_status['num_connections'], len(n_status), n_neuron, n_local,
                                   brain_load_time)

    return csv_recorder


class DeterministicClosedLoopEngineProfiler(DeterministicClosedLoopEngine):
    """
    Implementation of the closed loop engine that runs with a profiler
    """

    # In this dictionary is stored a mapping between subclasses of IBrainCommunicationAdapter
    # and the function used to log out information from the corresponding brain simulator.
    # To add support for other adapters: 1) define a function with signature:
    # :param timestep: simulation timestep
    # :param brain_load_time: time taken to load the brain file in seconds
    # :param csv_recorder:  instance of CSVRecorder to be used for logging. If None, a new one is
    #                       created
    # :param csv_filename: name for the csv file where information will be logged. Just used if
    #                  csv_recorder is None.
    # :return:  csv_recorder
    #                                    2) add an entry with value equal to the new function and
    # key equal to the IBrainCommunicationAdapter class that it gives support to
    __log_brain_sim_info_f = {NestCommunicationAdapter: log_brain_sim_info_nest,
                              PyNNNestCommunicationAdapter: log_brain_sim_info_nest,
                              DistributedPyNNCommunicationAdapter: log_brain_sim_info_nest}

    # Dictionary containing the available profiler modes
    __profiler_modes = {'disabled': srv.CreateNewSimulationRequest.PROFILER_DISABLED,
                        'cle_step': srv.CreateNewSimulationRequest.PROFILER_CLE_STEP,
                        'cprofile': srv.CreateNewSimulationRequest.PROFILER_CPROFILE}

    def __init__(self,
                 robot_control_adapter,
                 robot_comm_adapter,
                 brain_control_adapter,
                 brain_comm_adapter,
                 transfer_function_manager,
                 dt,
                 profiler_mode=srv.CreateNewSimulationRequest.PROFILER_DISABLED,
                 profiler_dir=''
                 ):
        #  pylint: disable = R0913
        """
        Create an instance of the cle.

        :param robot_control_adapter: an instance of IRobotContolAdapter
        :param robot_comm_adapter: an instance of IRobotCommunicationAdapter
        :param brain_control_adapter: an instance of IBrainContolAdapter
        :param brain_comm_adapter: an instance of IBrainCommunicationAdapter
        :param transfer_function_manager: an instance of ITransferFunctionManager
        :param dt: The CLE time step in seconds
        :param profiler_mode: specifies the profiler mode: disabled, cle_step, cprofile
        :param profiler_dir: path to the folder where cProfile output files will be saved to
        """

        super(DeterministicClosedLoopEngineProfiler, self).__init__(
            robot_control_adapter,
            robot_comm_adapter,
            brain_control_adapter,
            brain_comm_adapter,
            transfer_function_manager,
            dt)

        self._profiler_mode = profiler_mode
        self._profiler_dir = profiler_dir

        self._cle_csv_recorder = None
        self._brain_sim_csv_recorder = None
        self._csv_recorders = []

        self._times_brain_loaded = 0
        self._times_loop_started = 0

        # handle unsupported profiler cases
        if self._profiler_mode not in self.__profiler_modes.values():
            logger.warn('Disabling CLE profiler due to: unknown profiler mode passed')
            self._profiler_mode = self.__profiler_modes['disabled']
        elif self._profiler_mode != self.__profiler_modes['disabled'] and \
                not os.path.isdir(self._profiler_dir):
            logger.warn('Disabling CLE profiler due to: incorrect profiler directory passed')
            self._profiler_mode = self.__profiler_modes['disabled']

    def __log_brain_sim_info(self, timestep, brain_load_time=0.0, csv_recorder=None,
                             csv_filename=''):
        """
        Calls the right brain simulation log function for the current simulator
        :return: the CSVRecorder used to log the information
        """

        try:
            f = self.__log_brain_sim_info_f[type(self.bcm)]
        except KeyError:
            logger.warn('Logging brain simulation configuration is not supported for {} '.format(
                type(self.bcm)))
            return None
        else:
            return f(timestep, brain_load_time, csv_recorder, csv_filename)

    def __dump_csv_recorder_to_file(self, csv_recorder):
        """
        Writes values from csv_recorder to file
        :param csv_recorder: an instance of CSVRecorder
        """

        assert(isinstance(csv_recorder, CSVRecorder))
        name = csv_recorder.get_csv_recorder_name()

        # creates file and add headers if the file does not exist
        if not os.path.isfile(os.path.join(self._profiler_dir, name)):
            with open(os.path.join(self._profiler_dir, name), 'w') as f:
                f.write(''.join(csv_recorder.get_csv_headers()))

        # append values
        with open(os.path.join(self._profiler_dir, name), 'a') as f:
            for value in csv_recorder.cleanup():
                f.write(value)

    def run_step(self, timestep):
        """
        Runs both simulations for the given time step in seconds recording profiling data

        :param timestep: simulation time, in seconds
        :return: Updated simulation time, otherwise -1
        """

        # get robot and brain elapse time before the step
        robot_start_time = self._rca_elapsed_time
        brain_start_time = self._bca_elapsed_time

        # run step
        start = time.time()
        super(DeterministicClosedLoopEngineProfiler, self).run_step(timestep)

        # record profiling data
        if self._profiler_mode == self.__profiler_modes['cle_step'] and self._cle_csv_recorder:
            self._cle_csv_recorder.record_entry(
                self._rca_elapsed_time - robot_start_time, self._bca_step_time,
                self._bca_elapsed_time - brain_start_time, time.time() - start)

        return cle.clock

    def loop(self):  # pragma: no cover
        """
        Starts the orchestrated simulations with the profiler activated.
        This function does not return (starts an infinite loop).
        """

        if self._profiler_mode == self.__profiler_modes['cprofile']:
            # runs CLE loop with cProfile
            file_n = '{dir}/cle_nest_stats_{sufix}.pstats'.format(dir=self._profiler_dir,
                                                                  sufix=self._times_loop_started)
            cProfile.runctx("super(DeterministicClosedLoopEngineProfiler, self).loop()",
                            globals(), locals(), file_n)
        else:
            # adds an entry to nest_csv_recorder before starting the loop for the first time
            if self._times_loop_started == 0 and \
                    self._profiler_mode == self.__profiler_modes['cle_step']:
                self.__log_brain_sim_info(self.timestep, csv_recorder=self._brain_sim_csv_recorder)

            super(DeterministicClosedLoopEngineProfiler, self).loop()

        self._times_loop_started += 1

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

        # measure brain load time
        start = time.time()
        super(DeterministicClosedLoopEngineProfiler, self).load_brain(brain_file,
                                                                      **brain_populations)
        brain_time = time.time() - start

        # creates CSVRecorders and logs brain simulation information
        if self._profiler_mode == self.__profiler_modes['cle_step']:
            nest_filename = 'nest_info_{}.csv'.format(self._times_brain_loaded)
            cle_filename = 'cle_time_profile_{}.csv'.format(self._times_brain_loaded)

            self._brain_sim_csv_recorder = self.__log_brain_sim_info(self.timestep, brain_time,
                                                                     csv_filename=nest_filename)

            if self._brain_sim_csv_recorder:
                self._csv_recorders += [self._brain_sim_csv_recorder]

            self._cle_csv_recorder = CSVRecorder(cle_filename, ['robot_step', 'brain_step',
                                                                'brain_refresh', 'cle_step'])

            self._csv_recorders += [self._cle_csv_recorder]

        self._times_brain_loaded += 1

    def shutdown(self):
        """
        Shuts down both simulations.
        """

        super(DeterministicClosedLoopEngineProfiler, self).shutdown()

        # writes csv records to file
        for csv_recoder in self._csv_recorders:
            self.__dump_csv_recorder_to_file(csv_recoder)
