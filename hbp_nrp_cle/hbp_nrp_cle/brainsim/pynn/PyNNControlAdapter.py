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
'''
PyNNControlAdapter.py
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim import IBrainControlAdapter
from hbp_nrp_cle.brainsim.common import PythonBrainLoader as BrainLoader
from hbp_nrp_cle.brainsim.pynn import PyNNPopulationInfo
from hbp_nrp_cle.brainsim.pynn.PyNNInfo import is_population
import hbp_nrp_cle.brainsim as brainsim
from hbp_nrp_cle.cle.CLEInterface import BrainRuntimeException

import hbp_nrp_cle.tf_framework.config as tf_config

import logging
from os import path
import copy

logger = logging.getLogger(__name__)

__author__ = 'Dimitri Probst', 'Daniel Peppicelli', 'Luc Guyot'


class PyNNControlAdapter(IBrainControlAdapter):
    """
    Represents a controller object for the neuronal simulator
    """

    def __init__(self, sim):
        """
        Initializes the PyNN control adapter

        :param sim: The simulator module
        """
        self.__is_initialized = False
        self.__is_alive = False
        self.__rank = None
        self._sim = sim

    def load_populations(self, **populations):
        """
        Load (or reload) populations defined on the currently loaded brain.

        :param populations: A dictionary indexed by population names and
          containing neuron indices. Neuron indices can be defined by a single integer,
          list of integers or python slices. Python slices can be replaced by a
          dictionary containing the 'from', 'to' and 'step' values.
        :raise Exception: When no brain is currently loaded
        """

        # check if a valid brain is loaded
        if tf_config.brain_root is None:
            raise Exception("No brain is currently loaded, cannot add Populations.")

        if not hasattr(tf_config.brain_root, 'circuit'):
            raise AttributeError("No circuit is found in the currently loaded brain:"
                                 "Cannot add Populations.")

        # valid brain found, load populations
        tf_config.brain_populations = self.populations_using_json_slice(populations)

        if not hasattr(tf_config.brain_root, 'populations_keys'):
            tf_config.brain_root.populations_keys = []
        BrainLoader.clear_populations(tf_config.brain_root)

        BrainLoader.setup_access_to_population(
            tf_config.brain_root, **self.populations_using_python_slice(populations))

    def load_brain(self, brain_file, **populations):
        """
        Loads the neuronal network contained in the given file

        :param brain_file: The path to the neuronal network file
        :param populations: Additional populations to create (Optional)
        """
        extension = path.splitext(brain_file)[1]
        brainsim.simulator = self._sim

        if extension == ".py":
            self.__load_py_brain(brain_file)

            # load new populations if any, otherwise reload current ones (if any)
            # otherwise initialize empty dict
            if populations:
                pops = populations
            elif tf_config.brain_populations:
                pops = tf_config.brain_populations
            else:
                pops = {}

            self.load_populations(**pops)

        else:
            msg = "Neuronal network format {0} not supported".format(extension)
            raise Exception(msg)

    def __load_py_brain(self, brain_file):
        """
        Loads the brain model specified in the given Python script

        :param brain_file: The Python file containing the network
        """
        self.__is_alive = True
        if not self.__is_initialized:
            self.initialize()

        tf_config.brain_root = BrainLoader.load_py_network(brain_file)

        logger.info("Saving brain source")
        with open(brain_file) as source:
            tf_config.brain_source = source.read()

    def initialize(self, **params):
        """
        Initializes the neuronal simulator

        :param timestep: The timestep used for the neuronal simulation
        :param min_delay: The minimum delay
        :param max_delay: The maximum delay
        :param threads: The amount of threads that should be used to run the simulation
        :param rng_seeds: The rng seeds for the simulation
        :return: True if the simulator is initialized, otherwise False
        """
        if not self.__is_initialized:
            timestep = params.get('timestep', 0.1)
            min_delay = params.get('min_delay', "auto")
            max_delay = params.get('max_delay', 20.0)
            self.__rank = self._sim.setup(timestep=timestep, min_delay=min_delay,
                                          max_delay=max_delay)
            self.__is_initialized = True
            logger.info("neuronal simulator initialized")
        else:
            logger.warn("trying to initialize an already initialized controller")
        return self.__is_initialized

    # pylint: disable=no-self-use
    def _is_population(self, candidate):
        """
        Determines whether the candidate is a population

        :param candidate: The candidate
        """
        return is_population(candidate)

    # pylint: disable=no-self-use
    def _create_population_info(self, population, name):
        """
        Creates a population info object for the given population

        :param population: The population
        :param name: The name of the population
        """
        try:
            celltype = population.celltype.parameter_space
            parameters = {a: celltype[a].base_value for a in celltype.keys()}
        except AttributeError:
            parameters = {}
        return PyNNPopulationInfo(population, name, parameters)

    def __find_all_populations(self, candidate, member_name, populations):
        """
        Finds all populations under the given object and adds them to the list of populations

        :param candidate: The object tree
        :param member_name: The base member name
        :param populations: The list of populations
        """
        if self._is_population(candidate):
            populations.append(self._create_population_info(candidate, member_name))
        elif isinstance(candidate, list):
            for index in range(len(candidate)):
                self.__find_all_populations(candidate[index],
                                            member_name + "[" + str(index) + "]",
                                            populations)

    def get_populations(self):
        """
        Gets an information about the populations currently available

        :return: A list of population infos
        """
        populations = []
        for member in dir(tf_config.brain_root):
            candidate = getattr(tf_config.brain_root, member)
            self.__find_all_populations(candidate, member, populations)
        return populations

    @property
    def is_initialized(self):
        """
        Gets a value indicating whether initialize has been called
        """
        return self.__is_initialized

    def is_alive(self):  # -> bool:
        """
        Gets a status whether the neuronal simulator is still alive

        :return: True if the simulator is alive, otherwise False
        """
        return self.__is_alive

    def run_step(self, dt):  # -> None:
        """
        Runs the neuronal simulator for the given amount of simulated time

        :param dt: the simulated time in milliseconds
        """
        try:
            self._sim.run(dt)
        except Exception as e:
            raise BrainRuntimeException(str(e))

    def shutdown(self):  # -> None:
        """
        Shuts down the neuronal simulator
        """
        self.__is_alive = False
        self.__is_initialized = False
        self._sim.end()
        logger.info("neuronal simulator ended")

    def reset(self):  # -> None:
        """
        Resets the neuronal simulator
        """
        logger.info("neuronal simulator reset")

    @staticmethod
    def populations_using_json_slice(populations):
        """
        Turn populations defined as python slices into python dicts
        to allow straightforward translation in to json.

        :param populations: a dictionary whose values are either
        python lists or slices.
        Slices can be of two types, either python slices,
        or dictionnaries of the form {'from': 1, 'to': 10, 'step': 2}
        :return: A dictionary where python slices have been replaced
        by dictionaries referred to as 'json slices'.
        """
        result = copy.deepcopy(populations)
        for key, value in populations.iteritems():
            if isinstance(value, slice):
                p = {'from': value.start, 'to': value.stop, 'step': value.step}
                result[key] = p
        return result

    @staticmethod
    def populations_using_python_slice(populations):
        """
        Turn slices defined as python dicts
        into python slices.
        Populations of type list are left unchanged.

        :param populations: a dictionary whose values are either
        python lists or slices.
        Slices can be of two types, either python slices,
        or dictionaries of the form {'from': 1, 'to': 10, 'step': 2}
        :return: A dictionary where 'json slices' (plain python dicts) have been replaced
        by python slices.
        """
        result = copy.deepcopy(populations)
        for key, value in populations.iteritems():
            if isinstance(value, dict):
                if 'from' in value and 'to' in value:
                    step = value.get('step')
                    result[key] = slice(value['from'], value['to'], step)
                elif 'list' in value:
                    value_list = value.get('list')
                    result[key] = list(value_list)

        return result

    def get_Timeout(self):
        """
        returns The maximum amount of time (in seconds) to wait for the end of this step
        """
        return 5
