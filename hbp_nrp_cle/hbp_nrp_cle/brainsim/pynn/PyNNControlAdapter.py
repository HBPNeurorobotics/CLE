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
from hbp_nrp_cle.brainsim.pynn import H5PyNNBrainLoader
from hbp_nrp_cle.brainsim.pynn import PyNNPopulationInfo
from hbp_nrp_cle.brainsim.pynn.PyNNInfo import is_population
import hbp_nrp_cle.brainsim as brainsim

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

    def load_brain(self, network_file, **populations):
        """
        Loads the neuronal network contained in the given file

        :param network_file: The path to the neuronal network file
        :param populations: The populations to create
        """
        self.__is_alive = True
        import hbp_nrp_cle.tf_framework.config as tf_config
        tf_config.brain_populations = self.populations_using_json_slice(populations)
        extension = path.splitext(network_file)[1]
        brainsim.simulator = self._sim

        if extension == ".py":
            self.__load_python_brain(
                network_file,
                **self.populations_using_python_slice(populations)
            )
        elif extension == ".h5":
            self.__load_h5_brain(
                network_file,
                **self.populations_using_python_slice(populations)
            )
        else:
            msg = "Neuronal network format {0} not supported".format(extension)
            raise Exception(msg)

    def __load_h5_brain(self, network_file, **populations):
        """
        Loads the brain model in the given h5 file

        :param network_file: The path to the .5h file containing the network
        :param populations: A named list of populations to create
        """
        if not self.__is_initialized:
            self.initialize()
        H5PyNNBrainLoader.load_h5_network(network_file, self._sim, **populations)

    def __load_python_brain(self, network_file, **populations):
        """
        Loads the brain model specified in the given Python script

        :param network_file: The Python file containing the network
        :param populations: A named list of populations to create
        """
        if not self.__is_initialized:
            self.initialize()

        import hbp_nrp_cle.tf_framework.config as tf_config

        tf_config.brain_root = BrainLoader.load_py_network(network_file)
        BrainLoader.setup_access_to_population(tf_config.brain_root, **populations)

        logger.info("Saving brain source")
        with open(network_file) as source:
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
        import hbp_nrp_cle.tf_framework.config as config
        populations = []
        for member in dir(config.brain_root):
            candidate = getattr(config.brain_root, member)
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
        self._sim.run(dt)

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
            if (isinstance(value, slice)):
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
        or dictionnaries of the form {'from': 1, 'to': 10, 'step': 2}
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
