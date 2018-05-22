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
This module contains an adapted implementation of a neural controller for SpiNNaker
"""

from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter, PyNNPopulationInfo
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connection
import logging
logger = logging.getLogger(__name__)


def all_ids(population):
    """
    Returns all ids of the neurons

    :param population: The population
    """
    # pylint: disable=protected-access
    return population._all_ids


class PySpiNNakerControlAdapter(PyNNControlAdapter):
    """
    An implementation to control a SpiNNaker board simulation synchronously
    """

    def __init__(self, sim):
        super(PySpiNNakerControlAdapter, self).__init__(sim)
        sim.Population.all = all_ids

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
        if not 'timestep' in params:
            params['timestep'] = 1.0
        if not 'min_delay' in params:
            params['min_delay'] = 1.0
        # store logging setup
        formatter = logging.root.handlers[0].formatter
        super(PySpiNNakerControlAdapter, self).initialize(**params)
        # restore logging setup
        for handler in logging.root.handlers:
            handler.setFormatter(formatter)
            if len(handler.filters) > 0:
                handler.removeFilter(handler.filters[-1])

    def run_step(self, dt):
        live_connection.create_and_start_connections()
        try:
            super(PySpiNNakerControlAdapter, self).run_step(dt)
        # it may happen that the simulation wants to write provenance data even though the temp
        # directory no longer exists. In that case, the simulation is about to terminate, so
        # we ignore the error, but log it
        except IOError, e:
            logger.exception(e)

    def _is_population(self, candidate):
        """
        Determines whether the candidate is a population

        :param candidate: The candidate
        """
        return isinstance(candidate, self._sim.Population)

    def _create_population_info(self, population, name):
        """
        Creates a population info object for the given population

        :param population: The population
        :param name: The name of the population
        """
        celltype = population.celltype
        parameters = dict()
        for parameter_name in celltype.default_parameters:
            parameters[parameter_name] = celltype.get_value(parameter_name)
        return PyNNPopulationInfo(population, name, parameters)