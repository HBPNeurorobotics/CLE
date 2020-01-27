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

from spinn_front_end_common.utilities.database.database_connection import DatabaseConnection
from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter, PyNNPopulationInfo
from hbp_nrp_cle.cle.CLEInterface import BrainRuntimeException
import logging
from hbp_nrp_excontrol.logs import clientLogger
from threading import Thread, Condition

logger = logging.getLogger(__name__)


def all_ids(population):
    """
    Returns all ids of the neurons

    :param population: The population
    """
    # pylint: disable=protected-access
    return population._all_ids


class PySpiNNakerControlAdapter(PyNNControlAdapter): # pragma no cover
    """
    An implementation to control a SpiNNaker board simulation synchronously
    """

    def __init__(self, sim):
        super(PySpiNNakerControlAdapter, self).__init__(sim)
        sim.Population.all = all_ids
        self._running = False
        self._ready = False
        self._exception = None
        self._ready_sync = Condition()

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
        self._running = False
        self._ready = False

    def _notify_ready(self):
        """
        Notify
        """
        with self._ready_sync:
            self._ready = True
            self._ready_sync.notify_all()

    def _do_run(self):
        """
        Do run
        """
        try:
            self._sim.external_devices.run_forever()

        # pylint: disable=W0703
        except Exception as e:
            logger.exception(e)
            with self._ready_sync:
                self._running = False
                self._ready = False
                self._exception = e
                self._ready_sync.notify_all()

    def run_step(self, dt):
        if not self._running:
            self._running = True
            clientLogger.advertise("Brain is loading to the Spinnaker Board. "
                                   "This can take a couple of seconds.")
            connection = DatabaseConnection(
                start_resume_callback_function=self._notify_ready,
                local_port=None)
            self._sim.external_devices.add_database_socket_address(
                database_notify_host=None,
                database_notify_port_num=connection.local_port,
                database_ack_port_num=None)
            runner = Thread(target=self._do_run)
            runner.start()
            with self._ready_sync:
                while not self._ready and self._exception is None:
                    self._ready_sync.wait()
            clientLogger.advertise("Brain loading to the Spinnaker Board has been finished.")
            if self._exception is not None:
                raise BrainRuntimeException(str(self._exception))

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
        parameters = dict(celltype.default_parameters)
        return PyNNPopulationInfo(population, name, parameters)

    def get_Timeout(self):
        """
        returns The maximum amount of time (in seconds) to wait for the end of this step
        """
        return None
