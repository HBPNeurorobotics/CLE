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
Implementation of ISpikeInjector
'''

from hbp_nrp_cle.brainsim.common.devices.__AbstractBrainDevice import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeInjector
from hbp_nrp_cle.brainsim.pynn_spiNNaker import spynnaker
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as live_connections
from hbp_nrp_cle.brainsim.pynn.devices.__PyNNDictParser import set_connector, set_synapse_type
import logging

logger = logging.getLogger(__name__)


class SpiNNakerSpikeInjector(AbstractBrainDevice, ISpikeInjector):  # pragma no cover
    """
    An implementation of the spike injector interface
    """

    default_parameters = {
        "connector": None,
        "weight": 2.0,
        "delay": 1.0,
        "source": None,
        "receptor_type": "excitatory",
        "synapse_type": None,
        "label": None,
        "port": None
    }

    def __init__(self, **params):
        super(SpiNNakerSpikeInjector, self).__init__(**params)
        self.__connection = None
        self.__neuron_ids = None
        self.__label = None

    def _disconnect(self):
        """
        Disconnects the live connection
        """
        if self.__connection is not None:
            self.__connection = None

    # pylint: disable=W0221
    def inject_spikes(self, neuron_ids=None):
        """
        Injects a spike to the connected population

        :param neuron_ids: The ids of the neurons to inject, or None for all
        """
        neurons_to_send = neuron_ids
        if neuron_ids is None:
            neurons_to_send = self.__neuron_ids
        if self.__connection is not None:
            self.__connection.send_spikes(self.__label, neurons_to_send)
        else:
            logger.warn("Spike could not be injected as spike connection not started, yet")

    def connect(self, neurons):
        """
        Connects the spike injector to the specified neurons

        :param neurons: A population of neurons
        """
        live_connections.register_sender(
            neurons, self.__started, **self.get_parameters(
                "source", "receptor_type", "connector", "synapse_type"))
        self.__neuron_ids = list(range(neurons.size))

    def _update_parameters(self, params):
        """
        This method updates the device parameter dictionary with the provided parameter
        dictionary. The dictionary has to be validated before as this method assumes it to be
        correct.

        Overriding subclasses can provide additional configuration parameter adaption as this
        method is called before the brain simulator devices are constructed.

        :param params: The validated parameter dictionary
        """
        super(SpiNNakerSpikeInjector, self)._update_parameters(params)
        set_connector(self._parameters, spynnaker, params)
        set_synapse_type(self._parameters, spynnaker)

    # pylint: disable=unused-argument
    def __started(self, pop_label, connection):
        """
        Gets called when the spike connection is ready
        :param pop_label:
        :param connection:
        :return:
        """
        if self.__connection is None:
            logger.info("Spike injector ready to inject")
        elif self.__connection is not connection:
            raise Exception("Spike injector already has a connection assigned")
        self.__connection = connection
        self.__label = pop_label
