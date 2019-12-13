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
Implementation of PyNNLeakyIntegratorExp
"""
import decimal
from hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegratorTypes import PyNNLeakyIntegratorExp
from hbp_nrp_cle.brainsim.pynn_spiNNaker import spynnaker as sim
from hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection import register_devices, \
    get_key
import logging
logger = logging.getLogger(__name__)
try:
    # pylint: disable=import-error
    from data_specification.enums import DataType
    from spynnaker.pyNN.external_devices_models import AbstractMulticastControllableDevice
    from spynnaker.pyNN.external_devices_models.abstract_multicast_controllable_device \
        import SendType
except ImportError:  # pragma: no cover
    AbstractMulticastControllableDevice = object
    from enum import Enum

    class DataType(object):
        """A mock of a data type used in case Spinnaker is not installed"""
        class S1615(object):
            """An S1615 is a fixed-point floating point number"""
            max = 65000

    def get_simulator():
        """A dummy for the get_simulator method in case Spinnaker is not installed"""
        return None

    class SendType(Enum):
        """ The data type to be sent in the payload of the multicast packet
        """
        SEND_TYPE_INT = 0
        SEND_TYPE_UINT = 1
        SEND_TYPE_ACCUM = 2
        SEND_TYPE_UACCUM = 3
        SEND_TYPE_FRACT = 4
        SEND_TYPE_UFRACT = 5

__author__ = 'Georg Hinkel'


class PyNNSpiNNakerLeakyIntegratorExp(PyNNLeakyIntegratorExp,
                                      AbstractMulticastControllableDevice): # pragma no cover
    """
    Represents the membrane potential of a current-based LIF neuron
    with decaying-exponential post-synaptic currents
    """

    def sim(self):
        return sim

    default_parameters = {
        'cm': 1.0,
        'tau_m': 20.0,
        'tau_syn_E': .5,
        'tau_syn_I': .5,
        'v_rest': 0.0,
        'v_reset': 0.0,
        'tau_refrac': 0.1,
        'i_offset': 0.0,
        'connector': None,
        'weight': None,
        'delay': 1.0,
        'key': None,
        'timesteps': 10,
        'source': None,
        'receptor_type': 'excitatory',
        'synapse_type': None,
        'partition': None
    }

    def __init__(self, **params):
        super(PyNNSpiNNakerLeakyIntegratorExp, self).__init__(**params)

    def _update_parameters(self, params):
        """
        Updates the parameters

        :param params: Configuration parameters
        """
        super(PyNNSpiNNakerLeakyIntegratorExp, self)._update_parameters(params)
        self._parameters['key'] = get_key(self._parameters['key'])

    def create_device(self):
        """
        Creates the device

        The communication component itself acts as device, so nothing to do here
        """
        pass

    def start_record_voltage(self):
        """
        Starts to record voltages

        This device does not support dynamic changes
        """
        pass

    def stop_record_voltage(self):
        """
        Stops to record voltages

        This device does not support dynamic changes
        """
        pass

    def refresh(self, time):
        """
        Synchronizes the leaky integrator with the simulation time

        :param time: The current simulation time
        """
        pass

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the
        device. The connection structure is specified via the
        PyNN connection object "connector". If "connector" is None,
        the weights and delays between the neurons and the device
        are sampled from a uniform distribution.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        population = register_devices([self], **self.get_parameters(
            'cm',
            'tau_m',
            'tau_syn_E',
            'tau_syn_I',
            'v_rest',
            'v_reset',
            'tau_refrac',
            'i_offset'
        ))
        logger.info("Creating projection from {n} to {p} with connector {c} and"
                    " synapse type {st}, weight {w}".format(
                        n=neurons, p=population, c=self._parameters['connector'],
                        st=self._parameters['synapse_type'],
                        w=self._parameters['synapse_type'].weight))
        sim.Projection(neurons, population,
                       connector=self._parameters['connector'],
                       synapse_type=self._parameters['synapse_type'],
                       source=self._parameters['source'],
                       receptor_type=self._parameters['receptor_type'])
        if self._parameters['partition'] is None:
            self._parameters['partition'] = neurons.label

    def run(self, voltage):
        """
        Tells the device that a new value is available

        :param voltage: The membrane potential
        """
        # Store all voltages since last requested
        # pylint: disable=E1101
        # pylint: disable=E1103
        self._voltage = (float)(decimal.Decimal(voltage) / DataType.S1615.scale)

    @property
    def device_control_key(self):
        """
        Gets the key used to identify this device

        :return: The key that spinnaker uses send multicast commands to this device
        """
        return self._parameters['key']

    @property
    def device_control_partition_id(self):
        """
        Gets the partition ID

        :return: A partition ID to inform spinnaker where the respective device can be allocated
        """
        return self._parameters['partition']

    # pylint: disable=no-self-use
    @property
    def device_control_uses_payload(self):
        """
        Returns True, as the payload is used to carry the membrane potential

        :return: True
        """
        return True

    # pylint: disable=no-self-use
    @property
    def device_control_min_value(self):
        """
        Gets the minimum value that one can expect as voltage

        :return: 0
        """
        return 0

    # pylint: disable=no-self-use
    @property
    def device_control_max_value(self):
        """
        Gets the maximum value that one can expect as voltage

        :return: 2^15
        """
        # pylint for some reason does not understand the spinnaker enums
        # pylint: disable=no-member
        return DataType.S1615.max

    @property
    def device_control_timesteps_between_sending(self):
        """
        Gets the number of timesteps that occur between subsequent multicast commands
        """
        return self._parameters['timesteps']

    @property
    def device_control_send_type(self):
        """
        Returns the device control send type

        :return: the device control send type
        """
        return SendType.SEND_TYPE_ACCUM

    def _get_connector_weight(self):
        """
        Returns the default connector weight in case no explicit weight is specified as parameter

        :return: the weight of the synaptic connection
        """
        return 0.1
