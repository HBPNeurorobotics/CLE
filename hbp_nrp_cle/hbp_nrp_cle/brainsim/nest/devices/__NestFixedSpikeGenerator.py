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
Implementation of NestFixedSpikeGenerator
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator
from hbp_nrp_cle.brainsim.nest.devices.__NestDictParser import set_synapse_type, set_connector
import numpy as np

import nest

__author__ = 'LorenzoVannucci'


class NestFixedSpikeGenerator(AbstractBrainDevice, IFixedSpikeGenerator):
    """
    Represents a spike generator which generates equidistant
    spike times at a given frequency
    """

    default_parameters = {
        'initial_rate': 0.0,
        'C_m': 1000.0,
        'tau_m': 1000.0,
        't_ref': 0.2,
        'V_th': -50.0,
        'V_reset': -100.0,
        'E_L': -100.0,
        'connector': None,
        'weight': 0.15,
        'delay': 0.1,
        'receptor_type': 'excitatory',
        'synapse_type': None
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a Fixed spike generator.

        :param rate: Rate/frequency of spike train, default: 0.0 Hz
        :param connector: a connector type, or, if neurons is
            a list of two populations, a list of two connectors
        :param receptor_type: string specifying which synapse on the postsynaptic cell
            to connect to: excitatory or inhibitory. If neurons is a list of
            two populations, target is ['excitatory', 'inhibitory'], dafault is
            excitatory
        :param synapse_type: a synapse type
        """
        super(NestFixedSpikeGenerator, self).__init__(**params)

        self._generator = None
        self._currentsource = None

        self._calculate_rate_and_current = self._setup_rate_and_current_calculation()

        (self._rate,
         self._current) = self._calculate_rate_and_current(self._parameters["initial_rate"])

        self._last_rate_before_deactivation = None

        self.create_device()

    @property
    def rate(self):
        """
        Returns the frequency of the Fixed spike generator
        """
        return self._rate

    @rate.setter
    def rate(self, value):  # pragma: no cover
        """
        Sets the frequency of the Fixed spike generator

        :param value: float
        """
        self._rate, current = self._calculate_rate_and_current(value)

        if current != self._current:
            self._current = current
            nest.SetStatus(self._currentsource, {'amplitude': current})

    def _activate(self):
        """
        Activates the Fixed spike generator restoring the last value of the rate
        """
        if not self.active:
            self.rate = self._last_rate_before_deactivation

    def _deactivate(self):
        """
        Deactivates the Fixed spike generator setting the rate to zero
        """
        if self.active:
            self._last_rate_before_deactivation = self.rate
            self.rate = 0.0

    def create_device(self):
        """
        Create a fixed spike-distance device
        """
        self._generator = nest.Create('iaf_psc_exp', 1,
                                      self.get_parameters("C_m",
                                                          "tau_m",
                                                          "V_th",
                                                          "V_reset",
                                                          "E_L"))
        nest.SetStatus(self._generator, {'V_m': self._parameters['E_L']})

        self._currentsource = nest.Create('dc_generator', 1, {'amplitude': self._current})
        nest.Connect(self._currentsource, self._generator)

    def _setup_rate_and_current_calculation(self):
        """
        This method sets up the calculation of the suitable current based on specified spiking
        rate values. As this calculation is dependent on neuron parameters which are only set
        once, this method returns a callable which expects the desired spiking rate and returns a
        tuple of closest achievable rate and the appropriate current in pA.

        :return: a callable function: float --> (float, float)
        """
        tau_m, tau_refrac, cm, v_thresh, v_rest = \
            self.get_parameters("tau_m", "t_ref", "C_m", "V_th", "E_L").values()

        def calculate_rate_and_current(rate):
            """
            Returns current in nA corresponding to frequency "rate". If the specified spiking rate
            is not achievable due to the neuron configuration the current is determined for the
            possible maximum.

            :param rate: Frequency in Hz
            :return: The frequency which results from injecting the configured neuron with the
                    resulting current in Hz.
            :return: The resulting dc current
            """
            nom = (cm / tau_m) * (v_thresh - v_rest)
            denom = 1.0
            if rate != 0.0:
                inter_time = 1000.0 / rate
                if inter_time < 10 * tau_refrac:
                    rate = 100.0 / tau_refrac
                    inter_time = 10 * tau_refrac
                denom = 1.0 - np.exp((tau_refrac - inter_time) / tau_m)
            return rate, nom / denom

        return calculate_rate_and_current

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        syn_dict = self._parameters["synapse_type"].copy()
        syn_dict["weight"] = self._parameters["weight"]
        syn_dict["delay"] = self._parameters["delay"]
        nest.Connect(self._generator,
                     neurons,
                     conn_spec=self.get_parameters("connector")["connector"],
                     syn_spec=syn_dict)

    def _disconnect(self):
        """
        Disconnects the device by setting rate to 0 since we cannot delete the device or
        connection directly via NEST.
        """
        if self._currentsource:
            self.rate = 0.0
            self._generator = None
            self._currentsource = None

    def _update_parameters(self, params):
        """
        This method updates the device parameter dictionary with the provided parameter
        dictionary. The dictionary has to be validated before as this method assumes it to be
        correct.

        Overriding subclasses can provide additional configuration parameter adaption as this
        method is called before the brain simulator devices are constructed.

        :param params: The validated parameter dictionary
        """
        super(NestFixedSpikeGenerator, self)._update_parameters(params)
        set_synapse_type(self._parameters)
        set_connector(self._parameters, params)
