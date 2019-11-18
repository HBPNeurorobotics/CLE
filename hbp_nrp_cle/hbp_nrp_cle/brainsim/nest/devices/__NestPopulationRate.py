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
Implementation of NestPopulationRate
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IPopulationRate
# pylint: disable=no-name-in-module
from scipy.integrate import simps
import numpy as np

import nest

__author__ = 'LorenzoVannucci'


class NestPopulationRate(AbstractBrainDevice, IPopulationRate):
    """
    Represents the rate of a population of LIF neurons by
    measuring and normalizing the membrane potential of a
    leaky integrator with decaying-exponential post-synaptic currents
    """

    default_parameters = {
        'tau_fall': 20.0,
        'tau_rise': 10.0
    }

    fixed_parameters = {
        'V_th': float('inf'),
        'C_m': 1000.0,
        'E_L': 0.0
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is read out and
        normalized. The obligatory threshold voltage 'V_th' is set to
        infinity. The rising time constant is always smaller
        than the falling time constant. If tau_rise > tau_fall,
        "tau_rise" is interpreted as falling time constant and
        "tau_fall" as rising time constant.

        :param tau_rise: Rising time constant, default: 10.0 ms
        :param tau_fall: Falling time constant, default: 20.0 ms
        """
        super(NestPopulationRate, self).__init__(**params)

        self._cell = None
        self._weight = None
        self._rate = None

        self._create_device()
        self._calculate_weight()

    @property
    def rate(self):
        """
        Returns the population firing rate
        """
        return self._rate

    def _update_parameters(self, params):
        super(NestPopulationRate, self)._update_parameters(params)
        self._parameters["tau_rise"], self._parameters["tau_fall"] = \
            sorted(self.get_parameters("tau_rise", "tau_fall").values())

    def _create_device(self):
        """
        Creates a LIF neuron with decaying-exponential post-synaptic currents
        and current-based synapses.
        """
        self._cell = nest.Create('iaf_psc_exp', 1,
                                 self.get_parameters(("tau_m", "tau_fall"),
                                                     ("tau_syn_ex", "tau_rise"),
                                                     "V_th",
                                                     "C_m",
                                                     "E_L"))
        nest.SetStatus(self._cell, {'V_m': self._parameters['E_L']})

    def _calculate_weight(self):
        """
        Calculates the weight of a neuron from the population to the device
        such that the area below the resulting PSP is 1. The exact shape of a
        PSP can be found e.g. in Bytschok, I., Diploma thesis.
        """
        tau_c = (1. / self._parameters['tau_rise'] - 1. / self._parameters['tau_fall']) ** -1
        t_end = -np.log(1e-10) * self._parameters['tau_fall']
        x_new = np.arange(0., t_end, 0.1)
        y_new = tau_c / self._parameters['C_m'] * (np.exp(
            -x_new / self._parameters['tau_fall']) - np.exp(
            -x_new / self._parameters['tau_rise']))
        self._weight = 1.0 / simps(y_new, dx=nest.GetKernelStatus('resolution'))

    # No connection parameters necessary for this device
    # pylint: disable=W0613
    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        nest.Connect(neurons,
                     self._cell,
                     conn_spec='all_to_all',
                     syn_spec={'model': 'static_synapse',
                               'weight': self._weight * 1000,
                               'delay': nest.GetKernelStatus('resolution')})

    def _disconnect(self):
        """
        Disconnects the rate recorder by disabling voltage recording and setting
        rate to 0 since we cannot delete the neuron or synapses via NEST.
        """
        if self._cell:
            self._cell = None
            self._rate = 0

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    def refresh(self, time):
        """
        Refreshes the rate value

        :param time: The current simulation time
        """
        self._rate = nest.GetStatus(self._cell, 'V_m')[0]
