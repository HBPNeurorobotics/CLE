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
Implementation of PyNNFixedSpikeGenerator
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNFixedSpikeGenerator
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice

import pyNN.nest as nestsim

__author__ = 'DimitriProbst, Sebastian Krach'


class PyNNNestFixedSpikeGenerator(PyNNFixedSpikeGenerator, PyNNNestDevice):
    """
    Represents a spike generator which generated equidistant
    spike times at a given frequency
    """

    @property
    def rate(self):
        """
        Returns the frequency of the Fixed spike generator
        """
        return self._rate

    def sim(self):
        """
        Gets the simulator module to use
        """
        return nestsim

    # Pylint does not really recognize property overrides
    # pylint: disable=arguments-differ
    @rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Fixed spike generator

        :param value: float
        """
        self._rate, current = self._calculate_rate_and_current(value)

        if current != self._current:
            self._current = current
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._currentsource._device, {"amplitude": 1000.0 * current})

    def create_device(self):
        """
        Create a fixed spike-distance device
        """

        self._generator = self.sim().Population(1, self.sim().IF_curr_exp(
            **self.get_parameters("cm",
                                  "tau_m",
                                  "v_thresh",
                                  "v_reset",
                                  "v_rest")))

        params = self.get_parameters("v_rest")
        self.sim().initialize(self._generator, v=params["v_rest"])

        self._currentsource = self.sim().DCSource(amplitude=self._current)
        self._currentsource.inject_into(self._generator)
