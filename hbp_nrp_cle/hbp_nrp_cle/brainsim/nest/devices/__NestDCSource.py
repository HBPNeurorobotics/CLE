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
Implementation of NestDCSource
"""

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IDCSource

import nest

__author__ = 'LorenzoVannucci'


class NestDCSource(AbstractBrainDevice, IDCSource):
    """
    Represents a direct current generator
    """

    default_parameters = {
        "amplitude": 0.0,
        "start": 0.0,
        "stop": float("inf")
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a direct current generator.

        :param amplitude: Amplitude of direct current, default: 1000.0 pA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """

        super(NestDCSource, self).__init__(**params)
        self._generator = None
        self._last_amplitude_before_deactivation = None
        self.create_device()

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return nest.GetStatus(self._generator, 'amplitude')[0]

    @amplitude.setter
    def amplitude(self, value):  # pragma: no cover
        """
        Sets the amplitude of the current

        :param value: float
        """
        nest.SetStatus(self._generator, {'amplitude': value})

    def _activate(self):
        """Activate this source, if inactive, restoring the previous amplitude value"""
        if not self.active:
            self.amplitude = self._last_amplitude_before_deactivation

    def _deactivate(self):
        """Deactivate this source, if active, setting the amplitude to zero"""
        if self.active:
            self._last_amplitude_before_deactivation = self.amplitude
            self.amplitude = 0.0

    def create_device(self):
        """
        Create a direct current source
        """
        self._generator = nest.Create('dc_generator', 1,
                                      self.get_parameters("amplitude",
                                                          "start",
                                                          "stop"))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a NEST population
        """
        nest.Connect(self._generator, neurons)

    def _disconnect(self):
        """
        Disconnects the device by setting amplitude to 0 since we cannot delete the device or
        connection directly via NEST.
        """
        if self._generator:
            self.amplitude = 0.0
            self._generator = None
