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
Implementation of PyNNNCSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNNCSource
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice, \
    create_transformation
import pyNN.nest as nestsim

__author__ = 'Georg Hinkel, Dimitri Probst'


class PyNNNestNCSource(PyNNNestDevice, PyNNNCSource):
    """
    Represents a noisy current generator
    :param kwargs: Optional configuration parameters
    """

    transformations = {
        "mean": create_transformation("mean", 1000.0),
        "stdev": create_transformation("std", 1000.0)
    }

    @property
    def mean(self):
        """
        Returns the mean of the current
        """
        return self._parameters["mean"]

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ,function-redefined
    @PyNNNCSource.mean.setter
    def mean(self, value):
        """
        Sets the mean of the current

        :param value: float
        """

        if self.mean != value:
            self._parameters["mean"] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {'mean': 1000.0 * value})

    def sim(self):
        """
        Gets the simulator module to use
        """
        return nestsim

    @property
    def stdev(self):
        """
        Returns the stdev of the current
        """
        return self._parameters["stdev"]

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ,function-redefined
    @PyNNNCSource.stdev.setter
    def stdev(self, value):
        """
        Sets the stdev of the current

        :param value: float
        """
        if self.stdev != value:
            self._parameters["stdev"] = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            self.SetStatus(self._generator._device, {'std': 1000.0 * value})
