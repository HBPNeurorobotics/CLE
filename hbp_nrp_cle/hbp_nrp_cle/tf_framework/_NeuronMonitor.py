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
Defines a neuron monitor
"""

import logging
import sys
from ._TransferFunction import TransferFunction
from ._Neuron2Robot import MapSpikeSink
from ._Robot2Neuron import MapRobotPublisher
from ._NeuronSelectors import MapNeuronSelector
from hbp_nrp_cle.tf_framework import config, TFRunningException
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder, ILeakyIntegratorAlpha, \
    ILeakyIntegratorExp, IPopulationRate
from hbp_nrp_cle.robotsim.RobotInterface import Topic
from cle_ros_msgs.msg import SpikeRate, SpikeEvent, SpikeData

logger = logging.getLogger(__name__)

__author__ = 'Georg Hinkel'

SPIKE_RECORDER_TOPIC = "/monitor/spike_recorder"
LEAKY_INTEGRATOR_ALPHA_TOPIC = "/monitor/leaky_integrator_alpha"
LEAKY_INTEGRATOR_EXP_TOPIC = "/monitor/leaky_integrator_exp"
POPULATION_RATE_TOPIC = "/monitor/population_rate"


class NeuronMonitor(TransferFunction):
    """
    Class to represent transfer functions used for monitoring neurons
    """

    def __init__(self, neurons, monitor_type):
        """
        Defines a new transfer function from robots to neurons

        :param neurons: The neurons that should be monitored
        :param monitor_type: The type of monitor that should be injected
        """
        super(NeuronMonitor, self).__init__()
        self.__count = 0

        cfg = {}
        _topic = None
        _type = SpikeRate
        if monitor_type is ISpikeRecorder:
            self.__handler = self.__send_spike_recorder
            _topic = SPIKE_RECORDER_TOPIC
            _type = SpikeEvent
            cfg["use_ids"] = False
        elif monitor_type is ILeakyIntegratorExp:
            self.__handler = self.__send_leaky_integrator
            _topic = LEAKY_INTEGRATOR_EXP_TOPIC
        elif monitor_type is ILeakyIntegratorAlpha:
            self.__handler = self.__send_leaky_integrator
            _topic = LEAKY_INTEGRATOR_ALPHA_TOPIC
        elif monitor_type is IPopulationRate:
            self.__handler = self.__send_population_rate
            _topic = POPULATION_RATE_TOPIC
        else:
            raise Exception("Type {0} is not a supported monitor type"
                            .format(monitor_type.__name__))

        self.__publisher_spec = MapRobotPublisher("publisher", Topic(_topic, _type))
        self.__device_spec = MapSpikeSink("device", neurons, monitor_type, **cfg)

        self.device = None
        self.publisher = None

    def __call__(self, func):  # -> Neuron2Robot:
        """
        Applies the transfer functions object to the given function

        :param func: The function body for this transfer function
        :return The transfer function object
        """
        self._init_function(func, config.active_node.n2r)
        if self.__publisher_spec not in self._params:
            self._params.append(self.__publisher_spec)
        if self.__device_spec not in self._params:
            self._params.append(self.__device_spec)
        return self

    def __repr__(self):  # pragma: no cover
        return "{0} monitors {1}" \
            .format(self.name, self.__device_spec.neurons)

    def initialize(self, tfm, bca_changed, rca_changed):
        """
        Initializes this transfer function to be used with the given TFM

        :param tfm: The Transfer Function Manager
        :param bca_changed: True, if the brain communication adapter has changed
        :param rca_changed: True, if the robot communication adapter has changed
        """
        if bca_changed:
            if hasattr(self.device, 'neurons'):

                if type(self.device.neurons_count) is list:
                    self.__count = sum(self.device.neurons_count)
                else:
                    self.__count = self.device.neurons_count
            else:
                self.__count = None

    def __send_spike_recorder(self, t):
        """
        Sends spike data to the given spike recorder monitoring topic

        :param t: The simulation time
        :return:
        """

        if hasattr(self.device, 'device_type'):
            # TODO: Since the import of PyNNSpiNNakerSpikeRecorder does not work, we had to add
            # a string comparison here in order to check if the device_type of the current device
            # is a PyNNSpiNNakerSpikeRecorder. This is not a satisfying solution and should be
            # fixed.
            if 'PyNNSpiNNakerSpikeRecorder' in str(self.device.device_type):
                self.__send_spike_recorder_spinnaker(t)
                return

        times = self.device.times

        if type(times) is not list:
            times = [times]

        population_names = []

        # TODO: week solution, a deeper refactoring of population referencing is required
        # See: [NRRPLT-7609] [NRRPLT-6822] [NRRPLT-7099]
        def get_name_from_ppath(ppath):
            """
            Extracts population name from a property path
            """
            return ppath if '(root)' not in ppath else ppath[ppath.find('.') + 1:ppath.find('[')]

        if isinstance(self.__device_spec.neurons, MapNeuronSelector):
            nrange = self.__device_spec.neurons.neuron_range
            nmapping = self.__device_spec.neurons.mapping

            # TODO: another case in nrange (_NeuronSelectors.py:81) is not been considered
            if isinstance(nrange, int):
                nrange = range(0, nrange)

            for i in nrange:
                ppath = str(nmapping(i).name)
                population_names += [get_name_from_ppath(ppath)]
        else:
            ppath = str(self.__device_spec.neurons.name)
            population_names += [get_name_from_ppath(ppath)]

        for population_name, spikes in zip(population_names, times):

            # Create a message that contains the spike data
            msgs = [SpikeData(int(spike[0]), spike[1]) for spike in spikes]

            # Send message
            self.publisher.send_message(
                SpikeEvent(t, self.__count, msgs, self.name, str(population_name)))

    def __send_spike_recorder_spinnaker(self, t):
        """
        Sends spike data from the Spinnaker board

        :param t: The simulation time
        :return:
        """

        for device in self.device.devices:
            # Create a message that contains the spike data
            msgs = [SpikeData(int(spike[0]), spike[1]) for spike in device.times]

            # Send message
            self.publisher.send_message(
                SpikeEvent(t, self.__count, msgs, self.name, str(device.population_name)))

    def __send_leaky_integrator(self, t):
        """
        Sends spike data to the given leaky integrator monitoring topic

        :param t: The simulation time
        :return:
        """
        self.publisher.send_message(SpikeRate(t, self.device.voltage, self.name))

    def __send_population_rate(self, t):
        """
        Sends spike data to the given population rate monitoring topic

        :param t: The simulation time
        :return:
        """
        self.publisher.send_message(SpikeRate(t, self.device.rate, self.name))

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time

        :param t: The simulation time
        """
        # pylint: disable=broad-except
        try:
            self._params[0] = t
            return_value = self._func(*self._params[:-2])
            if return_value is not None:
                self.__handler(t)
        except Exception, e:
            self._handle_error(e, sys.exc_info()[2])
            raise TFRunningException(str(e))

    def unregister(self):
        """
        Unregisters the device for this transfer function. It will no longer produce
        messages. Use the full brain adapter unregistration to allow custom adapters
        to properly handle the event.

        Leave the publisher alone otherwise the console fills with many warnings about
        the topic not being published, without a device it won't be usable anyway.
        """
        if self.device is not None:
            config.active_node.brain_adapter.unregister_spike_sink(self.device)
            self.device = None
