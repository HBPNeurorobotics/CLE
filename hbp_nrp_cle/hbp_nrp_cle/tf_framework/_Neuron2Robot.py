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
This module contains the representation from transfer functions from the neuronal simulator towards
the world simulator
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import Topic
from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, IPopulationRate, ISpikeInjector, \
    ICustomDevice, IBrainCommunicationAdapter, ISpikeRecorder, IRawSignal
from ._MappingSpecification import ParameterMappingSpecification
from . import config
from ._TransferFunction import TransferFunction
import sys

import logging
logger = logging.getLogger(__name__)


class MapSpikeSink(ParameterMappingSpecification):
    """
    Class to map parameters to spike sinks such as leaky integrators
    """

    supported_device_types = [ILeakyIntegratorAlpha, ILeakyIntegratorExp,
                              IPopulationRate, ISpikeRecorder, IRawSignal]

    def __init__(self, key, value, device_type, **kwargs):  # -> None:
        """
        Maps a parameter to a neuron

        :param key: the parameter name
        :param value: the neuron reference
        :param kwargs: Additional configuration
        :param device_type: The type of device that should be created at the referenced neurons
        """
        super(MapSpikeSink, self).__init__(key)
        self.__value = value
        if not isinstance(device_type, ICustomDevice):
            if not self.is_supported(device_type):
                raise Exception("Device type {} is not supported".format(device_type.__name__))
        self.__device_type = device_type
        self.__config = kwargs

    def is_supported(self, device_type):  # pylint: disable=R0201
        """
        Gets a value indicating whether the given device type is supported

        :param device_type: The device type that should be looked for
        """
        return device_type in MapSpikeSink.supported_device_types

    @property
    def neurons(self):
        """
        Gets the neurons referenced by this mapping
        """
        return self.__value

    @property
    def device_type(self):
        """
        Gets the device type referenced by this mapping
        """
        return self.__device_type

    @property
    def config(self):
        """
        Gets the configuration used by this mapping
        """
        return self.__config

    @property
    def is_brain_connection(self):
        """
        Returns whether the the parameter is connected to the neuronal network

        :return: True, if the parameter is mapped to the neuronal network, otherwise False
        """
        return True

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        adapter = transfer_function_manager.brain_adapter
        assert isinstance(adapter, IBrainCommunicationAdapter)
        neurons = self.neurons.select(config.brain_root, adapter)
        return adapter.register_spike_sink(neurons,
                                           self.device_type,
                                           **self.config)

    def create_tf(self):
        """
        Creates a TF in case the TF specification has been omitted
        """
        return Neuron2Robot()


class MapSpikeSource(MapSpikeSink):
    """
    Class to map parameters to spike sources such as poisson generators
    """

    supported_device_types = [IPoissonSpikeGenerator, IFixedSpikeGenerator,
                              IDCSource, IACSource, INCSource, ISpikeInjector, IRawSignal]

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        adapter = transfer_function_manager.brain_adapter
        assert isinstance(adapter, IBrainCommunicationAdapter)
        neurons = self.neurons.select(config.brain_root, adapter)
        #print neurons
        return adapter.register_spike_source(neurons,
                                             self.device_type,
                                             **self.config)

    def is_supported(self, device_type):
        """
        Gets a value indicating whether the given device type is supported

        :param device_type: The device type that should be looked for
        """
        return device_type in MapSpikeSource.supported_device_types

    def create_tf(self):
        """
        Creates a TF in case the TF specification has been omitted
        """
        from hbp_nrp_cle.tf_framework._Robot2Neuron import Robot2Neuron
        return Robot2Neuron()


class Neuron2Robot(TransferFunction):
    """
    Class to represent a transfer function from neurons to robot
    """

    def __init__(self, robot_topic=None, triggers=None, throttling_rate=None):
        """
        Defines a new transfer function from robots to neurons

        :param robot_topic: the robot topic reference
        :param other_topics: other topics required by this transfer function
        """
        super(Neuron2Robot, self).__init__(triggers, throttling_rate)
        if robot_topic is not None:
            assert isinstance(robot_topic, (Topic, str))
        self.__main_topic = robot_topic

    @property
    def topic(self):  # -> Topic:
        """
        The main robot topic is the topic that the return value of the transfer function is
        connected to
        """
        return self.__main_topic

    @topic.setter
    def topic(self, robot_topic):  # -> None:
        """
        Sets the main robot topic

        :param robot_topic: The new main robot topic
        """
        self.__main_topic = robot_topic

    def __call__(self, func):  # -> Neuron2Robot:
        """
        Applies the transfer functions object to the given function

        :param func: The function body for this transfer function
        :return The transfer function object
        """

        self._init_function(func, config.active_node.n2r)
        return self

    def __repr__(self):  # pragma: no cover
        return "{0} transfers to robot {1} using {2}" \
            .format(self.name, self.__main_topic, self._params)

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time

        :param t: The simulation time
        """

        return_value = super(Neuron2Robot, self).run(t)

        if return_value is not None:
            try:
                topic_publisher = self.__main_topic
                if topic_publisher is not None:
                    topic_publisher.send_message(return_value)
            # pylint: disable=broad-except
            except Exception, e:
                self._handle_error(e, sys.exc_info()[2])

    def unregister(self):
        """
        Unregister the main robot topic publisher, this TF will no longer
        be able to publish messages to the robot.
        """
        topic_publisher = self.__main_topic
        if topic_publisher is not None:
            self.__main_topic = None
            if hasattr(topic_publisher, "unregister"):
                topic_publisher.unregister()
