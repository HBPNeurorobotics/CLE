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
The mock implementation of the communication adapter to the world simulation
"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter, IRobotPublishedTopic, \
    IRobotSubscribedTopic
from hbp_nrp_cle.tests.mocks.robotsim.test_mock_ros_communicationadapter import MockRosServiceProxy

__author__ = 'GeorgHinkel'


class Roscore(object):
    """
    Represents a mock implementation of the message passing in roscore
    """
    def __init__(self):
        """
        Creates a new mock message passing
        """
        self._associations = []

    def connect(self, pub, sub):
        """
        Connect a publisher to a subscriber"

        :param pub: An IRobotPublishedTopic instance
        :param sub: An IRobotSubscribedTopic instance
        """
        self._associations.append((pub, sub))

    def sendAll(self):
        """
        Send all present messages to the connected subscribers
        """
        for association in self._associations:
            data = association[0].sent
            for d in data:
                association[1].value = d


class MockRobotCommunicationAdapter(IRobotCommunicationAdapter):
    """
    Represents a mock implementation of the world simulation communication adapter
    """
    def __init__(self):
        """
        Creates a new mock robot adapter
        """
        super(MockRobotCommunicationAdapter, self).__init__()
        self.__name = None

    def initialize(self, name):
        """
        Initializes the mock with the given name

        :param name: The name for the mock
        """
        self.__name = name

    @property
    def name(self):
        """
        Gets the name of the mock
        """
        return self.__name

    def create_topic_publisher(self, topic, **config):
        """
        Creates a publisher object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the publisher
        :return: A publisher object
        """
        return MockPublishedTopic()

    def create_topic_subscriber(self, topic, **config):
        """
        Creates the subscription object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :return: A subscription object
        """
        return MockSubscribedTopic()

    def create_service_proxy(self, service, **config):
        """
        Creates the subscription object for the given service

        :param service: The service
        :param config: Additional configuration for the ServiceProxy
        :return: A service proxy object
        """
        return MockRosServiceProxy()

    def refresh_buffers(self, t):
        """
        Refreshes the subscribed topic buffers for the given simulation time

        :param t: The simulation time
        """
        pass

    def shutdown(self):
        """
        Closes any connections created by the adapter
        """
        pass


class MockPublishedTopic(IRobotPublishedTopic):
    """
    Represents a mock for a robot topic publisher
    """
    def __init__(self):
        """
        Initializes the new mock
        """
        self.__sent = []

    def send_message(self, value):
        """
        Send a message to the robot topic represented by this instance

        :param value: The message to be sent to the robot
        """
        self.__sent.append(value)

    @property
    def sent(self):
        """
        Gets a list of the messages sent to the mocked publisher
        """
        return self.__sent

    def _unregister(self):
        """
        Unregister the Topic. Meaningless for mocks
        """
        pass


class MockSubscribedTopic(IRobotSubscribedTopic):
    """
    Represents a mock for a robot topic subscriber
    """

    def __init__(self):
        """
        Created a new mock
        """
        self.__changed = False
        self.__value = None
        self.__tfs = []

    def register_tf_trigger(self, tf):
        """
        Registers to trigger the provided TF in case a new value appears

        :param tf: The transfer function
        """
        if not tf in self.__tfs:
            self.__tfs.append(tf)

    @property
    def changed(self):
        """
        Gets a value indicating whether the value of the subscribed topic has changed since the last
        time step
        """
        return self.__changed

    @property
    def value(self):
        """
        Gets the current value of the subscribed topic
        """
        return self.__value

    @value.setter
    def value(self, value):  # pylint: disable=W0221
        """
        Sets the current value of the subscribed topic

        :param value: The new current value
        """
        self.__changed = (value != self.__value)
        self.__value = value
        for tf in self.__tfs:
            if tf.active and tf.should_run(42.0):
                tf.run(42.0)

    def _unregister(self):
        """
        Unregister the Topic. Meaningless for mocks
        """
        pass
