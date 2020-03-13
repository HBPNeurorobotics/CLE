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
This module represents the interfaces for the robot (world) simulation both in terms of data
exchange and control of the simulation
"""

__author__ = 'GeorgHinkel'

# pylint: disable=W0613


class Service(object):
    """
    Represents a reference to a robot service
    """

    def __init__(self, name, service_type):  # -> None:
        """
        Create a new robot type reference

        :param name: the name of the service
        :param service_type: the type of the service
        """
        self.__name = name
        self.__type = service_type

    @property
    def name(self):  # -> str:
        """
        Gets the name of the service
        """
        return self.__name

    @property
    def service_type(self):  # -> type:
        """
        Gets the type of the service
        """
        return self.__type

    def __repr__(self):  # pragma: no cover
        return self.__name + " : " + self.__type.__name__


class Topic(object):
    """
    Represents a reference to a robot topic
    """

    def __init__(self, name, topic_type):  # -> None:
        """
        Create a new robot type reference

        :param name: the name of the topic
        :param topic_type: the type of the topic
        """
        self.__name = name
        self.__type = topic_type

    @property
    def name(self):  # -> str:
        """
        Gets the name of the topic
        """
        return self.__name

    @property
    def topic_type(self):  # -> type:
        """
        Gets the type of the topic
        """
        return self.__type

    def __repr__(self):  # pragma: no cover
        return self.__name + " : " + self.__type.__name__


class PreprocessedTopic(Topic):
    """
    Represents a topic where pre-processing is applied to
    """

    def __init__(self, name, topic_type, pre_processing):
        """
        Creates a new pre-processing topic

        :param name: The name of the topic
        :param topic_type: The topic type
        :param pre_processing: The pre-processing function
        """
        super(PreprocessedTopic, self).__init__(name, topic_type)
        self.__pre_processing = pre_processing

    @property
    def pre_processor(self):  # -> func
        """
        Gets the pre_processor
        """
        return self.__pre_processing

    def _unregister(self):
        """
        INTERNAL USE ONLY: this should never be directly invoked by a user.

        Unregister the Topic. After this call, nobody can publish or receive messages anymore.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotPublishedTopic(object):  # pragma: no cover
    """
    Represents a communication object for a published robot topic
    """

    def send_message(self, value):  # -> None:
        """
        Send a message to the robot topic represented by this instance

        :param value: The message to be sent to the robot
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self, transfer_function_manager):
        """
        Resets the published topic

        :param transfer_function_manager: The transfer function manager the publisher belongs to
        :return: The reset adapter
        """
        return self

    def _unregister(self):
        """
        INTERNAL USE ONLY: this should never be directly invoked by a user.

        Unregister the Topic. After this call, nobody can publish.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotSubscribedTopic(object):  # pragma: no cover
    """
    Represents a communication object for a subscribed robot topic
    """

    @property
    def changed(self):  # -> bool:
        """
        Gets a value indicating whether the value of the subscribed topic has changed since the last
        time step
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def value(self):  # -> object:
        """
        Gets the current value of the subscribed topic
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self, transfer_function_manager):
        """
        Resets the subscribed topic

        :param transfer_function_manager: The transfer function manager the subscriber belongs to
        :return: The reset adapter
        """
        return self

    def _unregister(self):
        """
        INTERNAL USE ONLY: this should never be directly invoked by a user.

        Unregister the Topic. After this call, nobody can receive messages anymore.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotServiceProxy(object):  # pragma: no cover
    """
    Represents a communication object for a service proxy
    """

    def set_request_args(self, *args, **kwargs):
        """
        Set the request call arguments
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def update_value(self):
        """
        Call service and update response value
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def value(self):  # -> object:
        """
        Gets the current value of the service response
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self, transfer_function_manager):
        """
        Resets the service proxy

        :param transfer_function_manager: The transfer function manager the service proxy belongs to
        :return: The reset adapter
        """
        return self

    def _unregister(self):
        """
        INTERNAL USE ONLY: this should never be directly invoked by a user.

        Unregister the Service. After this call, nobody can use the service proxy anymore.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotCommunicationAdapter(object):  # pragma: no cover
    """
    Represents the communication adapter to the robot
    """

    def __init__(self):  # -> None:
        self.__published_topics = []
        self.__subscribed_topics = []

        self.__service_proxies = []

    @property
    def published_topics(self):  # -> list:
        """
        Gets the published topics for the robot communication adapter

        :return: A hash table of the communication adapters published topics
        """
        return self.__published_topics

    @property
    def subscribed_topics(self):  # -> list:
        """
        Gets the subscribed topics for the robot communication adapter

        :return: A hash table of the communication adapters subscribed topics
        """
        return self.__subscribed_topics

    @property
    def service_proxies(self):  # -> list:
        """
        Gets the service proxies for the robot communication adapter

        :return: A hash table of the communication adapters service responses
        """
        return self.__service_proxies

    def register_subscribe_topic(self, topic, **kwargs):  # -> IRobotSubscribedTopic:
        """
        Requests a subscription object for the given topic

        :param topic: The topic that should be subscribed
        :param kwargs: Additional configuration parameters
        :return: A subscription object that holds the current data
        """

        subscriber = self.create_topic_subscriber(topic, **kwargs)
        self.__subscribed_topics.append(subscriber)
        return subscriber

    def unregister_subscribe_topic(self, topic):
        """
        Unregisters and removes the given subscriber topic object.

        :param topic The IRobotSubscribedTopic to unregister.
        """
        topic._unregister()  # pylint: disable=protected-access
        if topic in self.__subscribed_topics:
            self.__subscribed_topics.remove(topic)

    def register_publish_topic(self, topic, **kwargs):  # -> IRobotPublishedTopic:
        """
        Requests a publisher object for the given topic

        :param topic: The topic for which to create a publisher
        :param kwargs: Additional configuration parameters
        :return: A publisher communication object
        """
        publisher = self.create_topic_publisher(topic, **kwargs)
        self.__published_topics.append(publisher)
        return publisher

    def unregister_publish_topic(self, topic):
        """
        Unregisters and removes the given publisher topic object.

        :param topic The IRobotPublishedTopic to unregister.
        """
        topic._unregister() # pylint: disable=protected-access
        if topic in self.__published_topics:
            self.__published_topics.remove(topic)

    def register_service_proxy(self, service, **kwargs):  # -> IRobotServiceProxy:
        """
        Requests a proxy object for the given service

        :param service: The service to which a proxy should be established
        :param kwargs: Additional configuration parameters
        :return: A responder object that holds the current data
        """

        service_proxy = self.create_service_proxy(service, **kwargs)
        self.__service_proxies.append(service_proxy)
        return service_proxy

    def unregister_service_proxies(self, service_proxy):
        """
        Unregisters and removes the given service proxy object.

        :param service_proxy The IRobotServiceProxy to unregister.
        """
        service_proxy._unregister()  # pylint: disable=protected-access
        if service_proxy in self.__service_proxies:
            self.__service_proxies.remove(service_proxy)

    def create_topic_subscriber(self, topic, **config):  # -> IRobotSubscribedTopic:
        """
        Creates the subscription object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :return: A subscription object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def create_topic_publisher(self, topic, **config):  # -> IRobotPublishedTopic:
        """
        Creates a publisher object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the publisher
        :return: A publisher object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def create_service_proxy(self, service, **config):  # -> IRobotServiceProxy:
        """
        Creates a service proxy object for the given service

        :param service: The Service
        :param config: Additional configuration for the service proxy
        :return: A service proxy object
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self, name):  # -> None:
        """
        Initializes the robot adapter

        :param name: The name of the node
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def refresh_buffers(self, t):  # -> None:
        """
        Refreshes the subscribed topic buffers for the given simulation time

        :param t: The simulation time
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def shutdown(self):
        """
        Closes any connections created by the adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")


class IRobotControlAdapter(object):  # pragma: no cover
    """
    Represents a control adapter for the world simulation
    """

    @property
    def is_alive(self):  # -> bool:
        """
        Queries the current status of the world simulation

        :return: True, if the world simulation is alive, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def is_paused(self):  # -> bool:
        """
        Queries the current status of the physics simulation

        :return: True, if the physics simulation is paused, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def initialize(self):  # -> None:
        """
        Initializes the world simulation control adapter
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def run_step(self, dt):  # -> float64:
        """
        Runs the world simulation for the given CLE time step in seconds

        :param dt: The CLE time step in seconds
        :return: Updated simulation time, otherwise -1
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def run_step_async(self, dt):  # -> Future:
        """
        Runs the world simulation for the given CLE time step in seconds in an asynchronous manner.

        :param dt: The CLE time step in seconds
        :return: a Future for the result or potential exceptions of the execution
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def shutdown(self):  # -> None:
        """
        Shuts down the world simulation
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    @property
    def time_step(self):  # -> float64:
        """
        Gets the physics simulation time step in seconds

        :param dt: The physics simulation time step in seconds
        :return: The physics simulation time step in seconds
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def set_time_step(self, time_step):  # -> Bool:
        """
        Sets the physics simulation time step in seconds

        :param dt: The physics simulation time step in seconds
        :return: True, if the physics simulation time step is updated, otherwise False
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def set_robots(self, robots):
        """
        Sets the list of robots
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset(self):  # -> None:
        """
        Resets the physics simulation
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")

    def reset_world(self, models, lights):  # -> None:
        """
        Resets the world excluding the robot
        :param models: A dictionary containing pairs model_name: model_sdf.
        :param lights: A dictionary containing pairs light_name: light sdf.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
