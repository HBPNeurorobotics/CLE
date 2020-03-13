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
Represents an implementation of the robot communication adapter actually
using ROS
"""

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter, \
    Topic, PreprocessedTopic, IRobotSubscribedTopic, IRobotPublishedTopic, \
    Service, IRobotServiceProxy
from hbp_nrp_cle.tf_framework._TransferFunctionManager import TransferFunctionManager
import rosgraph_msgs.msg
import rospy
import rosservice
import logging
import rosgraph.masterapi as master

logger = logging.getLogger(__name__)

__author__ = 'GeorgHinkel'


sim_time = 0.0
# Receive buffer size for subscriber topics (in bytes). Same as ROS default.
DEFAULT_SUB_BUFFER_SIZE = 65536
# Queue size for subscriber topics.
# ROS docs suggest either 1 or None, the latter denoting an infinite queue.
DEFAULT_SUB_QUEUE_SIZE = 1

# Queue size for publisher topics.
DEFAULT_PUB_QUEUE_SIZE = 10


class RosCommunicationAdapter(IRobotCommunicationAdapter):
    """
    Represents a robot communication adapter actually using ROS
    """

    def get_topic_type(self, topic_name, update_when_miss=True):
        """
        Gets the type for the given ROS topic

        :param topic_name: The name of the topic
        :param update_when_miss: If set, the function will try to update the cached topic
        types if the topic cannot be found
        """
        for name, type_str in self.__topic_types:
            if name == topic_name:
                t = "?"
                mod = "?"
                try:
                    mod, t = type_str.split('/')
                    return getattr(__import__(mod).msg, t)
                except AttributeError:
                    raise Exception("The type {0} is not present in package {1}".format(t, mod))
                except ValueError:
                    raise Exception("The format of the topic type {0} is not supported"
                                    .format(type_str))
        if update_when_miss:
            self.__refresh_topic_types()
            return self.get_topic_type(topic_name, False)
        return None

    def __init__(self):
        """
        Create a new RosCommunicationAdapter
        """
        IRobotCommunicationAdapter.__init__(self)
        self.__topic_types = []
        self.__refresh_topic_types()
        self.__clock_listener = None

    def __refresh_topic_types(self):
        """
        Updates the topic types
        """
        m = master.Master('masterapi')
        self.__topic_types = m.getTopicTypes()

    def initialize(self, name):
        """
        Initializes this robot communication adapter

        :param name: The name of this node
        """
        try:
            rospy.init_node(name)
            logger.info("Robot communication adapter initialized")
        except rospy.exceptions.ROSException:
            logger.warn("ROS node already initialized with another name")
        self.__clock_listener = rospy.Subscriber("/clock", rosgraph_msgs.msg.Clock,
                                                 self.__update_clock)

    @staticmethod
    def __update_clock(data):
        """
        Updates the clock directly from the simulation time

        Using rospy.get_time() does not work, as it uses the wall-time even when use_sim_time is set

        :param data: The clock message
        """
        # pylint: disable=global-statement
        global sim_time
        sim_time = data.clock.secs + data.clock.nsecs / 1.0e9

    def create_topic_publisher(self, topic, **config):
        """
        Creates a publisher object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the publisher
        :param **queue_size: ROS Publisher queue_size parameter, please refer to ROS documentation

        :return: A publisher object
        """
        if isinstance(topic, PreprocessedTopic):
            return RosPublishedPreprocessedTopic(topic, **config)
        elif isinstance(topic, str):
            topic_type = self.get_topic_type(topic)
            if topic_type is None:
                raise Exception("The type of topic {0} is unknown. Please specify it explicitly"
                                .format(topic))
            topic = Topic(topic, topic_type)
        return RosPublishedTopic(topic, **config)

    def create_topic_subscriber(self, topic, **config):
        """
        Creates the subscription object for the given topic

        :param topic: The topic
        :param config: Additional configuration for the subscriber
        :param **queue_size: ROS Subscriber queue_size parameter, please refer to ROS documentation
        :param **buff_size: ROS Subscriber buff_size parameter, please refer to ROS documentation

        :return: A subscription object
        """
        if isinstance(topic, PreprocessedTopic):
            return RosSubscribedPreprocessedTopic(topic,
                                                  config.get('initial_value', None),
                                                  **config)
        elif isinstance(topic, str):
            topic_type = self.get_topic_type(topic)
            if topic_type is None:
                raise Exception("The type of topic {0} is unknown. Please specify it explicitly"
                                .format(topic))
            topic = Topic(topic, topic_type)
        return RosSubscribedTopic(topic,
                                  config.get('initial_value', None),
                                  **config)

    def create_service_proxy(self, service, **config):
        """
        Creates a service proxy object for the given service

        :param service: The service
        :param config: Additional configuration for the service proxy

        :return: A service proxy object
        """
        return RosServiceProxy(service, **config)

    @property
    def is_alive(self):  # pylint: disable=R0201
        """
        Gets a value indicating whether the robot simulation is still alive
        """
        return not rospy.is_shutdown()

    def refresh_buffers(self, t):
        """
        Resets the changed bit for all subscribers

        :param t: The world simulation time in milliseconds
        """
        for subscriber in self.subscribed_topics:
            subscriber.reset_changed()

    def shutdown(self):
        """
        Closes any connections created by the adapter
        """
        for publisher in self.published_topics:
            publisher._unregister()  # pylint: disable=protected-access
        for subscriber in self.subscribed_topics:
            subscriber._unregister()  # pylint: disable=protected-access


class RosPublishedTopic(IRobotPublishedTopic):
    """
    Represents a robot topic publisher actually using ROS
    """
    def __init__(self, topic, **config):
        """
        Creates a new robot topic publisher

        :param topic: The topic where data should be sent to
        :param config: Additional configuration for the publisher
        :param **queue_size: ROS Publisher queue_size parameter, please refer to ROS documentation
        """
        self.__lastSent = None
        assert isinstance(topic, Topic)
        logger.info("ROS publisher created: topic name = %s, topic type = %s",
                    topic.name, topic.topic_type)
        self.__pub = rospy.Publisher(topic.name, topic.topic_type,
                                     queue_size=config.get('queue_size', DEFAULT_PUB_QUEUE_SIZE))

    def send_message(self, value):
        """
        Sends a message

        :param value: The message to be sent (the type must match the topic type)
        """
        # if value != self.__lastSent:
        if self.__pub is not None:
            self.__pub.publish(value)
            self.__lastSent = value
            logger.debug("ROS message published: topic value = %s",
                         value)
        else:
            logger.error("Trying to publish messages on an unregistered topic")

    def _unregister(self):
        """
        Unregister the Topic. After this call, nobody can publish
        anymore.
        """
        if self.__pub:
            self.__pub.unregister()
            self.__pub = None


class RosPublishedPreprocessedTopic(RosPublishedTopic):
    """
    Represents a robot topic publisher actually using ROS
    """
    def __init__(self, topic, **config):
        """
        Creates a new robot topic publisher

        :param topic: The topic where data should be sent to
        :param config: Additional configuration for the subscriber
        """
        super(RosPublishedPreprocessedTopic, self).__init__(topic, **config)
        assert isinstance(topic, PreprocessedTopic)
        self.__pre_processor = topic.pre_processor

    def send_message(self, value):
        """
        Sends a message
        :param value: The message to be sent
        """
        to_send = self.__pre_processor(value)
        super(RosPublishedPreprocessedTopic, self).send_message(to_send)


class RosSubscribedTopic(IRobotSubscribedTopic):
    """
    Represents a robot topic subscriber actually using ROS
    """

    def __init__(self, topic, initial_value, **config):
        """
        Initializes a new subscriber for the given topic

        :param topic: The topic that is subscribed
        :param initial_value: The initial value for the subscriber
        :param config: Additional configuration for the subscriber
        :param **queue_size: ROS Subscriber queue_size parameter, please refer to ROS documentation
        :param **buff_size: ROS Subscriber buff_size parameter, please refer to ROS documentation
        """
        self.__changed = False
        self.__value = initial_value
        self.__tfs = []
        assert isinstance(topic, Topic)
        self.__subscriber = rospy.Subscriber(topic.name, topic.topic_type,
                                             self._callback,
                                             queue_size=config.get('queue_size',
                                                                   DEFAULT_SUB_QUEUE_SIZE),
                                             buff_size=config.get('buff_size',
                                                                  DEFAULT_SUB_BUFFER_SIZE))
        logger.info("ROS subscriber created: topic name = %s, topic type = %s",
                    topic.name, topic.topic_type)

    def register_tf_trigger(self, tf):
        """
        Registers to trigger the provided TF in case a new value appears

        :param tf: The transfer function
        """
        if tf not in self.__tfs:
            self.__tfs.append(tf)

    def _callback(self, data):
        """
        This method is called whenever new data is available from ROS

        :param data: The incoming data on this topic
        """
        logger.debug("ROS subscriber callback")
        self.__changed = True
        self.__value = data
        t = sim_time
        for tf in self.__tfs:
            TransferFunctionManager.run_tf(tf, t)

    @property
    def changed(self):
        """
        Indicates whether the current value of this subscriber has changed
        since the last iteration
        """
        return self.__changed

    def reset_changed(self):
        """
        Resets the changed status of the current subscriber
        """
        self.__changed = False

    @property
    def value(self):
        """
        Gets the last value received by this ROS subscribed topic
        """
        return self.__value

    def reset(self, transfer_function_manager):
        """
        Gets a reset subscriber

        :param transfer_function_manager: The transfer function manager in which the subscribed
         topic is contained
        """
        self.reset_changed()
        self.__value = None
        return self

    def _unregister(self):
        """
        Detaches the subscribed topic from ROS
        """
        if self.__subscriber:
            self.__subscriber.unregister()
            self.__subscriber = None


class RosSubscribedPreprocessedTopic(RosSubscribedTopic):
    """
    Represents a robot topic subscriber with a preprocessor
    """

    def __init__(self, topic, initial_value, **config):
        """
        Creates a new preprocessing topic subscriber

        :param topic: The topic that is subscribed
        :param initial_value: The initial value for the subscriber
        :param config: Additional configuration for the subscriber

        """
        super(RosSubscribedPreprocessedTopic, self).__init__(topic, initial_value, **config)

        assert isinstance(topic, PreprocessedTopic)
        self.__pre_processor = topic.pre_processor

    def _callback(self, data):
        """
        This method is called whenever new data is available from ROS

        :param data: The incoming data
        """
        pre_processed = self.__pre_processor(data)
        super(RosSubscribedPreprocessedTopic, self)._callback(pre_processed)


class RosServiceProxy(IRobotServiceProxy):
    """
    Represents a robot service proxy actually using ROS
    """
    def __init__(self, service, initial_response=None, persistent=False, headers=None):
        """
        Initializes a new service proxy for the given service

        :param service: The service that is requested
        :param initial_response: The initial value of the service proxy response
        :param persistent: Persistence of Service.
        Look at rospy.ServiceProxy documentation for details
        :param headers: Service headers.
        Look at rospy.ServiceProxy documentation for details
        """
        self.__response_value = initial_response
        self.__request_args = []
        self.__request_kwargs = {}

        assert isinstance(service, Service)

        services = rosservice.get_service_list()
        if service.name not in services:
            raise Exception("Service {} not available", service.name)

        self.__service_proxy = rospy.ServiceProxy(service.name, service.service_type,
                                                  persistent=persistent, headers=headers)
        logger.info("ROS service responder created: service name = %s, service type = %s",
                    service.name, service.service_type)

    def __call__(self, *args, **kwargs):
        """Calling the RosServiceProxy returns the service response"""
        if args or kwargs:
            self.set_request_args(*args, **kwargs)

        return self.update_value()

    def set_request_args(self, *args, **kwargs):
        """
        Update service proxy request arguments
        """
        self.__request_args = args
        self.__request_kwargs = kwargs

    def update_value(self, *args, **kwargs):
        """
        Update service response value
        :return Returns Service Response
        """
        if args or kwargs:
            self.set_request_args(*args, **kwargs)

        if self.__service_proxy is not None:
            self.__response_value = \
                self.__service_proxy.call(*self.__request_args, **self.__request_kwargs)
            return self.__response_value
        else:
            logger.error("Trying to send request on an unavailable service")

    @property
    def value(self):
        """
        Gets the last value received by this ROS subscribed service
        """
        return self.__response_value

    def reset(self, transfer_function_manager):
        """
        Gets a reset subscriber

        :param transfer_function_manager: The transfer function manager in which the subscribed
         service is contained
        """
        self.__response_value = None
        return self

    def _unregister(self):
        """
        Detaches the service
        """
        if self.__service_proxy is not None:
            self.__service_proxy.close()
            self.__service_proxy = None
