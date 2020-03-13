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
Tests the ROS communication adapter
'''

import unittest
import rospy
import rosservice
from mock import patch, Mock
from hbp_nrp_cle.robotsim.RobotInterface import PreprocessedTopic, Topic, Service
from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter, RosPublishedPreprocessedTopic, \
    RosPublishedTopic, RosSubscribedPreprocessedTopic, RosSubscribedTopic, RosServiceProxy
from hbp_nrp_cle.tests.mocks.robotsim.test_mock_ros_communicationadapter import MockRosServiceProxy
import hbp_nrp_cle.robotsim.RosCommunicationAdapter as tested_module
from testfixtures import LogCapture
from rosgraph_msgs.msg import Log


__author__ = 'LarsPfotzer'


class TestRosCommunicationAdapter(unittest.TestCase):
    """
    Tests the ROS communication adapter module
    """

    def setUp(self):
        p = patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.master')
        master_mock = p.start()
        master_mock.Master().getTopicTypes.return_value = [
            ['/rosout', 'rosgraph_msgs/Log'],
            ['/err', 'rosgraph_msgs/not_existing'],
            ['/foo', 'something/not_existing'],
            ['/bar', 'something/too/long']
        ]
        self.rca = RosCommunicationAdapter()
        self.addCleanup(p.stop)

    def test_get_topic_type(self):
        self.assertEqual(self.rca.get_topic_type('/rosout'), Log)
        with self.assertRaises(Exception):
            self.rca.get_topic_type("/err")
        with self.assertRaises(Exception):
            self.rca.get_topic_type("/foo")
        with self.assertRaises(Exception):
            self.rca.get_topic_type("/bar")
        self.assertIsNone(self.rca.get_topic_type("/does_not_exist"))

    # Tests for ROSCommunicationAdapter
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_initialize(self, _, mock_init_node):
        with LogCapture('hbp_nrp_cle.robotsim.RosCommunicationAdapter') as logcapture:
            self.rca.initialize("test_node")
            self.assertEquals(mock_init_node.call_count, 1)

            mock_init_node.side_effect = rospy.exceptions.ROSException
            self.rca.initialize("test_node")
            self.assertEquals(mock_init_node.call_count, 2)
            logcapture.check(('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO',
                              'Robot communication adapter initialized'),
                             ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'WARNING',
                              'ROS node already initialized with another name')
            )

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_create_topic_publisher(self, _, mock_rospy_publisher, mock_init_node):
        self.rca.initialize("test_node")

        r = self.rca.create_topic_publisher(PreprocessedTopic(
            'preprocessed_topic', 'topic_type', lambda x: ()
        ))
        self.assertIsInstance(r, RosPublishedPreprocessedTopic)

        r = self.rca.create_topic_publisher(Topic('topic', 'topic_type'))
        self.assertIsInstance(r, RosPublishedTopic)

        r = self.rca.create_topic_publisher("/rosout")
        self.assertIsInstance(r, RosPublishedTopic)

        with self.assertRaises(Exception):
            self.rca.create_topic_publisher("/does_not_exist")

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertGreater(mock_rospy_publisher.call_count, 0)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_create_topic_subscriber(self, mock_rospy_subscriber, mock_init_node):
        self.rca.initialize("test_node")

        r = self.rca.create_topic_subscriber(PreprocessedTopic(
            'preprocessed_topic', 'topic_type', lambda x: ()
        ))
        self.assertIsInstance(r, RosSubscribedPreprocessedTopic)

        r = self.rca.create_topic_subscriber(Topic('topic', 'topic_type'))
        self.assertIsInstance(r, RosSubscribedTopic)

        r = self.rca.create_topic_subscriber('/rosout')
        self.assertIsInstance(r, RosSubscribedTopic)

        with self.assertRaises(Exception):
            self.rca.create_topic_subscriber('/does_not_exist')

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertGreater(mock_rospy_subscriber.call_count, 0)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_create_topic_subscriber_triggers_tf(self, mock_rospy_subscriber, mock_init_node):
        self.rca.initialize("test_node")
        tf = Mock()
        tf.active = True
        tf.elapsed_time = 0
        tf.should_run.return_value = True
        tested_module.sim_time = 42.0

        sub = self.rca.create_topic_subscriber(Topic('topic', 'topic_type'))
        self.assertIsInstance(sub, RosSubscribedTopic)

        sub.register_tf_trigger(tf)
        sub._callback("foo")

        self.assertEqual(sub.value, "foo")
        tf.should_run.assert_called_once_with(42.0)
        tf.run.assert_called_once_with(42.0)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.is_shutdown')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_is_alive(self, _, mock_init_node, mock_is_shutdown):
        self.rca.initialize("test_node")
        mock_is_shutdown.return_value = False
        self.assertTrue(self.rca.is_alive)
        mock_is_shutdown.return_value = True
        self.assertFalse(self.rca.is_alive)

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertEqual(mock_is_shutdown.call_count, 2)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_refresh_buffers(self, _, mock_init_node, mock_publisher):
        t1, t2 = Topic('a', 'b'), Topic('c', 'd')
        self.rca.register_publish_topic(t1)
        self.rca.register_publish_topic(t2)
        self.rca.initialize("test_node")
        self.rca.refresh_buffers(0.1)

        for key in self.rca.subscribed_topics:
            self.assertTrue(self.rca.subscribed_topics[key].changed)

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertEquals(mock_publisher.call_count, 2)


class TestTopicImplementations(unittest.TestCase):

    # Tests for RosPublishedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rpt_init(self, mock_rospy_publisher):
        RosPublishedTopic(Topic('topic_name', 'topic_type'))
        self.assertEquals(mock_rospy_publisher.call_count, 1)
        self.assertEquals(mock_rospy_publisher.call_args_list[0][0][0], 'topic_name')
        self.assertEquals(mock_rospy_publisher.call_args_list[0][0][1], 'topic_type')
        self.assertRaises(AssertionError, RosPublishedTopic, None)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rpt_send_message(self, mock_rospy_publisher):
        rpt = RosPublishedTopic(Topic('topic_name', 'topic_type'))
        pub = mock_rospy_publisher.return_value
        rpt.send_message('message')
        self.assertEquals(pub.publish.call_count, 1)
        self.assertEquals(pub.publish.call_args_list[0][0][0], 'message')
        rpt._unregister()
        rpt.send_message('message')
        self.assertEquals(pub.publish.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rpt_unregister(self, mock_rospy_publisher):
        rpt = RosPublishedTopic(Topic('topic_name', 'topic_type'))
        pub = mock_rospy_publisher.return_value
        rpt._unregister()
        self.assertEquals(pub.unregister.call_count, 1)

    # Tests for RosPublishedPreprocessedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rppt_init(self, mock_rospy_publisher):
        self.assertRaises(AssertionError, RosPublishedPreprocessedTopic, None)
        t = PreprocessedTopic('a', 'b', lambda x: ())
        rppt = RosPublishedPreprocessedTopic(t)
        self.assertEquals(t.pre_processor, rppt._RosPublishedPreprocessedTopic__pre_processor)
        self.assertEquals(mock_rospy_publisher.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rppt_send_message(self, mock_rospy_publisher):
        with LogCapture('hbp_nrp_cle.robotsim.RosCommunicationAdapter') as logcapture:
            rppt = RosPublishedPreprocessedTopic(PreprocessedTopic('a', 'b', lambda x: x))
            oldp = rppt._RosPublishedTopic__pub
            rppt._RosPublishedTopic__pub = None
            rppt.send_message('message')
            rppt._RosPublishedTopic__pub = oldp
            rppt.send_message('message')
            logcapture.check(('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO',
                              'ROS publisher created: topic name = a, topic type = b'),
                             ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'ERROR',
                              'Trying to publish messages on an unregistered topic'),
                             ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'DEBUG',
                              'ROS message published: topic value = message'))
            self.assertEquals(mock_rospy_publisher.call_count, 1)

    # Tests for RosSubscribedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_init(self, mock_rospy_subscriber):
        RosSubscribedTopic(Topic('topic_name', 'topic_type'), None)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)
        self.assertEquals(mock_rospy_subscriber.call_args_list[0][0][0], 'topic_name')
        self.assertEquals(mock_rospy_subscriber.call_args_list[0][0][1], 'topic_type')
        self.assertRaises(AssertionError, RosSubscribedTopic, None, None)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_callback(self, mock_rospy_subscriber):
        rst = RosSubscribedTopic(Topic('topic_name', 'topic_type'), 10)
        rst._callback('data')
        self.assertTrue(rst.changed)
        self.assertEquals(rst.value, 'data')
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_reset_changed(self, mock_rospy_subscriber):
        rst = RosSubscribedTopic(Topic('topic_name', 'topic_type'), None)
        rst.reset_changed()
        self.assertFalse(rst.changed)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_reset(self, mock_rospy_subscriber):
        rst = RosSubscribedTopic(Topic('topic_name', 'topic_type'), None)
        res = rst.reset(None)
        self.assertFalse(rst.changed)
        self.assertFalse(res.changed)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    # Tests for RosSubscribedPreprocessedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rspt_init(self, mock_rospy_subscriber):
        self.assertRaises(AssertionError, RosSubscribedPreprocessedTopic, None, None)
        t = PreprocessedTopic('a', 'b', lambda x: ())
        rppt = RosSubscribedPreprocessedTopic(t, None)
        self.assertEquals(t.pre_processor, rppt._RosSubscribedPreprocessedTopic__pre_processor)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rspt_callback(self, mock_rospy_subscriber):
        rst = RosSubscribedPreprocessedTopic(
            PreprocessedTopic('topic_name', 'topic_type', lambda x: x), None)
        rst._callback('data')
        self.assertTrue(rst.changed)
        self.assertEquals(rst.value, 'data')
        self.assertEquals(mock_rospy_subscriber.call_count, 1)


class TestServiceImplementation(unittest.TestCase):

    # Tests for RosServiceProxy
    @patch('rospy.ServiceProxy')
    @patch.object(rosservice, 'get_service_list', return_value=['service_name'])
    def test_rsp_init(self, mock_service_list, mock_rospy_service):
        RosServiceProxy(Service('service_name', 'service_type'))
        self.assertEquals(mock_service_list.call_count, 1)

        self.assertEquals(mock_rospy_service.call_count, 1)
        self.assertEquals(mock_rospy_service.call_args_list[0][0][0], 'service_name')
        self.assertEquals(mock_rospy_service.call_args_list[0][0][1], 'service_type')
        self.assertRaises(AssertionError, RosServiceProxy, None, None)

    @patch('rospy.ServiceProxy')
    @patch.object(rosservice, 'get_service_list', return_value=['service_name'])
    def test_rsp_send_request(self, mock_service_list, mock_rospy_service):
        rsp = MockRosServiceProxy(Service('service_name', 'service_type'), mock_service=mock_rospy_service)

        rsp.set_request_args('args', kwargs='kwargs')
        rsp.update_value()
        mock_rospy_service.call.assert_called_with('args', kwargs='kwargs')

    @patch('rospy.ServiceProxy')
    @patch.object(rosservice, 'get_service_list', return_value=['service_name'])
    def test_rsp_reset(self, mock_service_list, mock_rospy_service):
        rsp = MockRosServiceProxy(Service('service_name', 'service_type'), initial_response=True,
                                  mock_service=mock_rospy_service)

        rsp.reset(None)
        self.assertEquals(rsp.value, None)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.ServiceProxy')
    @patch.object(rosservice, 'get_service_list', return_value=['service_name'])
    def test_rsp_unregister(self, mock_service_list, mock_rospy_service):
        rsp = MockRosServiceProxy(Service('service_name', 'service_type'), mock_service=mock_rospy_service)
        rsp._unregister()
        self.assertEquals(rsp.service_proxy, None)


if __name__ == '__main__':
    unittest.main()
