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
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter
import unittest

__author__ = 'GeorgHinkel'


class Neuron2RobotTests(unittest.TestCase):
    def test_parameter_not_parsable_fails(self):
        nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because there is no mapping for parameter "neuron1"

        @nrp.Neuron2Robot(Husky.RightArm.pose)
        def right_arm(t, neuronA):
            return neuronA.voltage

        self.init_adapters()

        self.assertRaises(Exception, nrp.initialize)

    def test_map_neuron_wrong_parameter_fails(self):
        nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because "neuronX" cannot be mapped to a parameter
        with self.assertRaises(Exception):
            @nrp.MapSpikeSink("neuronX", [1, 2, 3], nrp.leaky_integrator_exp)
            @nrp.Neuron2Robot(Husky.RightArm.pose)
            def right_arm(t, neuron1):
                return neuron1.voltage

    def test_no_tf_decorator_works(self):
        nrp.start_new_tf_manager()

        @nrp.MapSpikeSink("neuron", [1, 2, 3], nrp.leaky_integrator_alpha)
        def tf_sink_without_decorator(t, neuron):
            pass

        self.assertIsInstance(tf_sink_without_decorator, nrp.Neuron2Robot)

        @nrp.MapRobotPublisher("topic", "/a/topic")
        def tf_publisher_no_decorator(t, topic):
            pass

        self.assertIsInstance(tf_publisher_no_decorator, nrp.Neuron2Robot)

    def test_unavailable_service_name_fails(self):
        tf_manager = nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because the given service name is invalid

        @nrp.MapRobotServiceProxy("name", "unavailable_service_name", None)
        @nrp.Neuron2Robot()
        def test_tf(t, name):
            name.create_adapter(tf_manager)
            return t

        self.init_adapters()

        self.assertRaises(Exception, nrp.initialize)

    def init_adapters(self):
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)

if __name__ == "__main__":
    unittest.main()
