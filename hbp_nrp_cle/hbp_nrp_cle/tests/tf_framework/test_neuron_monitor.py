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
from cle_ros_msgs.msg import SpikeRate, SpikeEvent
from hbp_nrp_cle.tests.tf_framework.husky import Husky

from hbp_nrp_cle.mocks.robotsim import MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim import MockBrainCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim.__devices.MockAbstractBrainDevice import MockDeviceGroup
from hbp_nrp_cle.brainsim.pynn_spiNNaker.devices import PyNNSpiNNakerSpikeRecorder
import unittest
from mock import Mock, PropertyMock, patch

__author__ = 'GeorgHinkel'


class NeuronMonitorTests(unittest.TestCase):
    def test_parameter_not_parsable_fails(self):
        nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because there is no mapping for parameter "neuron1"

        @nrp.NeuronMonitor(nrp.brain.foo, nrp.spike_recorder)
        def example_monitor(t, neuronA):
            return neuronA.voltage > 20

        self.init_adapters()

        self.assertRaises(Exception, nrp.initialize)

    def test_map_neuron_wrong_parameter_fails(self):
        nrp.start_new_tf_manager()

        # This test checks if the initialization fails as expected, because "neuronX" cannot be mapped to a parameter
        with self.assertRaises(Exception):
            @nrp.MapSpikeSink("neuronX", [1, 2, 3], nrp.leaky_integrator_exp)
            @nrp.NeuronMonitor(nrp.brain.foo, nrp.spike_recorder)
            def example_monitor(t, neuron1):
                return neuron1.voltage > 20

    def test_spike_recorder_monitor(self):
        nrp.start_new_tf_manager()
        self.init_adapters()

        @nrp.NeuronMonitor(nrp.brain.foo, nrp.spike_recorder)
        def my_monitor(t):
            return True

        nrp.initialize("test")
        my_monitor.run(42.0)
        msg = my_monitor.publisher.sent[-1]
        self.assertIsInstance(msg, SpikeEvent)
        self.assertEqual(msg.simulationTime, 42.0)
        self.assertEqual(msg.monitorName, "my_monitor")
        self.assertEqual(msg.neuronCount, 42)
        self.assertEqual(len(msg.spikes), 0)

        my_monitor.unregister()
        self.assertIsNone(my_monitor.device)

    def test_spike_recorder_monitor_map_neuron(self):
        nrp.start_new_tf_manager()
        self.init_adapters()

        @nrp.NeuronMonitor(nrp.map_neurons(range(0, 1), lambda i: nrp.brain.foo),
                           nrp.spike_recorder)
        def my_monitor(t):
            return True

        nrp.initialize("test")
        my_monitor.run(42.0)
        msg = my_monitor.publisher.sent[-1]
        self.assertIsInstance(msg, SpikeEvent)
        self.assertEqual(msg.simulationTime, 42.0)
        self.assertEqual(msg.monitorName, "my_monitor")
        self.assertEqual(msg.neuronCount, 42)
        self.assertEqual(len(msg.spikes), 0)

        my_monitor.unregister()
        self.assertIsNone(my_monitor.device)

    def test_spike_recorder_monitor_for_spinnaker(self):
        nrp.start_new_tf_manager()
        self.init_adapters()

        @nrp.NeuronMonitor(nrp.brain.foo, nrp.spike_recorder)
        def my_spike_monitor(t):
            return True

        nrp.initialize("test")
        with patch('hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.PyNNSpiNNakerSpikeRecorder.population_name',
                   new_callable=PropertyMock) as a:
            a.return_value = "my_population_name"
            my_spike_recorder = PyNNSpiNNakerSpikeRecorder()
            my_spike_monitor.device = MockDeviceGroup(PyNNSpiNNakerSpikeRecorder, [my_spike_recorder])
            my_spike_monitor.run(43.0)
            msg = my_spike_monitor.publisher.sent[-1]
            self.assertEqual(msg.populationLabel, "my_population_name")
            my_spike_monitor.unregister()
            self.assertIsNone(my_spike_monitor.device)

    def test_leaky_integrator_alpha_monitor(self):

        nrp.start_new_tf_manager()
        self.init_adapters()

        @nrp.NeuronMonitor(nrp.brain.foo, nrp.leaky_integrator_alpha)
        def my_monitor(t):
            return True

        nrp.initialize("test")
        my_monitor.run(42.0)
        msg = my_monitor.publisher.sent[-1]
        self.assertIsInstance(msg, SpikeRate)
        self.assertEqual(msg.simulationTime, 42.0)
        self.assertEqual(msg.monitorName, "my_monitor")
        self.assertEqual(msg.rate, my_monitor.device.voltage)

        my_monitor.unregister()
        self.assertIsNone(my_monitor.device)

    def test_leaky_integrator_exp_monitor(self):

        nrp.start_new_tf_manager()
        self.init_adapters()

        @nrp.NeuronMonitor(nrp.brain.foo, nrp.leaky_integrator_exp)
        def my_monitor(t):
            return True

        nrp.initialize("test")
        my_monitor.run(42.0)
        msg = my_monitor.publisher.sent[-1]
        self.assertIsInstance(msg, SpikeRate)
        self.assertEqual(msg.simulationTime, 42.0)
        self.assertEqual(msg.monitorName, "my_monitor")
        self.assertEqual(msg.rate, my_monitor.device.voltage)

        my_monitor.unregister()
        self.assertIsNone(my_monitor.device)

    def test_population_rate_monitor(self):

        nrp.start_new_tf_manager()
        self.init_adapters()

        @nrp.NeuronMonitor(nrp.brain.foo, nrp.population_rate)
        def my_monitor(t):
            return True

        nrp.initialize("test")
        my_monitor.run(42.0)
        msg = my_monitor.publisher.sent[-1]
        self.assertIsInstance(msg, SpikeRate)
        self.assertEqual(msg.simulationTime, 42.0)
        self.assertEqual(msg.monitorName, "my_monitor")
        self.assertEqual(msg.rate, my_monitor.device.rate)

        my_monitor.unregister()
        self.assertIsNone(my_monitor.device)

    def init_adapters(self):
        brain = MockBrainCommunicationAdapter()
        robot = MockRobotCommunicationAdapter()
        nrp.set_nest_adapter(brain)
        nrp.set_robot_adapter(robot)
        m = Mock()
        m.foo.size = 42
        nrp.config.brain_root = m

if __name__ == "__main__":
    unittest.main()
