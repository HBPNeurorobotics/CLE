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
Test the mocked brain communication, control and
 devices
moduleauthor: Michael.Weber@fzi.de
"""

from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder, \
    IPoissonSpikeGenerator, IDCSource, IACSource, INCSource, \
    IPopulationRate, IFixedSpikeGenerator, ILeakyIntegratorAlpha, \
    ILeakyIntegratorExp
from hbp_nrp_cle.mocks.brainsim._MockBrainCommunicationAdapter import \
    MockBrainCommunicationAdapter
from hbp_nrp_cle.mocks.brainsim._MockBrainControlAdapter import MockBrainControlAdapter
import unittest
from hbp_nrp_cle.mocks.brainsim.__devices.MockCurrentSource import MockACSource, MockDCSource
from hbp_nrp_cle.mocks.brainsim.__devices.MockSpikeGenerator import MockFixedSpikeGenerator, \
    MockPoissonSpikeGenerator
from hbp_nrp_cle.mocks.brainsim.__devices.MockLeakyIntegrator import MockLeakyIntegratorAlpha, \
    MockLeakyIntegratorExp
from hbp_nrp_cle.mocks.brainsim.__devices.MockNCSource import MockNCSource
from hbp_nrp_cle.mocks.brainsim.__devices.MockPopulationRate import MockPopulationRate
from hbp_nrp_cle.mocks.brainsim.__devices.MockSpikeRecorder import MockSpikeRecorder
from hbp_nrp_cle.mocks.brainsim.__devices.MockAbstractBrainDevice import MockDeviceGroup

__author__ = 'MichaelWeber'


class MockBrainAdaptersTest(unittest.TestCase):
    """
    Tests the communication and control adapter to the neuronal simulator
    """

    def setUp(self):
        """
        Instantiates the Mock communication and control adapter
        """
        self.control = MockBrainControlAdapter()
        self.assertEqual(self.control.is_alive(), False)
        self.control.initialize(timestep=0.1,
                                min_delay=0.1,
                                max_delay=4.0,
                                num_threads=1)
        self.communicator = MockBrainCommunicationAdapter()
        self.neurons_cond = "4"
        self.neurons_curr = "8"
        self.two_neurons_pop_cond = [[1, 2, 3, 4], [1, 2, 3, 4]]
        self.two_neurons_pop_curr = [[5, 6, 7, 8], [5, 6, 7, 8]]

        self.assertEqual(self.communicator.is_initialized, False)
        self.assertEqual(self.communicator.detector_devices, [])
        self.assertEqual(self.communicator.generator_devices, [])

    def test_initialize(self):
        """
        Test the initialization of the Mock Brain adapters
        """
        self.communicator.initialize()
        self.assertEqual(self.communicator.is_initialized, True)

    def test_register_spike_source(self):
        """
        Tests the registration of the generator __devices
        """
        device_0 = self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator)
        self.assertEqual(device_0, self.communicator.generator_devices[0])
        self.assertIsInstance(device_0, MockPoissonSpikeGenerator)

        print("Poisson Spike Generator Rate (before): ",
              self.communicator.generator_devices[0].rate)
        self.communicator.generator_devices[0].rate = 2.0
        print("Poisson Spike Generator Rate (after): ",
              self.communicator.generator_devices[0].rate)

        device_1 = self.communicator.register_spike_source(
            self.neurons_cond, IDCSource)
        self.assertEqual(device_1, self.communicator.generator_devices[1])
        self.assertIsInstance(device_1, MockDCSource)

        print("DC Amplitude (before): ",
              self.communicator.generator_devices[1].amplitude)
        self.communicator.generator_devices[1].amplitude = 2.0
        print("DC Amplitude (after): ",
              self.communicator.generator_devices[1].amplitude)

        device_2 = self.communicator.register_spike_source(
            self.neurons_cond, IACSource)
        self.assertEqual(device_2, self.communicator.generator_devices[2])
        self.assertIsInstance(device_2, MockACSource)

        device_3 = self.communicator.register_spike_source(
            self.neurons_cond, INCSource)
        self.assertEqual(device_3, self.communicator.generator_devices[3])
        self.assertIsInstance(device_3, MockNCSource)

        device_4 = self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator)
        self.assertEqual(device_4, self.communicator.generator_devices[4])
        self.assertIsInstance(device_4, MockPoissonSpikeGenerator)

        device_5 = self.communicator.register_spike_source(
            self.two_neurons_pop_cond, IPoissonSpikeGenerator)
        self.assertEqual(device_5, self.communicator.generator_devices[5])
        self.assertIsInstance(device_5, MockDeviceGroup)

        device_6 = self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IPoissonSpikeGenerator)
        self.assertEqual(device_6, self.communicator.generator_devices[6])
        self.assertIsInstance(device_6, MockDeviceGroup)

        device_7 = self.communicator.register_spike_source(
            self.neurons_cond, IPoissonSpikeGenerator, target="inhibitory")
        self.assertEqual(device_7, self.communicator.generator_devices[7])
        self.assertIsInstance(device_7, MockPoissonSpikeGenerator)

        device_8 = self.communicator.register_spike_source(
            self.neurons_curr, IPoissonSpikeGenerator, target="inhibitory")
        self.assertEqual(device_8, self.communicator.generator_devices[8])
        self.assertIsInstance(device_8, MockPoissonSpikeGenerator)

        device_9 = self.communicator.register_spike_source(
            self.neurons_cond, IFixedSpikeGenerator)
        self.assertEqual(device_9, self.communicator.generator_devices[9])
        self.assertIsInstance(device_9, MockFixedSpikeGenerator)

        device_10 = self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator)
        self.assertEqual(device_10, self.communicator.generator_devices[10])
        self.assertIsInstance(device_10, MockFixedSpikeGenerator)

        device_11 = self.communicator.register_spike_source(
            self.two_neurons_pop_cond, IFixedSpikeGenerator)
        self.assertEqual(device_11, self.communicator.generator_devices[11])
        self.assertIsInstance(device_11, MockDeviceGroup)

        device_12 = self.communicator.register_spike_source(
            self.two_neurons_pop_curr, IFixedSpikeGenerator)
        self.assertEqual(device_12, self.communicator.generator_devices[12])
        self.assertIsInstance(device_12, MockDeviceGroup)

        device_13 = self.communicator.register_spike_source(
            self.neurons_cond, IFixedSpikeGenerator, target="inhibitory")
        self.assertEqual(device_13, self.communicator.generator_devices[13])
        self.assertIsInstance(device_13, MockFixedSpikeGenerator)

        device_14 = self.communicator.register_spike_source(
            self.neurons_curr, IFixedSpikeGenerator, target="inhibitory")
        self.assertEqual(device_14, self.communicator.generator_devices[14])
        self.assertIsInstance(device_14, MockFixedSpikeGenerator)

    def test_register_spike_sink(self):
        """
        Tests the registration of the detector __devices
        """
        device_0 = self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeRecorder)
        self.assertEqual(device_0, self.communicator.detector_devices[0])
        self.assertIsInstance(device_0, MockSpikeRecorder)

        device_1 = self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorAlpha)
        self.assertEqual(device_1, self.communicator.detector_devices[1])
        self.assertIsInstance(device_1, MockLeakyIntegratorAlpha)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device LeakyIntegratorAlpha): ",
              self.communicator.detector_devices[1].voltage)

        device_2 = self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorAlpha)
        self.assertEqual(device_2, self.communicator.detector_devices[2])
        self.assertIsInstance(device_2, MockLeakyIntegratorAlpha)

        device_3 = self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, ILeakyIntegratorAlpha)
        self.assertEqual(device_3, self.communicator.detector_devices[3])
        self.assertIsInstance(device_3, MockDeviceGroup)

        device_4 = self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, ILeakyIntegratorAlpha)
        self.assertEqual(device_4, self.communicator.detector_devices[4])
        self.assertIsInstance(device_4, MockDeviceGroup)

        device_5 = self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorAlpha, target='inhibitory')
        self.assertEqual(device_5, self.communicator.detector_devices[5])
        self.assertIsInstance(device_5, MockLeakyIntegratorAlpha)

        device_6 = self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorExp)
        self.assertEqual(device_6, self.communicator.detector_devices[6])
        self.assertIsInstance(device_6, MockLeakyIntegratorExp)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device LeakyIntegratorExp): ",
              self.communicator.detector_devices[6].voltage)

        device_7 = self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorExp)
        self.assertEqual(device_7, self.communicator.detector_devices[7])
        self.assertIsInstance(device_7, MockLeakyIntegratorExp)

        device_8 = self.communicator.register_spike_sink(
            self.two_neurons_pop_cond, ILeakyIntegratorExp)
        self.assertEqual(device_8, self.communicator.detector_devices[8])
        self.assertIsInstance(device_8, MockDeviceGroup)

        device_9 = self.communicator.register_spike_sink(
            self.two_neurons_pop_curr, ILeakyIntegratorExp)
        self.assertEqual(device_9, self.communicator.detector_devices[9])
        self.assertIsInstance(device_9, MockDeviceGroup)

        device_10 = self.communicator.register_spike_sink(
            self.neurons_curr, ILeakyIntegratorExp, target='inhibitory')
        self.assertEqual(device_10, self.communicator.detector_devices[10])
        self.assertIsInstance(device_10, MockLeakyIntegratorExp)

        device_11 = self.communicator.register_spike_sink(
            self.neurons_curr, IPopulationRate)
        self.assertEqual(device_11, self.communicator.detector_devices[11])
        self.assertIsInstance(device_11, MockPopulationRate)

        self.control.run_step(0.1)
        print("Voltage of IF neuron (= device PopulationRate): ",
              self.communicator.detector_devices[11].rate)

    def test_refresh_buffers(self):
        """
        Tests the refresh_buffers function
        """
        device_0 = self.communicator.register_spike_sink(
            self.neurons_cond, ISpikeRecorder)
        self.assertEqual(device_0, self.communicator.detector_devices[0])
        #self.assertIsInstance(device_0, SpikeDetector)

        device_1 = self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorAlpha)
        self.assertEqual(device_1, self.communicator.detector_devices[1])
        self.assertIsInstance(device_1, MockLeakyIntegratorAlpha)

        device_2 = self.communicator.register_spike_sink(
            self.neurons_cond, ILeakyIntegratorExp)
        self.assertEqual(device_2, self.communicator.detector_devices[2])
        self.assertIsInstance(device_2, MockLeakyIntegratorExp)

        device_3 = self.communicator.register_spike_sink(
            self.neurons_cond, IPopulationRate)
        self.assertEqual(device_3, self.communicator.detector_devices[3])
        self.assertIsInstance(device_3, MockPopulationRate)

        time = 0.2
        self.control.run_step(0.1)
        self.communicator.refresh_buffers(time)
        self.control.shutdown()
        self.assertEqual(self.communicator.refreshed_at[0], time)


if __name__ == "__main__":
    unittest.main()
