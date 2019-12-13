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

from hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection import Translator, reset, register_devices, shutdown, get_key
import unittest
from mock import patch, Mock

class TestEthernetControlConnections(unittest.TestCase):
    def test_key_generation(self):
        reset()
        self.assertEqual(1, get_key(None))
        self.assertEqual(2, get_key(None))
        self.assertEqual(4, get_key(4))
        self.assertEqual(5, get_key(None))
        self.assertEqual(6, get_key(3))

    def test_translator(self):
        t = Translator()
        device = Mock()
        device.device_control_key = "foo"
        t.register_translation(device)
        packet = Mock()
        packet.key = "foo"
        packet.payload = "bar"
        t.translate_control_packet(packet)
        device.run.assert_called_once_with("bar")
        t.reset()
        t.translate_control_packet(packet)
        # still only called once
        device.run.assert_called_once_with("bar")

    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection.translator")
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection.sim")
    def test_register_devices(self, sim, translator):
        device1 = Mock()
        device2 = Mock()
        reset()
        sim.Population()._get_vertex.get_outgoing_partition_ids.return_value = ["foo", "bar"]
        register_devices([device1, device2])

        translator.register_translation.assert_any_call(device1)
        translator.register_translation.assert_any_call(device2)
        self.assertEqual(1, sim.external_devices.ExternalDeviceLifControl.call_count)
        sim.external_devices.EthernetControlPopulation.assert_any_call(2, sim.external_devices.ExternalDeviceLifControl.return_value)

        device3 = Mock()
        register_devices([device3], foo="bar")

        # assert that no other ethernet control connection has been established
        translator.register_translation.assert_any_call(device3)
        self.assertEqual(2, sim.external_devices.ExternalDeviceLifControl.call_count)
        sim.external_devices.EthernetControlPopulation.assert_any_call(1, sim.external_devices.ExternalDeviceLifControl.return_value)


    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection.translator")
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.__EthernetControlConnection.sim")
    def test_new_connection_after_shutdown(self, sim, translator):
        device1 = Mock()
        reset()
        sim.Population()._get_vertex.get_outgoing_partition_ids.return_value = ["foo", "bar"]

        register_devices([device1])
        translator.register_translation.assert_called_once_with(device1)
        sim.external_devices.EthernetControlPopulation.assert_called_once()

        shutdown()
        reset()
        register_devices([device1])
        self.assertEqual(2, sim.external_devices.EthernetControlPopulation.call_count)
        self.assertEqual(2, translator.register_translation.call_count)
