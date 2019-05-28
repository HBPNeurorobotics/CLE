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

import unittest
from hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__SpikeInjector import SpiNNakerSpikeInjector
import hbp_nrp_cle.brainsim.pynn_spiNNaker.__LiveSpikeConnection as lsc
from mock import patch, Mock
import numpy as np


class TestSpikeInjector(unittest.TestCase):
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__SpikeInjector.spynnaker")
    @patch("hbp_nrp_cle.brainsim.pynn_spiNNaker.devices.__SpikeInjector.live_connections")
    def test_spike_injector_injects(self, live_connections, sim):
        # unpatch get_port
        population = Mock()
        population.label = "foo"
        population.size = 42
        dev = SpiNNakerSpikeInjector.create_new_device(population)

        dev.connect(population)
        # does not raise
        dev.inject_spikes()
        self.assertTrue(live_connections.register_sender.called)
