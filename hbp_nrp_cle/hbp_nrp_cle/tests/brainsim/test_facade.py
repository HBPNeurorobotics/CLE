# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
Tests the brainsim facade
"""

__author__ = "Sebastian Krach"
import unittest
import mock
import hbp_nrp_cle.brainsim
import hbp_nrp_cle.brainsim.config

class TestBrainAdapterFacade(unittest.TestCase):
    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_COMMUNICATION_ADAPTER")
    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_CONTROL_ADAPTER")
    def test_control_adapter(self, cont_mock, com_mock):
        cont_mock.return_value = "CONTROL_TYPE"
        com_mock.return_value = "COMMUNICATION_TYPE"

        hbp_nrp_cle.brainsim.config.communication_adapter_type = None
        hbp_nrp_cle.brainsim.config.control_adapter_type = None

        adapter = hbp_nrp_cle.brainsim.instantiate_control_adapter()
        self.assertEqual(adapter, "CONTROL_TYPE")

        with mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_CONTROL_ADAPTER") as new_cont:
            new_cont.return_value = "SHOULD_NOT_PROPAGATE"
            adapter = hbp_nrp_cle.brainsim.instantiate_control_adapter()
            self.assertEqual(adapter, "CONTROL_TYPE")

        adapter = hbp_nrp_cle.brainsim.instantiate_communication_adapter()
        self.assertEqual(adapter, "COMMUNICATION_TYPE")


    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_COMMUNICATION_ADAPTER")
    @mock.patch("hbp_nrp_cle.brainsim.__Facade.DEFAULT_CONTROL_ADAPTER")
    def test_communication_adapter(self, cont_mock, com_mock):
        cont_mock.return_value = "CONTROL_TYPE"
        com_mock.return_value = "COMMUNICATION_TYPE"

        hbp_nrp_cle.brainsim.config.communication_adapter_type = None
        hbp_nrp_cle.brainsim.config.control_adapter_type = None

        adapter = hbp_nrp_cle.brainsim.instantiate_communication_adapter()
        self.assertEqual(adapter, "COMMUNICATION_TYPE")


