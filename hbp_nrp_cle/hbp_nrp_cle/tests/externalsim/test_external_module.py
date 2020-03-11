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
ExternalModule unit test
"""

import rospy
from iba_manager.srv import Initialize, RunStep, Shutdown
from hbp_nrp_cle.externalsim.ExternalModule import ExternalModule

import unittest
from mock import patch


# all the methods are inherited from unittest.TestCase
class TestExternalModule(unittest.TestCase):

    def setUp(self):  # -> None
        self.mock_wait_for_service = patch('hbp_nrp_cle.externalsim.ExternalModule.rospy.wait_for_service').start()
        self.mock_service_proxy = patch('rospy.ServiceProxy').start()

        self._module_name = "module"
        self._em = ExternalModule(self._module_name)

    def tearDown(self):  # -> None
        self.mock_wait_for_service.close()
        self.mock_service_proxy.close()

    def test_init(self):
        # Check that service proxies for each module are initialized
        self.assertEquals('emi/' + self._module_name + '_module/', self._em.service_name)

        self.assertEquals(self.mock_wait_for_service.call_count, 3)
        self.assertEquals(self.mock_service_proxy.call_count, 3)
        self.assertEquals(self.mock_service_proxy.call_args_list[0][0][0], self._em.service_name + 'initialize')
        self.assertEquals(self.mock_service_proxy.call_args_list[0][0][1], Initialize)
        self.assertEquals(self.mock_service_proxy.call_args_list[1][0][0], self._em.service_name + 'run_step')
        self.assertEquals(self.mock_service_proxy.call_args_list[1][0][1], RunStep)
        self.assertEquals(self.mock_service_proxy.call_args_list[2][0][0], self._em.service_name + 'shutdown')
        self.assertEquals(self.mock_service_proxy.call_args_list[2][0][1], Shutdown)

    def test_initialize_call(self):
        # Check that each module's initialize function was called
        self._em.initialize()
        self.mock_service_proxy.call.assert_called_once()

    def test_run_step_call(self):
        # Check that each module's run_step function was called
        self._em.run_step()
        self.mock_service_proxy.call.assert_called_once()

    def test_shutdown_call(self):
        # Check that each module's shutdown function was called
        self._em.shutdown()
        self.mock_service_proxy.call.assert_called_once()


if __name__ == '__main__':
    unittest.main()
