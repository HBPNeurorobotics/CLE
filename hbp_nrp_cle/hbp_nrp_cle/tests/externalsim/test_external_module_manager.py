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
ExternalModuleManager unit test
"""

from hbp_nrp_cle.robotsim.AsynchronousServiceProxy import AsynchonousRospyServiceProxy
from hbp_nrp_cle.externalsim.ExternalModuleManager import ExternalModuleManager
import hbp_nrp_cle.externalsim.ExternalModule
from hbp_nrp_cle.tests.externalsim.test_external_module import MockSimpleFuture

import unittest
import copy
from mock import patch, MagicMock


# all the methods are inherited from unittest.TestCase
class TestExternalModuleManager(unittest.TestCase):

    def setUp(self):  # -> None
        self.mock_wait_for_service = patch('hbp_nrp_cle.externalsim.ExternalModule.rospy.wait_for_service').start()
        self.mock_service_proxy = patch('hbp_nrp_cle.robotsim.AsynchronousServiceProxy.ServiceProxy').start()

        self.module_names = ['/emi/module1_module/initialize', '/emi/module2_module/initialize']
        with patch('hbp_nrp_cle.externalsim.ExternalModuleManager.rosservice.get_service_list', return_value=self.module_names):
            self._ema = ExternalModuleManager()

    def tearDown(self):  # -> None
        self.mock_wait_for_service.close()
        self.mock_service_proxy.close()

    def test_init(self):
        # Check that service proxies for each module are initialized
        self.assertEquals(self.mock_wait_for_service.call_count, 3*len(self.module_names))
        self.assertEquals(self.mock_service_proxy.call_count, 3*len(self.module_names))

    def test_initialize_call(self):
        # Check that each module's initialize function was called
        with patch.object(hbp_nrp_cle.externalsim.ExternalModule.AsynchonousRospyServiceProxy, 'call',
                          return_value=MockSimpleFuture()) as call_method:
            self._ema.initialize()
            self.assertEquals(call_method.call_count, len(self.module_names))

    def test_run_step_call(self):
        # Check that each module's run_step function was called
        with patch.object(hbp_nrp_cle.externalsim.ExternalModule.AsynchonousRospyServiceProxy, 'call',
                          return_value=MockSimpleFuture()) as call_method:
            self._ema.run_step()
            self.assertEquals(call_method.call_count, len(self.module_names))

    def test_shutdown_call(self):
        # Check that each module's shutdown function was called
        with patch.object(hbp_nrp_cle.externalsim.ExternalModule.AsynchonousRospyServiceProxy, 'call',
                          return_value=MockSimpleFuture()) as call_method:
            self._ema.shutdown()
            self.assertEquals(call_method.call_count, len(self.module_names))


if __name__ == '__main__':
    unittest.main()
