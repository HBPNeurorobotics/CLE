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
This module contains a TF decorator for ROS service proxy interaction
"""

__author__ = 'Michael Zechmair'

from ._MappingSpecification import ParameterMappingSpecification
from hbp_nrp_cle.robotsim.RobotInterface import Service

import logging
logger = logging.getLogger(__name__)


class MapRobotServiceProxy(ParameterMappingSpecification):
    """
    ROS service proxy to interface with services
    """

    def __init__(self, parameter_name, service_name, service_type, persistent=False, headers=None):
        """
        Initializes the service proxy parameters

        :param parameter_name: Variable Name
        :param service_name: Service Name
        :param service_type: Service Type
        :param persistent: Persistence of Service.
        Look at rospy.ServiceProxy documentation for details
        :param headers: Service headers.
        Look at rospy.ServiceProxy documentation for details
        """
        super(MapRobotServiceProxy, self).__init__(parameter_name)

        self._service = Service(service_name, service_type)
        self._service_persistence = persistent
        self._service_headers = headers

        self._service_proxy = None

    def set_request_args(self, *args, **kwargs):
        """
        Update service proxy request arguments
        """
        self._service_proxy.set_request_args(*args, **kwargs)

    def update_value(self, *args, **kwargs):
        """
        Update service response value
        :return Returns Service Response
        """
        self._service_proxy.update_value(*args, **kwargs)

    def create_adapter(self, tf_manager):
        """
        Create a RosServiceProxy adapter
        """
        self._service_proxy = \
            tf_manager.robot_adapter.register_service_proxy(self._service,
                                                            persistent=self._service_persistence,
                                                            headers=self._service_headers)
        return self._service_proxy

    def create_tf(self):
        pass
