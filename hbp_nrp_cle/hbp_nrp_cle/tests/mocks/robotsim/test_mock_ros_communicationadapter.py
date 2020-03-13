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
Represents an implementation of the robot communication adapter actually
using ROS
"""

from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosServiceProxy, logger, Service
import rospy
import rosservice


class MockRosServiceProxy(RosServiceProxy):
    """
    Represents a robot service response actually using ROS
    """
    def __init__(self, service, initial_response=None, mock_service=rospy.Service):
        self.__value = initial_response
        self.__args = []
        self.__kwargs = {}

        self._RosServiceProxy__response_value = initial_response
        self._RosServiceProxy__service_proxy = mock_service

    @property
    def service_proxy(self):
        return self._RosServiceProxy__service_proxy
