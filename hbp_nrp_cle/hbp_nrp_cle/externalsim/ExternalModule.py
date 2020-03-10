"""
Includes the corresponding CLE class for external ROS modules.
"""

__author__ = 'Omer Yilmaz'

import logging
import rospy
from hbp_nrp_cle.robotsim.AsynchronousServiceProxy import AsynchonousRospyServiceProxy
from cle_ros_msgs.srv import Initialize, RunStep, Shutdown
from hbp_nrp_cle.robotsim.GazeboHelper import TIMEOUT

logger = logging.getLogger('hbp_nrp_cle')


class ExternalModule(object):
    """
    External ROS modules have initialize, run_step and shutdown methods.
    This class has the corresponding initialize, run_step and shutdown
    methods which triggers the external ones through ROS service proxies.
    Objects of this class is synchronized with the Deterministic Closed
    Loop Engine via ExternalModuleManager.
    """

    def __init__(self, module_name):
        self.service_name = 'emi/' + module_name + '_module/'
        self.response = None

        rospy.wait_for_service(self.service_name + 'initialize', timeout=TIMEOUT)
        self.initialize_proxy = AsynchonousRospyServiceProxy(
                self.service_name + 'initialize', Initialize, persistent=False)

        rospy.wait_for_service(self.service_name + 'run_step', timeout=TIMEOUT)
        self.run_step_proxy = AsynchonousRospyServiceProxy(
                self.service_name + 'run_step', RunStep, persistent=True)

        rospy.wait_for_service(self.service_name + 'shutdown', timeout=TIMEOUT)
        self.shutdown_proxy = AsynchonousRospyServiceProxy(
                self.service_name + 'shutdown', Shutdown, persistent=False)

    def initialize(self):
        """
        This method triggers the initialize method served at the external module synchronously
        with the CLE.
        """
        try:
            self.response = self.initialize_proxy.call()
            return self.response
        except rospy.ServiceException as e:
            logger.exception(self.service_name + 'initialize call failed: %s' % e)

    def run_step(self):
        """
        This method triggers the run_step method served at the external module synchronously
        with the CLE.
        """
        try:
            return self.run_step_proxy.call()
        except rospy.ServiceException as e:
            logger.exception(self.service_name + 'run_step call failed: %s' % e)

    def shutdown(self):
        """
        This method triggers the shutdown method served at the external module synchronously
        with the CLE.
        """
        try:
            return self.shutdown_proxy.call()
        except rospy.ServiceException as e:
            logger.exception(self.service_name + 'shutdown call failed: %s' % e)