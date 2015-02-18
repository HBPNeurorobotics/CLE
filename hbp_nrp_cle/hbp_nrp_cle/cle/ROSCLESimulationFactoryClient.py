"""
Helper class that takes care of making the appropriate ROS call(s) to
start a simulation.
"""

import rospy
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of the CLE (this) repository.
from ROSCLEServicesDefinitions import srv

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"


class ROSCLESimulationFactoryClient(object):

    """
    Helper class for the simualtion factory ROS service started with class
    ROSCLESimulationFactory.
    """

    def __init__(self):
        """
        Create the client. If the service is not available after 10 seconds, a ROSException
        will be raised.
        """

        self.__start_new_simulation_service = rospy.ServiceProxy(
            '/ros_cle_simulation/start_new_simulation',
            srv.start_new_simulation)
        self.__start_new_simulation_service.wait_for_service(timeout=10)

    def start_new_simulation(self, environment_file, generated_cle_script_file):
        """
        Start the simulation.
        """
        self.__start_new_simulation_service(environment_file, generated_cle_script_file)