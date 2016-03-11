"""
This module contains the mapping of a CSV recorder object to input parameters
"""
__author__ = ''

from ._MappingSpecification import ParameterMappingSpecification
from ._CleanableTransferFunctionParameter import ICleanableTransferFunctionParameter

import logging
import tempfile
import csv
import os

logger = logging.getLogger(__name__)


class MapCSVRecorder(ParameterMappingSpecification):
    """
    Class to map a CSV recorder object to transfer function parameters
    """

    def __init__(self, parameter_name, filename, headers):
        """
        Maps a parameter to a variable in the specified scope (per-default: the transfer function)
        and if the variable does not yet exist initializes it with the provided value.
        A transfer function using it could look like this:

        @nrp.MapRobotSubscriber("joint_state",
                                Topic('/gazebo/joint_states',
                                gazebo_msgs.msg.JointStates))
        @nrp.MapCSVRecorder("recorder",
                            filename = "all_joints_positions.csv",
                            headers=["Name", "time", "Position"])
        @nrp.Robot2Neuron()
        def joint_state_monitor(t, joint_state, recorder):
            for i in range(0, len(joint_state.value.name)):
                recorder.record_entry(joint_state.value.name[i], t, joint_state.value.position[i])

        :param parameter_name: the name of the parameter
        :param filename: the name of the file to write
        :param headers: An array of string containing the name of the columns that will be written
        in the CSV file
        """
        super(MapCSVRecorder, self).__init__(parameter_name)
        self.__filename = filename
        self.__headers = headers

    def create_adapter(self, transfer_function_manager): # pylint: disable=unused-argument
        """
        Replaces the current mapping operator with the mapping result

        :return: A ready to use CSVRecorder object
        """
        return CSVRecorder(self.__filename, self.__headers)


class CSVRecorder(ICleanableTransferFunctionParameter):
    """
    Record value and memory and dump them to a temporary file when asked to.
    """
    def __init__(self, filename, headers):
        """
        Constructor. Pretty straightforward,

        :param filename: the filename to save to.
        :param headers: the name of the columns.
        """
        self.__filename = filename
        self.__generatedFiles = []
        self.__values = [headers]

    def record_entry(self, *values):
        """
        Record the values provided (in memory)

        :param values: Values to record
        """
        if (len(self.__values[0]) == len(values)):
            self.__values.append(values)
        else:
            raise ValueError("Number of arguments not matching the number of headers !")

    def dump_to_file(self):
        """
        Create a CSV file and save the path in a global variable

        :return: filename of the temporary CSV file holding the results and filename
        specified by the user
        """
        with tempfile.NamedTemporaryFile(delete=False) as f:
            writer = csv.writer(f)
            writer.writerows(self.__values)
            temporary_path = f.name
            f.flush()
            self.__generatedFiles.append(temporary_path)
        return self.__filename, temporary_path

    def cleanup(self):
        """
        Remove all temporary files that could have been generated.
        """
        for filepath in self.__generatedFiles:
            if os.path.realpath(filepath).startswith(tempfile.gettempdir()):
                os.remove(filepath)
        self.__generatedFiles = []
