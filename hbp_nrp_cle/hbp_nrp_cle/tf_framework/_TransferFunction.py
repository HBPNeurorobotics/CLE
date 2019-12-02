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
Defines the base class for a transfer function
"""

import inspect
import textwrap
import logging
from hbp_nrp_cle.tf_framework import TFException, TFRunningException
from abc import abstractmethod
import sys

logger = logging.getLogger(__name__)

__author__ = 'GeorgHinkel'


class TransferFunction(object):
    """
    Represents the base class for a transfer function
    """

    # pylint: disable=unused-argument
    @staticmethod
    def __default_excepthook(tf, error, tb):
        """
        The default exception handler for transfer functions

        :param tf: The transfer function that raised the exception
        :param error: The exception object that was raised
        :param tb: The original traceback
        """
        raise TFException(tf.name, str(error), "Runtime")

    excepthook = __default_excepthook

    def __init__(self, triggers=None, throttling_rate=None):
        self._params = []
        self._func = None
        self.__active = False
        self.__priority = 0
        self.__local_data = {}
        self.__source = None
        self.__elapsed_time = 0.0
        self.__updated_since_last_error = True
        self.__publish_error_callback = None
        self.__triggers = triggers
        if triggers is None:
            self.__triggers = ["t"]
        elif isinstance(self.__triggers, str):
            self.__triggers = [self.__triggers]
        elif not isinstance(triggers, list):
            raise Exception("The triggers should be a list of parameters that trigger execution")
        if isinstance(throttling_rate, (int, float)) and throttling_rate > 0:
            self.__min_delta_t = 1.0 / throttling_rate
        elif throttling_rate is None:
            self.__min_delta_t = 0.0001
        else:
            raise Exception("Throttling should be a maximum frequency of TF execution in hertz")

    def should_run(self, t):
        """
        Steps the simulation time to t and decides whether the TF should be run in this timeslot

        :param t: The simulation time
        :return: True, if the simulation should be run, otherwise False
        """
        return self._params[0] + self.__min_delta_t <= t

    @property
    def name(self):
        """
        Gets the name of the Transfer Function

        :return: The name of the Transfer Function as string
        """
        return self._func.__name__

    @property
    def params(self):
        """
        Gets the simulation adapters for this transfer function, either robot subscriber, publisher
        or neuronal simulator device adapters

        :return: A list of adapters
        """
        return self._params

    @property
    def triggers(self):
        """
        Gets the triggers of the Transfer Function

        :return: A list of triggers or the simulation time t
        """
        return self.__triggers

    @property
    def elapsed_time(self):
        """
        Gets the elapsed time for this Transfer Function

        :return: The elapsed time for this TF in seconds
        """
        return self.__elapsed_time

    @elapsed_time.setter
    def elapsed_time(self, value):
        """
        Sets the elapsed time for this Transfer Function
        """
        self.__elapsed_time = value

    @property
    def updated(self):
        """
        Gets the update flag of the transfer function.

        :return: A boolean that tells if the source code was updated
                 but has its loading still pending, it is
                 True if the source code was updated but not loaded by an exec statement
                 False otherwise
        """

        return self.__updated_since_last_error

    @property
    def source(self):
        """
        Gets the source code of this transfer function.

        :return: A string containing the transfer function source code correctly indented.

        .. WARNING:: The source code is read from the generated file based on the template.
                     If someone patches the transfer function, the patched code will not be
                     returned (see python inspect module documentation).
        """

        if self.__source is None:
            self.__source = "# Transfer function not loaded properly."
            if self._func:
                try:
                    self.__source = textwrap.dedent(inspect.getsource(self._func))
                except IOError as e:
                    error_msg = "The transfer function source code cannot be retrieved."
                    logger.error(error_msg)
                    logger.error(e)
                    self.__source = "# " + error_msg

        return self.__source

    @source.setter
    def source(self, source):
        """
        Sets the source code of this transfer function.

        :param source: String containing transfer function's source code
        """
        self.__source = source
        self.__updated_since_last_error = True

    @property
    def active(self):
        """
        Gets the activation state of this transfer function

        :return: the activation state of this transfer function

        """
        return self.__active

    @active.setter
    def active(self, bool_value):
        """
        Sets the activation state of this transfer function.

        :param bool_value: bool denoting the activation state of this transfer function

        """
        if bool_value is not None and type(bool_value) == bool:
            self.__active = bool_value

    @property
    def priority(self):
        """
        Gets the execution priority of this transfer function

        :return: the execution priority of this transfer function

        """

        return self.__priority

    @priority.setter
    def priority(self, priority_value):
        """
        Sets the execution priority of this transfer function.

        :param priority_value: positive integer denoting the execution priority of this transfer
        function

        """
        try:
            self.__priority = int(priority_value)
        except (ValueError, TypeError):
            self.__priority = 0

    @abstractmethod
    def __call__(self, func):
        """
        Applies the transfer functions object to the given function

        :param func: The function body for this transfer function
        :return The transfer function object
        """
        raise Exception("Must be implemented by concrete realization of transfer function.")

    def _init_function(self, func, funcs_list):
        """
        Initializes the transfer function object and adds it to the specified collection of
        functions.

        :param func: The function body for this transfer function
        :param funcs_list: The collection of functions the transfer function should be appended to
        """
        if self._func is None:
            self._func = func

            funcs_list.append(self)
            args = inspect.getargspec(func).args
            if args[0] != "t":
                raise Exception("The first parameter of a transfer function must be the time!")
            self._params = list(args)
            for t in self.__triggers:
                if t not in self._params:
                    raise Exception("The specified trigger {0} is invalid, "
                                    "there is no parameter with that name".format(t))
            self._params[0] = -float('inf')
        else:
            raise Exception("It is not allowed to change the underlying function of a Transfer "
                            "Function after it has been initially set.")

    def check_params(self):  # -> None:
        """
        Checks whether all parameters have been mapped properly

        :exception: Exception if a parameter was not mapped
        """
        for param in self._params:
            if param != "t" and type(param) == str:
                raise Exception("Parameter %s was not mapped properly" % param)

    def run(self, t):  # -> None:
        """
        Runs this transfer function at the given simulation time

        :param t: The simulation time
        """
        # pylint: disable=broad-except
        try:
            self._params[0] = t
            return self._func(*self._params)
        except Exception, e:
            self._handle_error(e, sys.exc_info()[2])
            raise TFRunningException(str(e))

    def _handle_error(self, e, tb):
        """
        Handles the given exception

        :param e: The exception that occurred
        :param tb: The traceback of the error
        """
        TransferFunction.excepthook(self, e, tb)
        self.__updated_since_last_error = False

    def initialize(self, tfm, bca_changed, rca_changed):
        """
        Initializes this transfer function to be used with the given TFM

        :param tfm: The Transfer Function Manager
        :param bca_changed: True, if the brain communication adapter has changed
        :param rca_changed: True, if the robot communication adapter has changed
        """
        pass

    @abstractmethod
    def unregister(self):
        """
        Un-register any devices specific to this transfer function implementation.
        """
        raise Exception("Must be implemented by concrete realization of transfer function.")


class FlawedTransferFunction(object):
    """
    Represents the class for a flawed transfer function, i.e a TF deemed invalid because has failed
    the checks performed on loading.
    """

    def __init__(self, name, source, error=None):
        self.__name = name
        self.__source = source
        self.__error = error

    @property
    def source(self):
        """
        Gets the source code of this transfer function.

        :return: A string containing the transfer function source code

        """
        return self.__source

    @property
    def name(self):
        """
        Gets the name this transfer function.

        :return: A string containing the name of the transfer function

        """
        return self.__name

    @property
    def error(self):
        """
        Gets the exception object raised by the failed loading of this transfer function.

        :return: the exception raised by the failed loading of this transfer function.

        """
        return self.__error

    # pylint: disable=no-self-use
    @property
    def active(self):
        """
        A flawed transfer function can't be active.

        :return: False

        """
        return False

    # pylint: disable=no-self-use
    @active.setter
    def active(self, value):
        """
        Since a flawed transfer function can't be active, it doesn't do anything.
        """
        pass

    @property
    def priority(self):
        """
        A flawed transfer function can't be executed.

        :return: False

        """
        return None

    # pylint: disable=no-self-use
    @priority.setter
    def priority(self, value):
        """
        Since a flawed transfer function can't be executed, it doesn't do anything.
        """
        pass

    @property
    def params(self):
        """
        A Flawed TF has no adapter

        :return: An empty list
        """
        return []
