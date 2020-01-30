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
This module contains the implementation of a transfer functions manager
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_cle.robotsim.RobotInterface import IRobotCommunicationAdapter
from hbp_nrp_cle.brainsim.BrainInterface import IBrainCommunicationAdapter, IBrainDevice
from hbp_nrp_cle.tf_framework import FlawedTransferFunction
from ._TransferFunctionInterface import ITransferFunctionManager
from . import BrainParameterException
from . import TFRunningException
import itertools
import logging
import time

logger = logging.getLogger(__name__)


class TransferFunctionManager(ITransferFunctionManager):
    """
    Represents a transfer functions node
    """

    def __init__(self):  # -> None:
        """
        Creates a new transfer functions node
        """

        self.__n2r = []
        self.__r2n = []
        self.__silent = []
        self.__flawed = []
        self.__robotAdapter = None
        self.__nestAdapter = None
        self.__initialized = False
        self.__global_data = {}
        self.__tf_priority = lambda tf: tf.priority if tf.priority else 0

    @property
    def n2r(self):  # -> list:
        """
        Gets a list of transfer functions from the neuronal simulator to the world simulation
        """
        return self.__n2r

    @property
    def r2n(self):  # -> list:
        """
        Gets a list of transfer functions from the world simulator to the neuronal simulation
        """
        return self.__r2n

    @property
    def silent(self):  # -> list
        """
        Gets a list of transfer functions not controlled by the orchestration
        """
        return self.__silent

    @property
    def flawed(self):  # -> list:
        """
        Gets a list of faulty transfer functions
        """
        return self.__flawed

    @property
    def global_data(self):
        """
        Gets the global variable storage dictionary for the transfer functions

        :return: the dictionary mapping variable name to value
        """
        return self.__global_data

    @staticmethod
    def run_tf(tf, t):  # -> None:
        """
        Runs the transfer functions for the given point in simulation time

        :param tf: the transfer function
        :param t: The simulation time
        """
        if tf.active and tf.should_run(t):
            start = time.time()
            tf.run(t)
            tf.elapsed_time += time.time() - start

    def run_neuron_to_robot(self, t):  # -> None:
        """
        Runs the transfer functions from the neuronal simulator towards the robot

        :param t: The simulation time
        """
        for _n2r in self.__n2r:
            try:
                TransferFunctionManager.run_tf(_n2r, t)
            except TFRunningException as tf_exception:
                self.__flawed.append(FlawedTransferFunction(_n2r.name, _n2r.source, tf_exception))
                self.__n2r.remove(_n2r)

    def run_robot_to_neuron(self, t):  # -> None:
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation

        :param t:  The simulation time
        """
        for _r2n in self.__r2n:
            try:
                TransferFunctionManager.run_tf(_r2n, t)
            except TFRunningException as tf_exception:
                self.__flawed.append(FlawedTransferFunction(_r2n.name, _r2n.source, tf_exception))
                self.__r2n.remove(_r2n)

    def run_tfs(self, t):
        """
        Runs all the transfer functions in the order specified in bibi file

        :param t:  The simulation time
        """
        _tfs = self.transfer_functions(sorted_=True)

        for tf in _tfs:
            try:
                TransferFunctionManager.run_tf(tf, t)
            except TFRunningException as tf_exception:
                self.__flawed.append(FlawedTransferFunction(tf.name, tf.source, tf_exception))
                if tf in self.__n2r:
                    self.__n2r.remove(tf)
                elif tf in self.__r2n:
                    self.__r2n.remove(tf)

    @property
    def robot_adapter(self):  # -> IRobotCommunicationAdapter:
        """
        Gets or sets the adapter to the world simulation
        """
        return self.__robotAdapter

    @robot_adapter.setter
    def robot_adapter(self, robot_adapter):  # -> None:
        """
        Sets the robot adapter

        :param robot_adapter: The new robot adapter
        """
        if self.__initialized:
            raise Exception("Cannot exchange robot adapter after node has been initialized!")
        else:
            if not isinstance(robot_adapter, IRobotCommunicationAdapter):
                raise Exception("The given object is not a valid robot adapter!")
            self.__robotAdapter = robot_adapter

    @property
    def initialized(self):
        """
        Determines whether the current tf manager has already been initialized

        :return: True, if the TF manager has been initialized, otherwise False
        """
        return self.__initialized

    @property
    def brain_adapter(self):  # -> IBrainCommunicationAdapter:
        """
        Gets or sets the adapter to the brain simulation
        """
        return self.__nestAdapter

    # Pylint has a problem because the setter is not specified in the interface
    # pylint: disable=arguments-differ
    @brain_adapter.setter
    def brain_adapter(self, nest_adapter):  # -> None:
        """
        Sets the nest adapter

        :param nest_adapter: The new brain simulation adapter
        """
        if self.__initialized:
            raise Exception("Cannot exchange brainsim adapter after node has been initialized!")
        else:
            if not isinstance(nest_adapter, IBrainCommunicationAdapter):
                raise Exception("The given object is not a valid brain communication adapter")
            self.__nestAdapter = nest_adapter

    def transfer_functions(self, flawed=False, sorted_=False):
        """
        Gets a list of transfer functions managed by this instance

        :param flawed: if True the list will include also flawed TFs
        :param sorted_: if True the list is sorted by transfer function priority attribute.
        Flawed TFs, if any, come last.
        :return: A list of transfer functions
        """

        proper_tfs = self.__n2r + self.__r2n + self.__silent
        if sorted_:
            proper_tfs.sort(key=self.__tf_priority, reverse=True)

        return proper_tfs if not flawed else proper_tfs + self.__flawed

    def initialize_tf(self, tf, activation=True):
        """
        Initializes the given transfer function

        This method is used if a transfer function is replaced after initialization of the tf
        manager

        :param tf: The transfer function
        :param activation: desired activation state; True for activated, False otherwise
        """
        logger.info("Initialize transfer function " + repr(tf))
        tf.check_params()
        tf.elapsed_time = 0.0

        if hasattr(tf, 'topic') and tf.topic is not None:
            saved = tf.topic
            tf.topic = self.__robotAdapter.register_publish_topic(saved)
            tf.topic.spec = saved

        for i in range(1, len(tf.params)):
            param = tf.params[i]
            tf.params[i] = tf.params[i].create_adapter(self)
            tf.params[i].spec = param
            tf.__dict__[param.name] = tf.params[i]

        if "t" not in tf.triggers:
            if tf in self.__n2r:
                self.__n2r.remove(tf)
            elif tf in self.__r2n:
                self.__r2n.remove(tf)
            self.__silent.append(tf)

        tf.initialize(self, True, True)
        self.activate_tf(tf, activation)
        self._update_trigger(tf)

    @staticmethod
    def _update_trigger(tf):
        """
        Reconnects the TF to the specified trigger devices

        :param tf: The transfer function
        """
        for i in range(0, len(tf.triggers)):
            trigger = tf.triggers[i]
            name = trigger if isinstance(trigger, str) else trigger.spec.name

            if name == "t":
                continue

            trigger_device = None
            for dev in tf.params:
                if isinstance(dev, float):
                    continue
                if dev.spec.name == name:
                    trigger_device = dev
                    break
            # trigger device is not None due to previous checks
            try:
                trigger_device.register_tf_trigger(tf)
            except AttributeError:
                raise Exception("The device connected for {0} does not support triggers"
                                .format(name))
            tf.triggers[i] = trigger_device

    def activate_tf(self, tf, activate):
        """
        Change the activation state of tf

        :param tf: the tf on which to apply the change
        :param activate: a boolean value denoting the new activation state
        """
        if activate is None or type(activate) != bool:
            return

        if activate is True:
            if not tf.active:
                self._activate_tf(tf, True)
        else:
            if tf.active:
                self._activate_tf(tf, False)

    def _activate_tf(self, tf, activate):
        """
        Change the activation state of tf.

        It (de-)activates tf and its brain devices.

        :param tf: the tf on which to apply the change
        :param activate: a boolean value denoting the new activation state
        """

        # change TF activation state
        tf.active = activate

        # change TF's devices activation state
        for i in range(1, len(tf.params)):
            if isinstance(tf.params[i], IBrainDevice):
                self.brain_adapter.activate_device(tf.params[i], activate)

    def initialize(self, name):
        """
        Initializes the transfer Function node with the given name

        :param name: The name for this transfer function node
        """
        if self.__initialized:
            return

        logger.info("Initialize transfer functions node " + name)

        if not isinstance(self.__nestAdapter, IBrainCommunicationAdapter):
            raise Exception("The brain adapter is configured incorrectly")
        if not isinstance(self.__robotAdapter, IRobotCommunicationAdapter):
            raise Exception("The robot adapter is configured incorrectly")

        # Wire transfer functions
        for tf in itertools.chain(self.__r2n, self.__n2r):
            self.initialize_tf(tf)

        # Initialize dependencies
        self.__nestAdapter.initialize()
        self.__robotAdapter.initialize(name)
        self.__initialized = True

    def __reset_tf(self, tf):
        """
        Resets the given transfer function
        """
        tf.elapsed_time = 0.0
        tf.check_params()
        if hasattr(tf, "topic") and tf.topic is not None:
            tf.topic = tf.topic.reset(self)
        for i in range(1, len(tf.params)):
            param = tf.params[i]
            reset_value = param.reset(self)
            if param is not reset_value:
                tf.params[i] = reset_value
                for k in tf.__dict__:
                    if tf.__dict__[k] is param:
                        tf.__dict__[k] = reset_value
                reset_value.spec = param.spec
        self._update_trigger(tf)

    def reset(self):  # -> None:
        """
        Resets the transfer functions
        """
        logger.info("Resetting transfer functions")

        # Wire transfer functions from neuronal simulation to world simulation
        for tf in itertools.chain(self.__r2n, self.__n2r):
            self.__reset_tf(tf)

    def shutdown(self):
        """
        Shuts down the Transfer Function manager
        """
        del self.__n2r[:]
        del self.__r2n[:]
        del self.__flawed[:]
        del self.__silent[:]
        self.__global_data.clear()
        self.__initialized = False

    def hard_reset_brain_devices(self):
        """
        Performs a hard reset for the devices that connect with the neuronal simulation
        """
        if not self.initialized:
            return

        self.brain_adapter.shutdown()
        self.brain_adapter.initialize()

        exceptions_found = []
        for tf in itertools.chain(self.__r2n, self.__n2r, self.__silent):
            if not tf.active:
                continue
            tf_exception = None
            for i in range(1, len(tf.params)):
                try:
                    spec = tf.params[i].spec
                    if spec.is_brain_connection:
                        tf.params[i] = spec.create_adapter(self)
                        tf.params[i].spec = spec
                        tf.__dict__[spec.name] = tf.params[i]
                # pylint: disable=broad-except
                except Exception as e:
                    logger.exception(e)
                    tf_exception = BrainParameterException("Cannot map parameter '{0}' in transfer "
                                                           "function '{1}'"
                                                           .format(spec.name, tf.name))
                    break

            if tf_exception is not None:
                self.activate_tf(tf, False)
                exceptions_found.append(tf_exception)
            else:
                tf.initialize(self, True, False)
                self._update_trigger(tf)

        if exceptions_found:
            raise Exception(exceptions_found)

    def hard_reset_robot_devices(self):
        """
        Performs a hard reset for the devices that connect with the simulated robot
        """
        if not self.initialized:
            return

        self.robot_adapter.shutdown()
        self.robot_adapter.initialize()

        for tf in itertools.chain(self.__r2n, self.__n2r, self.__silent):
            for i in range(1, len(tf.params)):
                spec = tf.params[i].spec
                if spec.is_robot_connection:
                    tf.params[i] = spec.create_adapter(self)
                    tf.params[i].spec = spec
                    tf.__dict__[spec.name] = tf.params[i]
            tf.initialize(self, False, True)
            self._update_trigger(tf)
