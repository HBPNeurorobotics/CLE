"""
This package contains the classes to specify transfer functions and connect them to communication
adapters of both the neuronal simulator and the world simulator
"""

from hbp_nrp_excontrol.restricted_python import _inplacevar_
from hbp_nrp_cle.tf_framework._TfApi import TfApi

TF_API = TfApi()


class UserCodeException(Exception):
    """
    General exception class returning a meaningful message
    to the ExD frontend when user code fails to be loaded or run.

    :param message: message that needs to be forwarded to the frontend.
    :param error_type: Type of error (like 'CLE Error')
    """

    def __init__(self, message, error_type):
        super(UserCodeException, self).__init__(message)
        self.error_type = error_type

    def __str__(self):
        return "{0} ({1})".format(repr(self.message), self.error_type)


class BrainParameterException(Exception):
    """
    Exception raised when a brain connection parameter fails to create the underlying adapter

    :param source: the exception source
    :param message: the brain
    """

    def __init__(self, message):
        super(BrainParameterException, self).__init__(message)


class TFException(UserCodeException):
    """
    Exception class used to return a meaningful message
    to ExD front-end in case the update of TF's user code
    fails.

    :param tf_name: name of the TF updated by the user.
    :param message: message that needs to be forwarded to the front-end.
    """

    def __init__(self, tf_name, message, error_type):
        super(TFException, self).__init__(message, error_type)
        self.tf_name = tf_name

    def __str__(self):
        return "{0}: {1} ({2})".format(self.tf_name, repr(self.message), self.error_type)


class TFRunningException(UserCodeException):
    """
    Exception class used to communicate the TF with the TransferFunction manager

    :param tf_name: name of the TF updated by the user.
    :param message: message that needs to be forwarded to the front-end.
    """

    def __init__(self, message):
        super(TFRunningException, self).__init__(
            message, 'TF Running Exception')


class TFLoadingException(TFException):
    """
    Exception class used to return a meaningful message
    to ExD front-end in case the loading of a TF with updated user code
    fails.

    :param tf_name: name of the TF updated by the user.
    :param message: message that needs to be forwarded to the front-end.
    """

    def __init__(self, tf_name, message):
        super(TFLoadingException, self).__init__(
            tf_name, message, 'TF Loading Exception')


from . import config
from ._PropertyPath import PropertyPath, RangeSegment, CustomSegment


def resolve_brain_variable(var):
    """
    Resolves the given brain variable for the current brain

    :param var: The brain variable
    :return: If the variable does not depend on the neural network, it is returned unchanged.
    Otherwise, it is resolved for the current neural network
    """
    if isinstance(var, PropertyPath):
        return var.select(config.brain_root, config.active_node.brain_adapter)
    return var


from hbp_nrp_cle.brainsim.BrainInterface import IFixedSpikeGenerator, \
    ILeakyIntegratorAlpha, ILeakyIntegratorExp, IPoissonSpikeGenerator, IRawSignal, \
    IDCSource, IACSource, INCSource, IPopulationRate, ISpikeRecorder, ISpikeInjector

import logging

logger = logging.getLogger(__name__)

# alias _Facade module needed by set_transfer_function
import sys

nrp = sys.modules[__name__]

# CLE restricted python environment
# TODO(Luc): create a module for it
from RestrictedPython.PrintCollector import PrintCollector
from operator import getitem

# pylint: disable=unused-import
from ._Neuron2Robot import Neuron2Robot, MapSpikeSink, MapSpikeSource
from ._Robot2Neuron import Robot2Neuron, MapRobotPublisher, \
    MapRobotSubscriber
from hbp_nrp_cle.tf_framework._TransferFunction import TransferFunction, FlawedTransferFunction
from hbp_nrp_cle.tf_framework._CSVRecorder import MapCSVRecorder, CSVRecorder
from hbp_nrp_cle.tf_framework._NeuronMonitor import NeuronMonitor
from hbp_nrp_cle.tf_framework._GlobalData import MapVariable, GLOBAL, TRANSFER_FUNCTION_LOCAL
from hbp_nrp_cle.tf_framework._CleanableTransferFunctionParameter \
    import ICleanableTransferFunctionParameter
from . import _TransferFunctionManager, _NeuronSelectors
from ._TransferFunctionInterface import ITransferFunctionManager
from hbp_nrp_cle.robotsim.RobotInterface import Topic, IRobotCommunicationAdapter
import std_msgs.msg
import cle_ros_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import gazebo_msgs.msg
import dynamic_reconfigure.msg
import tf2_msgs.msg
import hbp_nrp_cle.tf_framework.tf_lib
from hbp_nrp_excontrol.logs import clientLogger
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Float32, Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np

__author__ = 'Georg Hinkel'

_getattr_ = getattr
_getitem_ = getitem
_getiter_ = iter
_print_ = PrintCollector


def _cle_write_guard():
    """
    Defines the write guard for execution of user code in restricted mode.
    """

    def guard(ob):
        """
        No guard at all
        """
        return ob

    return guard


cle_write_guard = _cle_write_guard()
_write_ = cle_write_guard

leaky_integrator_alpha = ILeakyIntegratorAlpha
leaky_integrator_exp = ILeakyIntegratorExp
fixed_frequency = IFixedSpikeGenerator
poisson = IPoissonSpikeGenerator
dc_source = IDCSource
ac_source = IACSource
nc_source = INCSource
population_rate = IPopulationRate
spike_recorder = ISpikeRecorder
injector = ISpikeInjector
raw_signal = IRawSignal

brain = PropertyPath()


def nrange(start, stop, step=None):
    """
    Defines a range of neurons

    :param start: The start of the range
    :param stop: The stop of the range
    :param step: The step of the range
    """
    return RangeSegment(start, stop, step)


def resolve(fun):
    """
    Resolves the given function when the neural network is available

    :param fun: The function that selects the item from the network
    """
    return CustomSegment(fun)


def map_neurons(neuron_range, mapping):
    """
    Maps the given range to neurons using the provided mapping

    :param neuron_range: A range that can be iterated
    :param mapping: A mapping function or lambda
    """
    return _NeuronSelectors.MapNeuronSelector(neuron_range, mapping)


def chain_neurons(*neuron_selectors):
    """
    Chains the given neuron selectors

    :param neuron_selectors: The neuron selectors
    """
    return _NeuronSelectors.ChainNeuronSelector(neuron_selectors)


def initialize(name):  # -> None:
    """
    Initializes and starts the TF node

    :param name: The name of the TF node
    """
    config.active_node.initialize(name)


def set_nest_adapter(nest_adapter):  # -> None:
    """
    Sets the brainsim adapter.

    :param nest_adapter: The brainsim adapter

    .. WARNING:: Must be executed before tf node initialization
    """
    config.active_node.brain_adapter = nest_adapter


def set_robot_adapter(robot_adapter):  # -> None:
    """
    Sets the robot adapter.

    :param robot_adapter: The robot adapter

    .. WARNING:: Must be executed before tf node initialization
    """
    config.active_node.robot_adapter = robot_adapter


def start_new_tf_manager():
    """
    Start a new transfer function manager
    """
    config.active_node = _TransferFunctionManager.TransferFunctionManager()
    config.csv_recorders = []


def get_transfer_functions(flawed=True):
    """
    Get all the transfer functions if flawed is True, only (R2N, N2R, Silent) otherwise

    :return: All the transfer functions if flawed is True, only (R2N, N2R, Silent) otherwise.
    """

    proper_tfs = config.active_node.n2r + \
        config.active_node.r2n + config.active_node.silent

    return proper_tfs + config.active_node.flawed if flawed else proper_tfs


def get_flawed_transfer_function(name):
    """
    Get the flawed transfer function with the given name

    :param name: The name of the flawed transfer function
    :return: The flawed transfer function with the given name
    """
    return next((f_tf for f_tf in config.active_node.flawed if f_tf.name == name), None)


def get_transfer_function(name):
    """
    Get the transfer function with the given name

    :param name: The name of the transfer function
    :return: The transfer function with the given name
    """

    return next((tf for tf in get_transfer_functions() if tf.name == name), None)


def activate_transfer_function(tf, activate):
    """
    Set the activation state of the transfer function
    In case of errors the change is not applied.

    :param tf: the tf to (de-)activate
    :param activate: a boolean value denoting the new activation state
    """
    config.active_node.activate_tf(tf, activate)


def get_brain_source():
    """
    Get the source of the brain (if loaded from a python file). Otherwise, returns
    None.

    :return: The source of the brain model
    """
    return config.brain_source


def get_brain_populations():
    """
    Get the brain populations as a dictionary
    If the brain model is not loaded,
    the function returns None.

    :return: A dictionary containing the brain populations
        The dictionary keys are population names
        and its values are one of the following types:
        list, or a 'slice' dictionary of the following form
        {'from': 1, 'to': 10, 'step': 1}.
    """
    return config.brain_populations


def dump_csv_recorder_to_files():
    """
    Find out all CSV recorders and dump their values to CSV files.

    :return: an array containing a string with the CSV filename,
    an array containing the CSV headers separated by a comma
    and an array containing the CSV values
    """
    result = []
    for tf in get_transfer_functions(flawed=False):
        for param in tf.params[1:]:
            if isinstance(param, CSVRecorder):
                name = param.get_csv_recorder_name()
                headers = param.get_csv_headers()
                values = param.cleanup()
                result.append([name, headers, values])
    return result


def clean_csv_recorders_files():
    """
    Clean out all CSV recorders generated files.
    """
    for tf in get_transfer_functions(flawed=False):
        for param in tf.params[1:]:
            if isinstance(param, CSVRecorder):
                param.cleanup()


def delete_transfer_function(name):
    """
    Delete a transfer function. If the transfer function does not exist,
    nothing will happen.

    :param name: The name of the transfer function
    :return: True if the transfer function is correctly deleted. False if the transfer function
             does not exist.
    """
    tf = get_transfer_function(name)
    is_flawed_deleted = False
    if delete_flawed_transfer_function(name):
        is_flawed_deleted = True

    if tf in config.active_node.n2r:
        config.active_node.n2r.remove(tf)
    elif tf in config.active_node.r2n:
        config.active_node.r2n.remove(tf)
    elif tf in config.active_node.silent:
        config.active_node.silent.remove(tf)

    else:
        return is_flawed_deleted

    tf.unregister()

    brain_adapter = config.active_node.brain_adapter
    robot_adapter = config.active_node.robot_adapter

    for i in range(1, len(tf.params)):
        if tf.params[i] in brain_adapter.detector_devices:
            brain_adapter.unregister_spike_sink(tf.params[i])
        elif tf.params[i] in robot_adapter.published_topics:
            robot_adapter.unregister_publish_topic(tf.params[i])
        elif tf.params[i] in brain_adapter.generator_devices:
            brain_adapter.unregister_spike_source(tf.params[i])
        elif tf.params[i] in robot_adapter.subscribed_topics:
            robot_adapter.unregister_subscribe_topic(tf.params[i])
        if isinstance(tf.params[i], ICleanableTransferFunctionParameter):
            tf.params[i].cleanup()

    return True


def delete_flawed_transfer_function(name):
    """
    Delete a flawed transfer function. If the transfer function does not exist,
    nothing will happen.

    :param name: The name of the transfer function
    :return: True if the transfer function is correctly deleted. False if the transfer function
             does not exist.
    """
    result = True

    tf = get_flawed_transfer_function(name)

    if tf:
        config.active_node.flawed.remove(tf)
    else:
        result = False

    return result


def set_transfer_function(new_source, new_code, new_name, activation=True, priority=None):
    """
    Apply transfer function changes made by a client

    :param new_source: Transfer function's updated source
    :param new_code: Compiled code of the updated source
    :param new_name: Transfer function's updated name
    :param activation: Activation state of the transfer function
    :param priority: execution order of the transfer function. Transfer functions with higher
    priority are executed first.
    """

    # pylint: disable=broad-except
    try:
        # pylint: disable=exec-used
        exec new_code
        tf = get_transfer_function(new_name)
        if not isinstance(tf, TransferFunction):
            raise Exception("Transfer function has no decorator specifying its type")
        config.active_node.initialize_tf(tf, activation)
    except Exception as e:
        tb = sys.exc_info()[2]
        logger.error("Error while loading new transfer function")
        logger.exception(e)
        delete_transfer_function(new_name)
        raise TFLoadingException(new_name, str(e)), None, tb

    # we set the new source in an attribute because inspect.getsource won't work after exec
    # indeed inspect.getsource is based on a source file object
    # see findsource in http://www.opensource.apple.com/source/python/python-3/python/Lib/inspect.py
    tf.source = new_source
    if priority is not None:
        tf.priority = priority


def set_flawed_transfer_function(source, name="NO_NAME", error=None):
    """
    Creates a new flawed transfer function,
    i.e. a TF that is not valid due to some error in the source code.
    The user will correct it at a later stage (e.g. during an experiment)

    :param source: the source code
    :param name: The name of the transfer function
    :param error: the Exception raised during the compilation/loading of the code
    """
    config.active_node.flawed.append(
        FlawedTransferFunction(name, source, error))


start_new_tf_manager()
