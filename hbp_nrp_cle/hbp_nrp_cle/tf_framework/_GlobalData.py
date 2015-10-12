"""
This module contains the mapping of transfer function local or global variables to input parameters
"""
__author__ = 'sebastiankrach'

from ._TransferFunction import TransferFunction

import abc
import logging

logger = logging.getLogger(__name__)

GLOBAL = "global_scope"
TRANSFER_FUNCTION_LOCAL = "transfer_function_scope"


class MapVariable(object):
    """
    Class to map transfer function local or global variables to transfer function parameters
    """

    def __init__(self, parameter_name, global_key=None, initial_value=None,
                 scope=TRANSFER_FUNCTION_LOCAL, **kwargs):  # -> None:
        """
        Maps a parameter to a variable in the specified scope (per-default: the transfer function)
        and if the variable does not yet exist initializes it with the provided value.

        :param parameter_name: the name of the parameter
        :param global_key: (Optional) the name of the variable in the global scope, if not specified
        it is assumed to be equal to the parameter name
        :param initial_value: (Optional) the value for the parameter
        :param scope: (Optional) the scope of validity for the variable
        :param kwargs: Additional configuration parameters
        """
        self.__parameter_name = parameter_name
        if global_key:
            self.__global_key = global_key
        else:
            self.__global_key = parameter_name
        self.__initial_value = initial_value
        self.__scope = scope

        self.__config = kwargs

        self.__tf = None

    def __call__(self, transfer_function):  # -> Robot2Neuron:
        """
        Applies the parameter mapping to the given transfer function
        """
        assert isinstance(transfer_function, TransferFunction)
        self.__tf = transfer_function

        topics = transfer_function.params
        for i in range(0, len(topics)):
            if topics[i] == self.__parameter_name:
                topics[i] = self
                return transfer_function
        raise Exception("Could not map parameter as no parameter with the given name exists")

    @property
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__parameter_name

    def create_adapter(self, transfer_function_manager):
        """
        Replaces the current mapping operator with the mapping result
        """
        if self.__scope == GLOBAL:
            return GlobalDataReference(self.__parameter_name, self.__global_key,
                                       self.__initial_value, transfer_function_manager.global_data)
        elif self.__scope == TRANSFER_FUNCTION_LOCAL:
            return LocalDataReference(self.__parameter_name, self.__initial_value)
        else:
            raise AttributeError("The specified parameter scope is not valid.")


class DataReference(object):
    """
    Abstract super class for transfer function variables. Must be implemented by subclasses which
    support storing and providing of parameters to transfer functions.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name, initial_value):
        self.__name = name
        self.__initial_value = initial_value

    @property
    def name(self):
        """
        Gets the name of the mapped parameter
        """
        return self.__name

    @property
    def value(self):
        """
        Gets the value associated with this parameter. Has to be provided by the implementing
        subclass.

        :return: the parameter value
        """
        return self._value_getter()

    @value.setter
    def value(self, _val):
        """
        Sets the value associated with this parameter. Has to be provided by the implementing
        subclass.

        :param _val: The new value
        """
        self._value_setter(_val)

    @abc.abstractmethod
    def _value_getter(self):
        """
        Gets the value associated with this parameter. Abstract method to force subclasses to
        provide functionality for value property.

        :return: The value.
        """
        raise Exception("Must be overwritten by instantiating subclass.")

    @abc.abstractmethod
    def _value_setter(self, _val):
        """
        Sets the value associated with this parameter. Abstract method to force subclasses to
        provide functionality for value property.

        :param _val: The new value
        """
        raise Exception("Must be overwritten by instantiating subclass.")

    # pylint: disable=unused-argument
    def reset(self, transfer_function_manager):
        """
        Resets the GlobalDataReference instance to its initial value.

        :param transfer_function_manager: The transfer function manager the tf is contained in
        :return: The reset instance
        """
        self.value = self.__initial_value
        return self


class LocalDataReference(DataReference):
    """
    This class acts as concrete mapping between an input parameter to a transfer function and the
    parameter value stored locally for one transfer function.
    """

    def __init__(self, name, initial_value):
        super(LocalDataReference, self).__init__(name, initial_value)
        self.__value = initial_value

    def _value_getter(self):
        """
        Gets the value associated with this parameter
        """
        return self.__value

    def _value_setter(self, _val):
        """
        Sets the value associated with this parameter

        :param _val: the new value
        """
        self.__value = _val


class GlobalDataReference(DataReference):
    """
    This class acts as concrete mapping between an input parameter to a transfer function and the
    dictionary storing the actual value.
    """

    def __init__(self, name, global_key, initial_value, data_dictionary):
        """
        Creates a new parameter mapping.

        :param name: the name of the parameter
        :param data_dictionary: the dictionary where to look up the value and store it to
        """

        super(GlobalDataReference, self).__init__(name, initial_value)
        self.__global_key = global_key
        self.__data_dict = data_dictionary

        if self.__global_key not in self.__data_dict:
            self.reset(None)

    def _value_getter(self):
        """
        Gets the value associated with this parameter
        """
        return self.__data_dict[self.__global_key]

    def _value_setter(self, _val):
        """
        Sets the value associated with this parameter

        :param _val: the new value
        """
        self.__data_dict[self.__global_key] = _val