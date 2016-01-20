'''
This module contains mock implementations for alternating and direct current sources which
record the changes to the device parameters and can be used to track transfer function activity.
'''

from .MockAbstractBrainDevice import AbstractMockBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IACSource, IDCSource

__author__ = 'MichaelWeber'


class MockCurrentSource(AbstractMockBrainDevice):
    """
    Represents an alternating current generator.
    """
    default_parameters = {
        "amplitude": 1.0
    }

    # pylint: disable = W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param amplitude: Amplitude of the current, default: 1.0 nA
        """
        super(MockCurrentSource, self).__init__(**params)

        self.__amplitude = self._parameters["amplitude"]
        self.__history = []

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self.__amplitude

    @amplitude.setter
    def amplitude(self, amplitude):
        """
        Sets the amplitude of the current

        :param amplitude: float
        """
        self.__amplitude = amplitude
        self.__history.append(amplitude)

    @property
    def history(self):
        """
        Lists the amplitudes assigned to this device

        :return: A list of float values
        """
        return self.__history


class MockACSource(MockCurrentSource, IACSource):
    """
    The mock device for an alternating current source
    """


class MockDCSource(MockCurrentSource, IDCSource):
    """
    The mock device ffor an direct current source
    """