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
Ethernet control connection handling for live interactions between Spinnaker and external devices
"""
from hbp_nrp_cle.brainsim.pynn_spiNNaker import spynnaker as sim
try:
    # pylint: disable=import-error, no-name-in-module
    from spynnaker.pyNN.external_devices_models import AbstractEthernetTranslator
except ImportError:  # pragma: no cover
    AbstractEthernetTranslator = object
import logging
logger = logging.getLogger(__name__)


class Translator(AbstractEthernetTranslator):
    """
    Forwards multicast commands based on
    """
    def __init__(self):
        """
        Creates a new, empty translator
        """
        self.__device_dict = {}

    def reset(self):
        """
        Resets the translator
        """
        self.__device_dict.clear()

    def register_translation(self, device):
        """
        Registers the given device to receive multicast commands

        :param device: The device to register
        """
        self.__device_dict[device.device_control_key] = device

    def translate_control_packet(self, multicast_packet):
        """
        Handles an incoming multicast command

        :param multicast_packet: The multicast command packet
        """
        logger.debug("Received packet {key}: {val}".format(
            key=multicast_packet.key, val=multicast_packet.payload))

        try:
            integrator = self.__device_dict[multicast_packet.key]
            integrator.run(multicast_packet.payload)
        except KeyError:
            logger.warn("Multicast packet with unknown key {0} received"
                        .format(multicast_packet.key))


__next_key = 1
translator = Translator()


def get_key(key):
    """
    Generates a key based on the given key proposal

    :param key: A proposed key or None
    """
    # pylint: disable=global-statement
    global __next_key
    if key is None or key < __next_key:
        key = __next_key
    __next_key = key + 1
    return key


def reset():
    """
    Resets the ethernet connection
    """
    # pylint: disable=global-statement
    global __next_key

    __next_key = 1
    translator.reset()


def shutdown():
    """
    Shuts down the ethernet control connection, if any
    :return:
    """
    pass


def register_devices(devices, **params):
    """
    Registers the given multicast devices using external device LIF control nodes

    :param devices: A list of devices
    :param params: configuration parameters for the external LIF control population
    :return: The generated LIF control population
    """
    # pylint: disable=protected-access
    # pylint: disable=global-statement
    model = sim.external_devices.ExternalDeviceLifControl(
        devices,
        False,
        translator,
        **params
    )
    population = sim.external_devices.EthernetControlPopulation(
        len(devices), model)
    for dev in devices:
        translator.register_translation(dev)
    return population
