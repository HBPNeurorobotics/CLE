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
Support methods to load a brain network from a python file
"""

__author__ = "Lorenzo Vannucci, Sebastian Krach"

from hbp_nrp_cle.cle.CLEInterface import BrainTimeoutException

import imp
import logging


logger = logging.getLogger("BrainLoader")
__brain_index = 0


def check_import_brain(path, is_completed):
    """
    it tries to import the Brain in order to check if the Brain contains errors.

    :param path: path to the .py file
    :param is_completed: it indicates that the brain was imported
    """
    try:
        logger.info("Loading brain model from python: " + path)
        imp.load_source(
            '__brain_model' + str(__brain_index), path)
    except SyntaxError:
        logger.info("Loading brain model error from python: " + path)
    finally:
        is_completed.value = True


def is_brain_safely_imported(path):
    """
    it checks if the brain can safely imported

    :param path: path to the .py file
    """
    from multiprocessing import Process
    from multiprocessing.sharedctypes import Value
    import time
    is_completed = Value('b', False)
    brain_import_proc = Process(
        target=check_import_brain, args=(path, is_completed))
    brain_import_proc.start()

    timeout = time.time() + 60 * 2  # 2 minutes from now
    while not is_completed.value:
        if time.time() > timeout:
            brain_import_proc.terminate()
            raise BrainTimeoutException()
        time.sleep(1)
    return True


# pylint: disable=global-statement
def load_py_network(path):
    """
    Load a python network file

    :param path: path to the .py file
    """
    global __brain_index
    brain_module = imp.load_source('__brain_model' + str(__brain_index), path)
    __brain_index += 1
    return brain_module


def clear_populations(brain_module):
    """
    clear populations to the population

    :param brain_module: The brain module
    """
    try:
        for population_id in brain_module.populations_keys:
            del brain_module.__dict__[population_id]
        brain_module.populations_keys = []
    except KeyError as e:
        logger.info("Error while clearing the populations: " + e.__repr__())


def setup_access_to_population(brain_module, **populations):
    """
    Sets up the access to the population

    :param brain_module: The brain module
    :param populations: A dictionary of the populations and their ids
    """
    try:
        circuit = brain_module.circuit
        logger.debug("Found circuit")
        for p in populations:
            population = populations[p]
            neurons = circuit[population]
            logger.debug("Population '%s': %s", p, neurons)
            if isinstance(population, slice):
                expected_size = abs(
                    (population.start - population.stop) / (population.step or 1))
                if neurons.size != expected_size:
                    raise Exception("Population '%s' out of bounds" % p)
            brain_module.__dict__[p] = neurons
            brain_module.populations_keys.append(p)
    except AttributeError:
        logger.exception("Could not initialize brain simulation")
        if len(populations) > 0:
            raise Exception(
                "Could not initialize populations, no circuit found")
