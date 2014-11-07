"""
brain_loader unit test
"""

from python_cle.brainsim import BrainLoader

import unittest

__author__ = 'Lorenzo Vannucci'


# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase
class TestClosedLoopEngine(unittest.TestCase):
    """
    Tests the brain_loader utility
    """

    # pylint: disable=R0201
    # method has to be a method in order to be run as part of the test suite
    def test_load_h5_network(self):
        """
        Test loading an .h5 file.
        """
        filename = 'python_cle/python_cle/tests/brainsim/braitenberg.h5'
        BrainLoader.load_h5_network(filename, [0, 1, 2], [3, 4, 5])


if __name__ == '__main__':
    unittest.main()
