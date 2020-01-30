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
This tests the ability of the PyNNControlAdapter to correctly load neuron populations
"""

__author__ = 'Georg Hinkel'

import unittest
import os
from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter
from hbp_nrp_cle.cle.CLEInterface import BrainRuntimeException
import hbp_nrp_cle.brainsim.config as brainconfig
from mock import patch, Mock


class TestPyNNControlAdapter(unittest.TestCase):

    def setUp(self):
        brainconfig.rng_seed = 123456

    def test_load_brain_with_empty_brainconfig(self):
        sim = Mock()
        adapter = PyNNControlAdapter(sim)
        directory = os.path.dirname(__file__)
        brain_path = os.path.join(directory, "DummyBrainModel.py")

        brainconfig.brain_populations = {}
        adapter.load_brain(brain_path)

        populations = adapter.get_populations()
        circuit = next(p for p in populations if p.name == "circuit")
        self.assertEqual("EIF_cond_alpha_isfa_ista", circuit.celltype)

    def test_load_brain_with_slices(self):
        sim = Mock()
        adapter = PyNNControlAdapter(sim)
        directory = os.path.dirname(__file__)
        brain_path = os.path.join(directory, "DummyBrainModel.py")
        population_slices = {'testPopulation1': slice(0, 1, 1),
                             'testPopulation2': slice(1, 2, 1)}
        adapter.load_brain(brain_path, **population_slices)
        populations = adapter.get_populations()
        population1 = next(p for p in populations if p.name == "testPopulation1")
        population2 = next(p for p in populations if p.name == "testPopulation2")
        circuit = next(p for p in populations if p.name == "circuit")
        self.assertEqual("EIF_cond_alpha_isfa_ista", population1.celltype)
        self.assertEqual("EIF_cond_alpha_isfa_ista", population2.celltype)
        self.assertEqual("EIF_cond_alpha_isfa_ista", circuit.celltype)

    def test_invalid_file_extension(self):
        sim = Mock()
        adapter = PyNNControlAdapter(sim)
        directory = os.path.dirname(__file__)
        with self.assertRaises(Exception):
            adapter.load_brain(os.path.join(directory, "DummyBrainModelWithWrongFiletype.py.txt"))

    def test_populations(self):
        sim = Mock()
        adapter = PyNNControlAdapter(sim)
        directory = os.path.dirname(__file__)
        adapter.is_brain_safely_imported = Mock(return_value=True)
        adapter.load_brain(os.path.join(directory, "example_pynn_brain.py"))
        populations = adapter.get_populations()
        self.assertEqual(6, len(populations))
        # Populations may be in any order, so we have to crawl the collection
        population = next(p for p in populations if p.name == "population")
        view = next(p for p in populations if p.name == "view")
        assembly = next(p for p in populations if p.name == "assembly")
        list_0 = next(p for p in populations if p.name == "list[0]")
        list_1 = next(p for p in populations if p.name == "list[1]")
        list_2 = next(p for p in populations if p.name == "list[2][0]")
        # check population
        self.assertEqual("population", population.name)
        self.assertEqual("IF_curr_alpha", population.celltype)
        self.assertNotEqual(0, len(population.parameters))
        self.assertEqual(3, len(population.gids))
        # check view
        self.assertEqual("view", view.name)
        self.assertEqual("IF_curr_alpha", view.celltype)
        self.assertNotEqual(0, len(view.parameters))
        self.assertEqual(1, len(view.gids))
        # check assembly
        self.assertEqual("assembly", assembly.name)
        self.assertEqual("PopulationAssembly", assembly.celltype)
        self.assertEqual(0, len(assembly.parameters))
        self.assertEqual(3, len(assembly.gids))
        # check population
        self.assertEqual("IF_curr_alpha", list_0.celltype)
        self.assertNotEqual(0, len(list_0.parameters))
        self.assertEqual(3, len(list_0.gids))
        # check view
        self.assertEqual("IF_curr_alpha", list_1.celltype)
        self.assertNotEqual(0, len(list_1.parameters))
        self.assertEqual(1, len(list_1.gids))
        # check assembly
        self.assertEqual("PopulationAssembly", list_2.celltype)
        self.assertEqual(0, len(list_2.parameters))
        self.assertEqual(3, len(list_2.gids))

    def test_exception_is_raised(self):
        sim = Mock()
        adapter = PyNNControlAdapter(sim)
        sim.run.side_effect = Exception("error")
        with self.assertRaises(BrainRuntimeException):
            adapter.run_step(1)
