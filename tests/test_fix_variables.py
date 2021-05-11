# Copyright 2021 D-Wave Systems Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import dimod.testing as dtest
from dimod.vartypes import Vartype
from dimod import BinaryQuadraticModel
from dimod import ExactSolver, NullSampler
from dimod import SampleSet

from dwave.preprocessing.composites import FixVariablesComposite


@dtest.load_sampler_bqm_tests(FixVariablesComposite(ExactSolver()))
@dtest.load_sampler_bqm_tests(FixVariablesComposite(NullSampler()))
class TestFixVariablesComposite(unittest.TestCase):
    def test_instantiation_smoke(self):
        sampler = FixVariablesComposite(ExactSolver())
        dtest.assert_sampler_api(sampler)

    def test_invalid_algorithm(self):
        with self.assertRaises(ValueError):
            sampler = FixVariablesComposite(ExactSolver(), algorithm="abc")

    def test_sample(self):
        bqm = BinaryQuadraticModel({1: -1.3, 4: -0.5},
                                   {(1, 4): -0.6},
                                   0,
                                   vartype=Vartype.SPIN)

        fixed_variables = {1: -1}
        sampler = FixVariablesComposite(ExactSolver())
        response = sampler.sample(bqm, fixed_variables=fixed_variables)

        self.assertEqual(response.first.sample, {4: -1, 1: -1})
        self.assertAlmostEqual(response.first.energy, 1.2)

    def test_empty_bqm(self):
        bqm = BinaryQuadraticModel({1: -1.3, 4: -0.5},
                                   {(1, 4): -0.6},
                                   0,
                                   vartype=Vartype.SPIN)

        fixed_variables = {1: -1, 4: -1}
        sampler = FixVariablesComposite(ExactSolver())
        response = sampler.sample(bqm, fixed_variables=fixed_variables)
        self.assertIsInstance(response, SampleSet)

    def test_empty_fix(self):
        linear = {1: -1.3, 4: -0.5}
        quadratic = {(1, 4): -0.6}

        sampler = FixVariablesComposite(ExactSolver())
        response = sampler.sample_ising(linear, quadratic)
        self.assertIsInstance(response, SampleSet)

        self.assertEqual(response.first.sample, {4: 1, 1: 1})
        self.assertAlmostEqual(response.first.energy, -2.4)

    def test_roof_duality_3path(self):
        sampler = FixVariablesComposite(ExactSolver(), algorithm='roof_duality')
        sampleset = sampler.sample_ising({'a': 10},  {'ab': -1, 'bc': 1})

        # all should be fixed, so should just see one
        self.assertEqual(len(sampleset), 1)
        self.assertEqual(set(sampleset.variables), set('abc'))

    def test_roof_duality_triangle(self):
        bqm = BinaryQuadraticModel.from_ising({}, {'ab': -1, 'bc': -1, 'ac': -1})

        # two equally good solutions
        sampler = FixVariablesComposite(ExactSolver(), algorithm='roof_duality')
        sampleset = sampler.sample(bqm)

        self.assertEqual(set(sampleset.variables), set('abc'))
        dtest.assert_response_energies(sampleset, bqm)

    def test_roof_duality_triangle_not_strict(self):
        bqm = BinaryQuadraticModel.from_ising({}, {'ab': -1, 'bc': -1, 'ac': -1})

        # two equally good solutions, but with strict=False, it will pick one
        sampler = FixVariablesComposite(ExactSolver(), algorithm='roof_duality')

        sampleset = sampler.sample(bqm, strict=False)

        self.assertEqual(set(sampleset.variables), set('abc'))
        self.assertEqual(len(sampleset), 1)  # all should be fixed
        dtest.assert_response_energies(sampleset, bqm)
