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

import dimod

from dwave.preprocessing.roof_duality import RoofDualityComposite
from dwave.preprocessing.lower_bounds import roof_duality

class TestFixVariables(unittest.TestCase):
    def test_empty(self):
        bqm = dimod.AdjVectorBQM('BINARY')
        fixed = roof_duality(bqm, strict=True)
        self.assertEqual(fixed, {})

        fixed = roof_duality(bqm, strict=False)
        self.assertEqual(fixed, {})

    def test_all_zero(self):
        num_vars = 3

        bqm = dimod.AdjVectorBQM(num_vars, 'BINARY')
        fixed = roof_duality(bqm, strict=True)
        self.assertEqual(fixed, {})

        fixed = roof_duality(bqm, strict=False)
        self.assertEqual(len(fixed), num_vars)
        for val in fixed.values():
            self.assertEqual(val, 1)

    def test_3path(self):
        bqm = dimod.BinaryQuadraticModel.from_ising({'a': 10}, {'ab': -1, 'bc': 1})
        fixed = roof_duality(bqm)
        self.assertEqual(fixed, {'a': -1, 'b': -1, 'c': 1})

@dimod.testing.load_sampler_bqm_tests(RoofDualityComposite(dimod.ExactSolver()))
@dimod.testing.load_sampler_bqm_tests(RoofDualityComposite(dimod.NullSampler()))
class TestRoofDualityComposite(unittest.TestCase):
    def test_construction(self):
        sampler = RoofDualityComposite(dimod.ExactSolver())
        dimod.testing.assert_sampler_api(sampler)

    def test_3path(self):
        sampler = RoofDualityComposite(dimod.ExactSolver())
        sampleset = sampler.sample_ising({'a': 10},  {'ab': -1, 'bc': 1})

        # all should be fixed, so should just see one
        self.assertEqual(len(sampleset), 1)
        self.assertEqual(set(sampleset.variables), set('abc'))

    def test_triangle(self):
        sampler = RoofDualityComposite(dimod.ExactSolver())

        bqm = dimod.BinaryQuadraticModel.from_ising({}, {'ab': -1, 'bc': -1, 'ac': -1})

        # two equally good solutions
        sampleset = sampler.sample(bqm)

        self.assertEqual(set(sampleset.variables), set('abc'))
        dimod.testing.assert_response_energies(sampleset, bqm)

    def test_triangle_not_strict(self):
        sampler = RoofDualityComposite(dimod.ExactSolver())

        bqm = dimod.BinaryQuadraticModel.from_ising({}, {'ab': -1, 'bc': -1, 'ac': -1})

        # two equally good solutions, but with strict=False, it will pick one
        sampleset = sampler.sample(bqm, strict=False)

        self.assertEqual(set(sampleset.variables), set('abc'))
        self.assertEqual(len(sampleset), 1)  # all should be fixed
        dimod.testing.assert_response_energies(sampleset, bqm)
