# Copyright 2021 D-Wave Systems Inc.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#
# =======================================

import unittest

import dimod

from dwave.preprocessing.roof_duality import fix_variables, RoofDualityComposite

class TestFixVariables(unittest.TestCase):
    def test_3path(self):
        bqm = dimod.BinaryQuadraticModel.from_ising({'a': 10}, {'ab': -1, 'bc': 1})
        fixed = fix_variables(bqm)
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

    def test_triangle_sampling_mode_off(self):
        sampler = RoofDualityComposite(dimod.ExactSolver())

        bqm = dimod.BinaryQuadraticModel.from_ising({}, {'ab': -1, 'bc': -1, 'ac': -1})

        # two equally good solutions, but with sampling mode off it will pick one
        sampleset = sampler.sample(bqm, sampling_mode=False)

        self.assertEqual(set(sampleset.variables), set('abc'))
        self.assertEqual(len(sampleset), 1)  # all should be fixed
        dimod.testing.assert_response_energies(sampleset, bqm)
