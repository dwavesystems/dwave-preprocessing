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

from dimod import ExactSolver, RandomSampler, SimulatedAnnealingSampler
import dimod.testing as dtest

from dwave.preprocessing.composites import SpinReversalTransformComposite

class TestSpinTransformComposite(unittest.TestCase):
    def test_instantiation(self):
        for factory in [ExactSolver, RandomSampler, SimulatedAnnealingSampler]:
            sampler = SpinReversalTransformComposite(factory())

            dtest.assert_sampler_api(sampler)
            dtest.assert_composite_api(sampler)
