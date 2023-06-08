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
import numpy as np

from dwave.preprocessing.composites import SpinReversalTransformComposite


@dimod.testing.load_sampler_bqm_tests(SpinReversalTransformComposite(dimod.ExactSolver()))
class TestSpinTransformComposite(unittest.TestCase):
    def test_instantiation(self):
        for factory in [dimod.ExactSolver, dimod.RandomSampler, dimod.SimulatedAnnealingSampler,
                        dimod.NullSampler]:
            sampler = SpinReversalTransformComposite(factory())

            dimod.testing.assert_sampler_api(sampler)
            dimod.testing.assert_composite_api(sampler)

    def test_NullSampler_composition(self):
        # Check NullSampler() works, this was a reported bug.

        sampler = SpinReversalTransformComposite(dimod.NullSampler())
        sampleset = sampler.sample_ising({'a': 1}, {}, num_spin_reversal_transforms=1)

        self.assertTrue(len(sampleset) == 0)
        sampleset = sampler.sample_ising({'a': 1}, {}, num_spin_reversal_transforms=2)

        self.assertTrue(len(sampleset) == 0)

    def test_concatenation_stripping(self):
        # Check samplesets are not stripped of information
        # under default operation

        # Simple sampler with an info field.
        # When multiple samplesets needn't be recombined, this should
        # be maintained
        class RichSampler(dimod.NullSampler):
            def sample_ising(self, *args, **kwargs):
                ss = super().sample_ising(*args, **kwargs)
                ss.info['hello'] = 'world'
                return ss

        sampler = SpinReversalTransformComposite(RichSampler())

        sampleset = sampler.sample_ising({0: 1}, {})
        self.assertTrue(hasattr(sampleset, 'info'))

    def test_sampleset_size(self):
        # Check num_reads and num_spin_reversal_transforms combine
        # for anticipated number of samples.

        sampler = SpinReversalTransformComposite(dimod.RandomSampler())
        for num_spin_reversal_transforms in [1, 2]:
            for num_reads in [1, 3]:
                sampleset = sampler.sample_ising(
                    {0: 1}, {},
                    num_spin_reversal_transforms=num_spin_reversal_transforms,
                    num_reads=num_reads)
                self.assertTrue(sum(sampleset.record.num_occurrences) ==
                                num_reads*num_spin_reversal_transforms)

    def test_empty(self):
        # Check that empty BQMs are handled
        sampler = SpinReversalTransformComposite(dimod.ExactSolver())
        bqm = dimod.BinaryQuadraticModel({}, {}, 0.0, dimod.SPIN)
        sampleset = sampler.sample(bqm, num_spin_reversal_transforms=3)

        self.assertEqual(sampleset.record.sample.shape, (0, 0))
        self.assertIs(sampleset.vartype, bqm.vartype)

    def test_ground_state(self):
        num_variables = 10

        bqm = dimod.BQM({v: 1 for v in range(num_variables)}, {}, 0, "SPIN")

        class Sampler:
            def sample(self, bqm):
                sample = {v: +1 if bias < 0 else -1 for v, bias in bqm.linear.items()}
                return dimod.SampleSet.from_samples_bqm(sample, bqm)

        sampler = SpinReversalTransformComposite(Sampler())
        sampleset = sampler.sample(bqm, num_spin_reversal_transforms=num_variables)

        self.assertTrue((sampleset.record.sample == -1).all())

    def test_async(self):
        class SampleSet:
            is_done = False

            def done(self):
                return self.is_done

            @property
            def record(self):
                raise Exception("boom")

        class Sampler:
            def __init__(self):
                self.count = 0

            def sample(self, bqm):
                self.count += 1
                return SampleSet()

        sampler = SpinReversalTransformComposite(Sampler())

        # No exception because nothing has been resolved
        sampleset = sampler.sample_ising({'a': 1}, {})

        # We can test whether it's done
        self.assertFalse(sampleset.done())
        SampleSet.is_done = True
        self.assertTrue(sampleset.done())

        # Resolving raises the exception
        with self.assertRaises(Exception) as cm:
            sampleset.resolve()

        self.assertEqual(str(cm.exception), "boom")

    def test_seed(self):
        bqm = dimod.BQM(1000, "SPIN")

        class Sampler:
            def sample(self, bqm):
                return dimod.SampleSet.from_samples_bqm([-1] * bqm.num_variables, bqm)

        ss1 = SpinReversalTransformComposite(Sampler(), seed=42).sample(bqm)
        ss2 = SpinReversalTransformComposite(Sampler(), seed=42).sample(bqm)
        ss3 = SpinReversalTransformComposite(Sampler(), seed=35).sample(bqm)

        self.assertTrue((ss1.record == ss2.record).all())
        self.assertFalse((ss1.record == ss3.record).all())
