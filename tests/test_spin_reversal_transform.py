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

    def test_empty_bqm_composition(self):
        # Check NullSampler() works, this was a reported bug.
        
        sampler = SpinReversalTransformComposite(dimod.RandomSampler())
        bqm = dimod.BinaryQuadraticModel('SPIN')
        sampleset = sampler.sample(bqm, num_spin_reversals=1)
        self.assertEqual(len(sampleset.variables), 0)
        
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

            @property
            def variables(self):
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
        with self.assertRaisesRegex(Exception, "boom"):
            sampleset.resolve()

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

    def test_variable_order(self):
        class AlternatingSampler:
            """Return the same solution, but in alternating order."""
            def __init__(self):
                self.rng = np.random.default_rng(42)

            def sample(self, bqm):
                assert(bqm.variables == "abcd")

                order = list(bqm.variables)
                self.rng.shuffle(order)

                sample = [[-1 if bqm.linear[v] >= 0 else +1 for v in order],
                          [+1 if bqm.linear[v] >= 0 else -1 for v in order]]

                return dimod.SampleSet.from_samples_bqm((sample, order), bqm, sort_labels=False)

        sampler = SpinReversalTransformComposite(AlternatingSampler(), seed=42)

        bqm = dimod.BinaryQuadraticModel({"a": -1, "b": 1, "c": -1, "d": -1.4}, {"ab": -2}, 0, "SPIN")

        sampleset = sampler.sample(bqm, num_spin_reversal_transforms=40)

        # there should only be two unique samples after SRTs
        self.assertEqual(len(sampleset.aggregate()), 2)

    def test_variable_order2(self):
        class GroundStateSampler:
            @staticmethod
            def sample(bqm):
                return dimod.ExactSolver().sample(bqm).lowest()

        bqm = dimod.BinaryQuadraticModel(
            {}, {(1, 0): 0, (4, 3): 0, (2, 0): -1, (2, 1): 1, (5, 3): -1, (5, 4): 1}, 0, 'SPIN')

        sampler = SpinReversalTransformComposite(GroundStateSampler(), seed=42)

        sampleset = sampler.sample(bqm, num_spin_reversal_transforms=3)

        self.assertEqual(len(sampleset.aggregate()), 4)

    def test_variable_order3(self):
        class GroundStateSampler:
            @staticmethod
            def sample(bqm):
                return dimod.ExactSolver().sample(bqm).lowest()

        bqm = dimod.BinaryQuadraticModel(
            {}, {(1, 0): 1, (2, 3): 1}, 0, 'SPIN')

        sampler = SpinReversalTransformComposite(GroundStateSampler(), seed=42)

        sampleset = sampler.sample(bqm, num_spin_reversal_transforms=10)

        self.assertEqual(len(sampleset.aggregate()), 4)

    def test_propagation_of_info(self):
        # NB: info is not propagated when num_spin_reversal_transforms is 
        # greater than 1, as a best general aggregation method is not obvious.
        class InfoRichSampler:
            @staticmethod
            def sample(bqm):
                ss = dimod.ExactSolver().sample(bqm).lowest()
                ss.info['has_some'] = True
                return ss
        sampler = SpinReversalTransformComposite(InfoRichSampler())

        bqm = dimod.BinaryQuadraticModel(
            {0: 1}, {(0,1): 1}, 0, 'SPIN')

        sampleset = sampler.sample(bqm, num_spin_reversal_transforms=1)

        self.assertTrue(hasattr(sampleset,'info'))
        self.assertEqual(sampleset.info, {'has_some': True})
        
    def test_srts_argument(self):
        # All 1 ground state
        class Sampler:
            def sample(self, bqm):
                return dimod.SampleSet.from_samples_bqm([-1] * bqm.num_variables, bqm)
        num_var = 10
        bqm = dimod.BinaryQuadraticModel(
            {i: -1 for i in range(num_var)}, {}, 0, 'SPIN')  
        
        sampler = Sampler()
        ss = sampler.sample(bqm)
        samples = ss.record.sample
        sampler = SpinReversalTransformComposite(sampler)
        srts = np.zeros(shape=(1, num_var), dtype=bool)  # 
        ss = sampler.sample(bqm, srts=srts)
        self.assertTrue(np.all(ss.record.sample == samples),
                        "Neutral srts leaves result unpermuted.")
        
        srts = np.ones(shape=(1, num_var), dtype=bool)
        ss = sampler.sample(bqm, srts=srts)
        self.assertTrue(np.all(ss.record.sample == -samples),
                        "Flip-all srts inverts the order")

        ss = sampler.sample(bqm, srts=srts, num_spin_reversal_transforms=0)
        self.assertTrue(np.all(ss.record.sample == samples),
                        "srts should be ignored when num_spin_reversal_transforms=0")
        
        num_spin_reversal_transforms = 3
        srts = np.unique(np.random.random(size=(num_spin_reversal_transforms, num_var)) > 0.5, axis=0)
        
        ss = sampler.sample(bqm, srts=srts)
        self.assertEqual(np.sum(ss.record.num_occurrences), srts.shape[0],
                         "Apply 3 srtss")
        self.assertTrue(np.all(srts == (ss.record.sample==1)))
        
        with self.assertRaises(ValueError):
            # Inconsistent arguments
            ss = sampler.sample(bqm, srts=srts, num_spin_reversal_transforms=num_spin_reversal_transforms+1)
            
