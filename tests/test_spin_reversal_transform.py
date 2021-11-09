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

from dwave.system.testing import MockDWaveSampler
from dimod import NullSampler, ExactSolver, RandomSampler, SimulatedAnnealingSampler
from greedy import SteepestDescentSolver
import dimod.testing as dtest

from dwave.preprocessing.composites import SpinReversalTransformComposite

import numpy as np

class TestSpinTransformComposite(unittest.TestCase):
    def test_instantiation(self):
        for factory in [ExactSolver, RandomSampler, SimulatedAnnealingSampler,
                        MockDWaveSampler, NullSampler, SteepestDescentSolver]:
            sampler = SpinReversalTransformComposite(factory())

            dtest.assert_sampler_api(sampler)
            dtest.assert_composite_api(sampler)
            
    def test_NullSampler_composition(self):
        # Check NullSampler() works, this was a reported bug.
        
        sampler = SpinReversalTransformComposite(NullSampler())
        sampleset = sampler.sample_ising({'a': 1},{},num_spin_reversals=1)
        self.assertTrue(len(sampleset)==0)
        sampleset = sampler.sample_ising({'a': 1},{},num_spin_reversals=2)
        self.assertTrue(len(sampleset)==0)
        
    def test_concatenation_size(self):
        # Check that concatenation works, and that when concatenation
        # is not applied (num_spin_reversal_transforms=1, Default)
        # information is not stripped from the sampleset.
        
        sampler = SpinReversalTransformComposite(MockDWaveSampler())
        num_reads = 1
        for num_spin_reversal_transforms in [1,2]:
            sampleset = sampler.sample_ising(
                {0: 1},{},
                num_spin_reversal_transforms=num_spin_reversal_transforms,
                num_reads=num_reads)
            self.assertTrue(sum(sampleset.record.num_occurrences)==
                            num_reads*num_spin_reversal_transforms)
            if num_spin_reversal_transforms == 1:
                self.assertTrue(hasattr(sampleset,'info'))
            
    def test_sampleset_size(self):
        # Check num_reads and num_spin_reversal_transforms combine
        # for anticipated number of samples.
        
        sampler = SpinReversalTransformComposite(MockDWaveSampler())
        for num_spin_reversal_transforms in [1,2]:
            for num_reads in [1,3]:
                sampleset = sampler.sample_ising(
                    {0: 1},{},
                    num_spin_reversal_transforms=num_spin_reversal_transforms,
                    num_reads=num_reads)
                self.assertTrue(sum(sampleset.record.num_occurrences)==
                                num_reads*num_spin_reversal_transforms)
                
            
    def test_sign_errors_vars(self):
        # Check that when an SRT is applied, sample sign is properly reversed.
        # Unique ground state of independent h=-1 biased variables is all +1. 
        
        sampler = SpinReversalTransformComposite(SteepestDescentSolver())
        
        sampleset = sampler.sample_ising({i: -1 for i in range(10)},{},
                                         num_spin_reversal_transforms=1,
                                         num_reads=1)
        self.assertTrue((sampleset.record.sample==1).all())
        
        sampleset = sampler.sample_ising({i: -1 for i in range(10)},{},
                                         num_spin_reversal_transforms=10,
                                         num_reads=1)
        
        self.assertTrue((sampleset.record.sample==1).all())
        
    def test_sign_errors_couplers(self):
        # Check that when an SRT is applied, sample sign is properly reversed.
        # Unique ground state of a ferromagnetically coupled pair has two
        # values equal, ground state can be found by steepest descent.
        
        sampler = SpinReversalTransformComposite(SteepestDescentSolver())
        
        sampleset = sampler.sample_ising({},{(0,1) : -1},
                                         num_spin_reversal_transforms=1,
                                         num_reads=1)
        self.assertTrue((np.prod(sampleset.record.sample,axis=1)==1).all())
        
        sampleset = sampler.sample_ising({i: -1 for i in range(10)},{},
                                         num_spin_reversal_transforms=10,
                                         num_reads=1)
        
        self.assertTrue((sampleset.record.sample==1).all())
