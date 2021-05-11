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

import numpy as np

from dimod.bqm import as_bqm, AdjVectorBQM, AdjDictBQM
from dimod.core.composite import ComposedSampler
from dimod.sampleset import SampleSet, append_variables

from dwave.preprocessing.lower_bounds import roof_duality

__all__ = ['FixVariablesComposite']

class FixVariablesComposite(ComposedSampler):
    """Composite to fix variables of a problem to provided.

    Fixes variables of a binary quadratic model (BQM) and modifies linear and
    quadratic terms accordingly. Returned samples include the fixed variable.

    Args:
        child_sampler (:class:`dimod.Sampler`):
            A dimod sampler

        fixed_variables (dict, optional, default=None):
            A dictionary of variable assignments.
        
        algorithm (str, optional, default=None):
            Determines how ``fixed_variables`` are found. If ``fixed_variables``
            is not None, ``algorithm`` is ignored. If ``algorithm`` and ``fixed_variables``
            are both None, sampling will be done without fixing any variables.

        strict (bool, optional, default=True):
            Only used if ``algorithm`` is 'roof_duality'. If True, only fixes 
            variables for which assignments are true for all minimizing points 
            (strong persistency). If False, also fixes variables for which the 
            assignments are true for some but not all minimizing points (weak 
            persistency).

    Examples:
       This example uses the :class:`.FixVariablesComposite` to instantiate a
       composed sampler that submits a simple Ising problem to a sampler.
       The composed sampler fixes a variable and modifies linear and quadratic
       biases accordingly.

       >>> from dimod import ExactSolver
       >>> from dwave.preprocessing.composites import FixVariablesComposite
       >>> h = {1: -1.3, 4: -0.5}
       >>> J = {(1, 4): -0.6}
       >>> sampler = FixVariablesComposite(ExactSolver(), fixed_variables={1: -1})
       >>> sampleset = sampler.sample_ising(h, J)

       This next example involves the same problem but calculates the ``fixed_variables``
       using the 'roof_duality' ``algorithm``.

       >>> sampler = FixVariablesComposite(ExactSolver(), algorithm='roof_duality')
       >>> sampleset = sampler.sample_ising(h, J)

    """

    def __init__(self, child_sampler, *, fixed_variables=None, algorithm=None, strict=True):
        self._children = [child_sampler]

        self.fixed_variables = fixed_variables
        self.algorithm = algorithm
        self.strict = strict

    @property
    def children(self):
        return self._children

    @property
    def parameters(self):
        params = self.child.parameters.copy()
        params['fixed_variables'] = []
        return params

    @property
    def properties(self):
        return {'child_properties': self.child.properties.copy()}

    def sample(self, bqm, **parameters):
        """Sample from the provided binary quadratic model.

        Args:
            bqm (:class:`dimod.BinaryQuadraticModel`):
                Binary quadratic model to be sampled from.

            **parameters:
                Parameters for the sampling method, specified by the child sampler.

        Returns:
            :class:`dimod.SampleSet`

        """

        if not self.fixed_variables and not self.algorithm:  # None is falsey
            return self.child.sample(bqm, **parameters)
        elif not self.fixed_variables:
            if self.algorithm == 'roof_duality':
                self.fixed_variables = roof_duality(bqm, strict=self.strict)
            else:
                raise RuntimeError("Unknown algorithm: {}".format(self.algorithm))

        # make sure that we're shapeable and that we have a BQM we can mutate
        bqm_copy = as_bqm(bqm, cls=[AdjVectorBQM, AdjDictBQM],
                          copy=True)

        bqm_copy.fix_variables(self.fixed_variables)

        sampleset = self.child.sample(bqm_copy, **parameters)

        def _hook(sampleset):
            # make RoofDualityComposite non-blocking

            if sampleset.variables:
                if len(sampleset):
                    return append_variables(sampleset, self.fixed_variables)
                else:
                    return sampleset.from_samples_bqm((np.empty((0, len(bqm))),
                                                       bqm.variables), bqm=bqm)

            # there are only fixed variables, make sure that the correct number
            # of samples are returned
            samples = [self.fixed_variables]*max(len(sampleset), 1)

            return sampleset.from_samples_bqm(samples, bqm=bqm)

        return SampleSet.from_future(sampleset, _hook)
