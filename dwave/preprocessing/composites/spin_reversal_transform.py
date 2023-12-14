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

import typing

import dimod
import numpy as np

from dimod import Vartype

__all__ = ['SpinReversalTransformComposite']


class SpinReversalTransformComposite(dimod.core.Sampler, dimod.core.Composite):
    """Composite for applying spin reversal transform preprocessing.

    A spin-reversal transform can improve sample statistics when the 
    sampler is a physical object with asymmetries such as a QPU.
    The technique works as follows: given an :math:`n`-variable Ising 
    problem, the composite selects a random :math:`g\in\{\pm1\}^n` and 
    transforms the problem via :math:`h_i\mapsto h_ig_i` and 
    :math:`J_{ij}\mapsto J_{ij}g_ig_j`. Solutions :math:`s` of the 
    original problem and :math:`s^\prime` of the transformed problem 
    are related by :math:`s^\prime_i=s_ig_i` and have identical energies. 
    [#km]_

    .. note::
        If you are configuring an anneal schedule, be mindful that this
        composite does not recognize the ``initial_state`` parameter 
        used by dimod's :class:`~dwave.system.samplers.DWaveSampler` for 
        reverse annealing (composites do not generally process all keywords 
        of child samplers) and does not flip any of the configured initial 
        states. 

    Args:
        sampler: A `dimod` sampler object.

        seed: As passed to :func:`numpy.random.default_rng`.

    Examples:
        This example composes a dimod ExactSolver sampler with spin transforms then
        uses it to sample an Ising problem.

        >>> from dimod import ExactSolver
        >>> from dwave.preprocessing.composites import SpinReversalTransformComposite
        >>> base_sampler = ExactSolver()
        >>> composed_sampler = SpinReversalTransformComposite(base_sampler)
        ... # Sample an Ising problem
        >>> response = composed_sampler.sample_ising({'a': -0.5, 'b': 1.0}, {('a', 'b'): -1})
        >>> response.first.sample
        {'a': -1, 'b': -1}

    References
    ----------
    .. [#km] Andrew D. King and Catherine C. McGeoch. Algorithm engineering
        for a quantum annealing platform. https://arxiv.org/abs/1410.2628,
        2014.

    """
    _children: typing.List[dimod.core.Sampler]
    _parameters: typing.Dict[str, typing.Sequence[str]]
    _properties: typing.Dict[str, typing.Any]

    def __init__(self, child: dimod.core.Sampler, *, seed=None):
        self._child = child
        self.rng = np.random.default_rng(seed)

    @property
    def children(self) -> typing.List[dimod.core.Sampler]:
        try:
            return self._children
        except AttributeError:
            pass

        self._children = children = [self._child]
        return children

    @property
    def parameters(self) -> typing.Dict[str, typing.Sequence[str]]:
        try:
            return self._parameters
        except AttributeError:
            pass

        self._parameters = parameters = dict(spin_reversal_variables=tuple())
        parameters.update(self._child.parameters)
        return parameters

    @property
    def properties(self) -> typing.Dict[str, typing.Any]:
        try:
            return self._properties
        except AttributeError:
            pass

        self._properties = dict(child_properties=self._child.properties)
        return self._properties

    class _SampleSets:
        def __init__(self, samplesets: typing.List[dimod.SampleSet]):
            self.samplesets = samplesets

        def done(self) -> bool:
            return all(ss.done() for ss in self.samplesets)

    @dimod.decorators.nonblocking_sample_method
    def sample(self, bqm: dimod.BinaryQuadraticModel, *,
               num_spin_reversal_transforms: int = 1,
               **kwargs,
               ):
        """Sample from the binary quadratic model.

        Args:
            bqm: Binary quadratic model to be sampled from.

            num_spin_reversal_transforms:
                Number of spin reversal transform runs.
                A value of ``0`` will not transform the problem.
                If you specify a nonzero value, each spin reversal transform
                will result in an independent run of the child sampler.

        Returns:
            A sample set. Note that for a sampler that returns ``num_reads`` samples,
            the sample set will contain ``num_reads*num_spin_reversal_transforms`` samples.

        Examples:
            This example runs 100 spin reversals applied to one variable of a QUBO problem.

            >>> from dimod import ExactSolver
            >>> from dwave.preprocessing.composites import SpinReversalTransformComposite
            >>> base_sampler = ExactSolver()
            >>> composed_sampler = SpinReversalTransformComposite(base_sampler)
            ...
            >>> Q = {('a', 'a'): -1, ('b', 'b'): -1, ('a', 'b'): 2}
            >>> response = composed_sampler.sample_qubo(Q,
            ...               num_spin_reversal_transforms=100)
            >>> len(response)
            400
        """
        sampler = self._child

        # No SRTs, so just pass the problem through
        if not num_spin_reversal_transforms or not bqm.num_variables:
            sampleset = sampler.sample(bqm, **kwargs)
            # yield twice because we're using the @nonblocking_sample_method
            yield sampleset  # this one signals done()-ness
            yield sampleset  # this is the one actually used by the user
            return

        # we'll be modifying the BQM, so make a copy
        bqm = bqm.copy()

        # We maintain the Leap behavior that num_spin_reversal_transforms == 1
        # corresponds to a single problem with randomly flipped variables.

        # Get the SRT matrix
        SRT = self.rng.random((num_spin_reversal_transforms, bqm.num_variables)) > .5

        # Submit the problems
        samplesets: typing.List[dimod.SampleSet] = []
        flipped = np.zeros(bqm.num_variables, dtype=bool)  # what variables are currently flipped
        for i in range(num_spin_reversal_transforms):
            # determine what needs to be flipped
            transform = flipped != SRT[i, :]

            # apply the transform
            for v, flip in zip(bqm.variables, transform):
                if flip:
                    bqm.flip_variable(v)
            flipped[transform] = ~flipped[transform]

            samplesets.append(sampler.sample(bqm, **kwargs))

        # Yield a view of the samplesets that reports done()-ness
        yield self._SampleSets(samplesets)

        # Undo the SRTs according to vartype
        if bqm.vartype is Vartype.BINARY:
            for i, sampleset in enumerate(samplesets):
                sampleset.record.sample[:, SRT[i, :]] = 1 - sampleset.record.sample[:, SRT[i, :]]
        elif bqm.vartype is Vartype.SPIN:
            for i, sampleset in enumerate(samplesets):
                sampleset.record.sample[:, SRT[i, :]] *= -1
        else:
            raise RuntimeError("unexpected vartype")

        # finally combine all samplesets together
        yield dimod.concatenate(samplesets)
