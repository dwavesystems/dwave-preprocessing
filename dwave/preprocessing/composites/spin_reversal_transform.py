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

from dimod import Vartype, ComposedSampler

__all__ = ['SpinReversalTransformComposite']


# versionadded (0.6.8) to the __init__ file for context manager

class SpinReversalTransformComposite(ComposedSampler):
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

    @staticmethod
    def _reorder_variables(sampleset: dimod.SampleSet,
                           order: dimod.variables.Variables) -> dimod.SampleSet:
        """Return a sampleset with the given variable order."""
        if sampleset.variables == order:
            return sampleset

        # .index(...) is O(1) for dimod's Variables objects so this isn't too bad
        sampleset_order = sampleset.variables
        reorder = [sampleset_order.index(v) for v in order]

        return dimod.SampleSet.from_samples(
            (sampleset.record.sample[:, reorder], order),
            sort_labels=False,
            vartype=sampleset.vartype,
            info=sampleset.info,
            **sampleset.data_vectors,
            )

    @dimod.decorators.nonblocking_sample_method
    def sample(self, bqm: dimod.BinaryQuadraticModel, *,
               srts: typing.Optional[np.ndarray] = None,
               num_spin_reversal_transforms: typing.Optional[int] = None,
               **kwargs,
               ):
        """Sample from the binary quadratic model.

        Args:
            bqm: Binary quadratic model to be sampled from.

            srts: A boolean NumPy array with shape
                ``(num_spin_reversal_transforms, bqm.num_variables)``.
                True indicates a flip and False indicates no flip; applied to
                in the order given by ``bqm.variables``.
                If this is not specified as an input values are generated uniformly
                at random from the class pseudo-random number generator.

            num_spin_reversal_transforms:
                Number of spin reversal transform runs.
                A value of ``0`` will not transform the problem.
                If you specify a nonzero value, each spin reversal transform
                will result in an independent run of the child sampler.
                If ``srts`` is set then ``num_spin_reversal_transforms``
                is inferred by the shape, otherwise the default is 1.

        Returns:
            A sample set. Note that for a sampler that returns ``num_reads`` samples,
            the sample set will contain ``num_reads*num_spin_reversal_transforms`` samples.

        Raises:
            ValueError: If ``srts`` is inconsistent with
            ``num_spin_reversal_transforms`` or the binary quadratic model.

        Examples:
            This example runs 10 spin reversals applied to an unfrustrated chain
            of length 6.

            Using the lowest energy (ground) state returned, you can define a
            special SRT that transforms all programmed couplers to be ferromagnetic 
            (ground state to all 1).

            >>> from dimod import ExactSolver
            >>> import numpy as np
            >>> from dwave.preprocessing.composites import SpinReversalTransformComposite
            >>> base_sampler = ExactSolver()
            >>> composed_sampler = SpinReversalTransformComposite(base_sampler)
            ...
            >>> num_var = 6
            >>> num_spin_reversal_transforms = 10
            >>> J = {(i, i+1): np.random.choice([-1,1]) for i in range(num_var-1)}
            >>> h = {i: 0 for i in range(num_var)}
            >>> response = composed_sampler.sample_ising(h, J,
            ...               num_spin_reversal_transforms=num_spin_reversal_transforms)
            >>> len(response) == 2**num_var * num_spin_reversal_transforms
            True
            >>> srts = np.array([[response.first.sample[i] != 1 for i in range(num_var)]])
            >>> response = composed_sampler.sample_ising(h, J,
            ...               srts=srts, num_reads=1)
            >>> sum(response.record.num_occurrences) == 2**num_var
            True
        """
        sampler = self._child

        if srts is not None:
            nsrt, num_bqm_var = srts.shape
            if num_bqm_var != bqm.num_variables:
                raise ValueError('srt shape is inconsistent with the bqm')
            if num_spin_reversal_transforms is not None:
                if num_spin_reversal_transforms != nsrt:
                    raise ValueError('srt shape is inconsistent with num_spin_reversal_transforms')
            else:
                num_spin_reversal_transforms = nsrt
        elif num_spin_reversal_transforms is None:
            num_spin_reversal_transforms = 1

        # No SRTs, so just pass the problem through
        if num_spin_reversal_transforms == 0 or not bqm.num_variables:
            sampleset = sampler.sample(bqm, **kwargs)
            # yield twice because we're using the @nonblocking_sample_method
            yield sampleset  # this one signals done()-ness
            yield sampleset  # this is the one actually used by the user
            return

        if srts is None:
            srts = self.rng.random((num_spin_reversal_transforms, bqm.num_variables)) > .5

        # we'll be modifying the BQM, so make a copy
        bqm = bqm.copy()

        # Submit the problems
        samplesets: typing.List[dimod.SampleSet] = []
        flipped = np.zeros(bqm.num_variables, dtype=bool)  # what variables are currently flipped
        for i in range(num_spin_reversal_transforms):
            # determine what needs to be flipped
            transform = flipped != srts[i, :]

            # apply the transform
            for v, flip in zip(bqm.variables, transform):
                if flip:
                    bqm.flip_variable(v)
            flipped[transform] = ~flipped[transform]

            samplesets.append(sampler.sample(bqm, **kwargs))

        # Yield a view of the samplesets that reports done()-ness
        yield self._SampleSets(samplesets)

        # Reorder the variables of all the returned samplesets to match our
        # original BQM
        samplesets = [self._reorder_variables(ss, bqm.variables) for ss in samplesets]

        # Undo the SRTs according to vartype
        if bqm.vartype is Vartype.BINARY:
            for i, sampleset in enumerate(samplesets):
                sampleset.record.sample[:, srts[i, :]] = 1 - sampleset.record.sample[:, srts[i, :]]
        elif bqm.vartype is Vartype.SPIN:
            for i, sampleset in enumerate(samplesets):
                sampleset.record.sample[:, srts[i, :]] *= -1
        else:
            raise RuntimeError("unexpected vartype")
        if num_spin_reversal_transforms == 1:
            # If one sampleset, return full information
            # (info returned in full)
            yield samplesets[0]
        else:
            # finally combine all samplesets together
            yield dimod.concatenate(samplesets)
