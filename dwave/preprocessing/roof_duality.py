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

from dimod.vartypes import Vartype
from dimod.reference.composites.fixedvariable import FixedVariableComposite

from dwave.preprocessing.cyfix_variables import fix_variables_wrapper

def fix_variables(bqm, *, strict=True):
    """Determine minimizing assignments for some variables of a binary quadratic 
    model using roof duality.

    Args:
        bqm (:class:`.BinaryQuadraticModel`):
            A binary quadratic model.

        strict (bool, optional, default=True):
            If True, only fixes variables for which assignments are true for all 
            minimizing points (strong persistency). If False, also fixes variables 
            for which the assignments are true for some but not all minimizing 
            points (weak persistency).

    Returns:
        dict: Variable assignments for some variables of ``bqm``.

    Examples:
        This example creates a binary quadratic model with a single ground state
        and fixes the model's single variable to the minimizing assignment.

        >>> import dimod
        >>> from dwave.preprocessing import roof_duality
        >>> bqm = dimod.BinaryQuadraticModel.from_ising({'a': 1.0}, {})
        >>> roof_duality.fix_variables(bqm)
        {'a': -1}

        This example has two ground states, :math:`a=b=-1` and :math:`a=b=1`, with
        no variable having a single value for all ground states, so neither variable
        is fixed.

        >>> bqm = dimod.BinaryQuadraticModel.empty(dimod.SPIN)
        >>> bqm.add_interaction('a', 'b', -1.0)
        >>> roof_duality.fix_variables(bqm) # doctest: +SKIP
        {}

        This example sets ``strict`` to False, so variables are fixed to an assignment
        that attains the ground state.

        >>> bqm = dimod.BinaryQuadraticModel.empty(dimod.SPIN)
        >>> bqm.add_interaction('a', 'b', -1.0)
        >>> roof_duality.fix_variables(bqm, strict=False) # doctest: +SKIP
        {'a': 1, 'b': 1}

    """
    bqm_ = bqm

    if bqm_.vartype is Vartype.SPIN:
        bqm_ = bqm.change_vartype(Vartype.BINARY, inplace=False)

    linear = bqm_.linear

    if all(v in linear for v in range(len(bqm_))):
        # we can work with the binary form of the bqm directly
        fixed = fix_variables_wrapper(bqm_, strict)
    else:
        inverse_mapping = dict(enumerate(linear))
        mapping = {v: i for i, v in inverse_mapping.items()}

        # no need to make a copy if we've already done so
        inplace = bqm.vartype is Vartype.SPIN
        bqm_ = bqm_.relabel_variables(mapping, inplace=inplace)

        fixed = fix_variables_wrapper(bqm_, strict)
        fixed = {inverse_mapping[v]: val for v, val in fixed.items()}

    if bqm.vartype is Vartype.SPIN:
        return {v: 2*val - 1 for v, val in fixed.items()}
    else:
        return fixed


class RoofDualityComposite(FixedVariableComposite):
    """A composite that uses the :func:`fix_variables` function to determine 
    variable assignments, then fixes them before calling its child sampler.
    
    Returned samples include the fixed variables.

    Args:
       child_sampler (:class:`dimod.Sampler`):
            A dimod sampler. Used to sample the binary quadratic model after
            variables have been fixed.

    Examples:
        This example uses the RoofDualityComposite to fix the variables of a 
        small Ising problem before solving with dimod's ExactSolver.

        >>> from dimod import ExactSolver
        >>> from dwave.preprocessing.roof_duality import RoofDualityComposite
        >>> sampler = RoofDualityComposite(ExactSolver())
        >>> sampleset = sampler.sample_ising({'a': 10},  {'ab': -1, 'bc': 1})
        >>> print(sampleset)
           a  b  c energy num_oc.
        0 -1 -1 +1  -12.0       1
        ['SPIN', 1 rows, 1 samples, 3 variables]

    """
    @property
    def parameters(self):
        params = self.child.parameters.copy()
        params['strict'] = []
        return params

    def sample(self, bqm, *, strict=True, **parameters):
        """Sample from the provided binary quadratic model.

        Uses the :func:`roof_duality.fix_variables` function to determine
        which variables to fix.

        Args:
            bqm (:class:`dimod.BinaryQuadraticModel`):
                Binary quadratic model to be sampled from.

            strict (bool, optional, default=True):
                If True, only fixes variables for which assignments are true for 
                all minimizing points (strong persistency). If False, also fixes 
                variables for which the assignments are true for some but not all 
                minimizing points (weak persistency).

            **parameters:
                Parameters for the child sampler.

        Returns:
            :class:`dimod.SampleSet`

        """
        # use roof-duality to decide which variables to fix
        parameters['fixed_variables'] = fix_variables(bqm, strict=strict)
        return super(RoofDualityComposite, self).sample(bqm, **parameters)
