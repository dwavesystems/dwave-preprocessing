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
# =============================================================================
"""A composite that uses the roof duality algorithm [#bht]_ [#bh]_ to fix some
variables in the binary quadratic model before passing it on to its child
sampler.

.. [#bht] Boros, E., P.L. Hammer, G. Tavares. Preprocessing of Unconstrained
    Quadratic Binary Optimization. Rutcor Research Report 10-2006, April, 2006.

.. [#bh] Boros, E., P.L. Hammer. Pseudo-Boolean optimization. Discrete Applied
    Mathematics 123, (2002), pp. 155-225

"""
from dimod.vartypes import Vartype

from dwave.preprocessing.cyfix_variables import fix_variables_wrapper

from dimod.bqm import AdjVectorBQM


def fix_variables(bqm, sampling_mode=True):
    """Determine assignments for some variables of a binary quadratic model.

    Roof duality finds a lower bound for the minimum of a quadratic polynomial. It
    can also find minimizing assignments for some of the polynomial's variables;
    these fixed variables take the same values in all optimal solutions [BHT]_ [BH]_.
    A quadratic pseudo-Boolean function can be represented as a network to find
    the lower bound through network-flow computations. `fix_variables` uses maximum
    flow in the implication network to correctly fix variables. Consequently, you can
    find an assignment for the remaining variables that attains the optimal value.

    Args:
        bqm (:obj:`.BinaryQuadraticModel`)
            A binary quadratic model.

        sampling_mode (bool, optional, default=True):
            In sampling mode, only roof-duality is used. When `sampling_mode` is false, strongly
            connected components are used to fix more variables, but in some optimal solutions
            these variables may take different values.

    Returns:
        dict: Variable assignments for some variables of the specified binary quadratic model.

    Examples:
        This example creates a binary quadratic model with a single ground state
        and fixes the model's single variable to the minimizing assignment.

        >>> bqm = dimod.BinaryQuadraticModel.from_ising({'a': 1.0}, {})
        >>> dimod.fix_variables(bqm)
        {'a': -1}

        This example has two ground states, :math:`a=b=-1` and :math:`a=b=1`, with
        no variable having a single value for all ground states, so neither variable
        is fixed.

        >>> bqm = dimod.BinaryQuadraticModel.empty(dimod.SPIN)
        >>> bqm.add_interaction('a', 'b', -1.0)
        >>> dimod.fix_variables(bqm) # doctest: +SKIP
        {}

        This example turns sampling model off, so variables are fixed to an assignment
        that attains the ground state.

        >>> bqm = dimod.BinaryQuadraticModel.empty(dimod.SPIN)
        >>> bqm.add_interaction('a', 'b', -1.0)
        >>> dimod.fix_variables(bqm, sampling_mode=False) # doctest: +SKIP
        {'a': 1, 'b': 1}

    .. [BHT] Boros, E., P.L. Hammer, G. Tavares. Preprocessing of Unconstraint Quadratic Binary
        Optimization. Rutcor Research Report 10-2006, April, 2006.

    .. [BH] Boros, E., P.L. Hammer. Pseudo-Boolean optimization. Discrete Applied Mathematics 123,
        (2002), pp. 155-225

    """
    if sampling_mode:
        method = 2  # roof-duality only
    else:
        method = 1  # roof-duality and strongly connected components

    # save original vartype to return consistently
    orig_vartype = bqm.vartype

    # convert to binary
    if bqm.vartype is Vartype.SPIN:
        bqm.change_vartype(Vartype.BINARY, inplace=True)

    linear = bqm.linear

    if all(v in linear for v in range(len(bqm))):
        # we can work with the binary form of the bqm directly
        fixed = fix_variables_wrapper(bqm, method)
    else:
        try:
            inverse_mapping = dict(enumerate(sorted(linear)))
        except TypeError:
            # in python3 unlike types cannot be sorted
            inverse_mapping = dict(enumerate(linear))
        mapping = {v: i for i, v in inverse_mapping.items()}

        fixed = fix_variables_wrapper(bqm.relabel_variables(mapping, inplace=False), method)
        fixed = {inverse_mapping[v]: val for v, val in fixed.items()}

    if orig_vartype is Vartype.SPIN:
        return {v: 2*val - 1 for v, val in fixed.items()}
    else:
        return fixed
