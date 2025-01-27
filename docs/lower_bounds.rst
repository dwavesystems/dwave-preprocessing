.. _preprocessing_lower_bounds:

Lower Bounds
============

A common preprocessing method for binary quadratic models (BQM) is finding the
lower bound of their energy.

Roof Duality
------------

`dwave-preprocessing` contains an implementation of roof duality, an
algorithm used for finding a lower bound for the minimum of a quadratic boolean
function, as well as minimizing assignments for some of the boolean variables;
these fixed variables take the same values in all, or some, optimal solutions
[#BHT]_ [#BH]_.

.. TODO: Move to shared bibliography

.. [#BHT]
    Boros, E., P.L. Hammer, G. Tavares.
    Preprocessing of Unconstraint Quadratic Binary Optimization.
    Rutcor Research Report 10-2006, April, 2006.

.. [#BH]
    Boros, E., P.L. Hammer.
    Pseudo-Boolean optimization.
    Discrete Applied Mathematics 123, (2002), pp. 155-225.

.. autofunction:: dwave.preprocessing.lower_bounds.roof_duality

The roof duality algorithm may also be accessed through the
:class:`~dwave.preprocessing.composites.FixVariablesComposite`.
