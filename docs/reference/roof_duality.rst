.. _preprocessing_roof_duality:

============
Roof Duality
============

.. currentmodule:: dwave.preprocessing.roof_duality

Roof duality finds a lower bound for the minimum of a quadratic polynomial. It 
can also find minimizing assignments for some of the polynomial's variables; 
these fixed variables take the same values in all optimal solutions [BHT]_ [BH]_.
The problem size may then be reduced by fixing the variables of a :term:`binary 
quadratic model` (BQM) before solving.

The roof duality algorithm may be accessed through :func:`fix_variables` or the 
:class:`RoofDualityComposite`.

.. [BHT] Boros, E., P.L. Hammer, G. Tavares. Preprocessing of Unconstraint Quadratic Binary Optimization. Rutcor Research Report 10-2006, April, 2006.

.. [BH] Boros, E., P.L. Hammer. Pseudo-Boolean optimization. Discrete Applied Mathematics 123, (2002), pp. 155-225

.. autofunction:: fix_variables

Class
~~~~~

.. autoclass:: RoofDualityComposite

Properties
~~~~~~~~~~

.. autosummary::
   :toctree: generated/

   RoofDualityComposite.child
   RoofDualityComposite.children
   RoofDualityComposite.parameters
   RoofDualityComposite.properties

Methods
~~~~~~~

.. autosummary::
   :toctree: generated/

   RoofDualityComposite.sample
   RoofDualityComposite.sample_ising
   RoofDualityComposite.sample_qubo
