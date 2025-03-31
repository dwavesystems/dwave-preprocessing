.. _preprocessing_api_ref:

=============
API Reference
=============

Composites
==========

.. currentmodule:: dwave.preprocessing.composites

.. automodule:: dwave.preprocessing.composites.__init__
    :no-index:

Clip
----

Class
~~~~~

.. autoclass:: ClipComposite

Properties
~~~~~~~~~~

.. autosummary::
    :toctree: generated/

    ~ClipComposite.child
    ~ClipComposite.children
    ~ClipComposite.parameters
    ~ClipComposite.properties

Methods
~~~~~~~

.. autosummary::
    :toctree: generated/

    ~ClipComposite.sample
    ~ClipComposite.sample_ising
    ~ClipComposite.sample_qubo

Connected Components
--------------------

Class
~~~~~

.. autoclass:: ConnectedComponentsComposite

Properties
~~~~~~~~~~

.. autosummary::
    :toctree: generated/

    ~ConnectedComponentsComposite.child
    ~ConnectedComponentsComposite.children
    ~ConnectedComponentsComposite.parameters
    ~ConnectedComponentsComposite.properties

Methods
~~~~~~~

.. autosummary::
    :toctree: generated/

    ~ConnectedComponentsComposite.sample
    ~ConnectedComponentsComposite.sample_ising
    ~ConnectedComponentsComposite.sample_qubo

Fix Variables
-------------

Class
~~~~~

.. autoclass:: FixVariablesComposite

Properties
~~~~~~~~~~

.. autosummary::
    :toctree: generated/

    ~FixVariablesComposite.child
    ~FixVariablesComposite.children
    ~FixVariablesComposite.parameters
    ~FixVariablesComposite.properties

Methods
~~~~~~~

.. autosummary::
    :toctree: generated/

    ~FixVariablesComposite.sample
    ~FixVariablesComposite.sample_ising
    ~FixVariablesComposite.sample_qubo

Scale
-----

Class
~~~~~

.. autoclass:: ScaleComposite

Properties
~~~~~~~~~~

.. autosummary::
    :toctree: generated/

    ~ScaleComposite.child
    ~ScaleComposite.children
    ~ScaleComposite.parameters
    ~ScaleComposite.properties

Methods
~~~~~~~

.. autosummary::
    :toctree: generated/

    ~ScaleComposite.sample
    ~ScaleComposite.sample_ising
    ~ScaleComposite.sample_qubo


Spin Reversal Transform
-----------------------

Class
~~~~~

.. autoclass:: SpinReversalTransformComposite

Properties
~~~~~~~~~~

.. autosummary::
    :toctree: generated/

    ~SpinReversalTransformComposite.child
    ~SpinReversalTransformComposite.children
    ~SpinReversalTransformComposite.parameters
    ~SpinReversalTransformComposite.properties

Methods
~~~~~~~

.. autosummary::
    :toctree: generated/

    ~SpinReversalTransformComposite.sample
    ~SpinReversalTransformComposite.sample_ising
    ~SpinReversalTransformComposite.sample_qubo


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
[Bor2006]_ [Bor2002]_.

.. autofunction:: dwave.preprocessing.lower_bounds.roof_duality

The roof duality algorithm may also be accessed through the
:class:`~dwave.preprocessing.composites.FixVariablesComposite`.


CQM Presolve
============

.. automodule:: dwave.preprocessing.presolve.pypresolve

Presolver
---------

Class
~~~~~

.. autoclass:: Presolver

Methods
~~~~~~~

.. autosummary::
    :toctree: generated/

    ~Presolver.add_techniques
    ~Presolver.apply
    ~Presolver.clear_model
    ~Presolver.copy_model
    ~Presolver.detach_model
    ~Presolver.feasibility
    ~Presolver.load_default_presolvers
    ~Presolver.normalize
    ~Presolver.presolve
    ~Presolver.restore_samples
    ~Presolver.set_techniques
    ~Presolver.techniques

Feasibility
-----------

.. autoclass:: Feasibility

TechniqueFlags
--------------

.. autoclass:: TechniqueFlags

C++ API
-------

.. doxygenclass:: dwave::presolve::Presolver
    :members:
    :project: dwave-preprocessing

.. doxygenenum:: dwave::presolve::Feasibility
    :project: dwave-preprocessing

.. doxygenenum:: dwave::presolve::TechniqueFlags
    :project: dwave-preprocessing


C++ API
=======

Functions
---------
.. doxygenfunction:: fixQuboVariables(dimod::BinaryQuadraticModel<B, V> &bqm, bool sample, double offset)
    :project: dwave-preprocessing
.. doxygenfunction:: fixQuboVariables(PosiformInfo &posiform_info, int num_bqm_variables, bool sample, std::vector<std::pair<int, int>> &fixed_variables)
    :project: dwave-preprocessing

Classes
-------
.. doxygenclass:: PosiformInfo
    :members:
    :project: dwave-preprocessing