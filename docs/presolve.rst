.. _preprocessing_presolve:

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


