.. _preprocessing_presolve:

============
CQM Presolve
============

Presolve algorithms enhance performance and solution quality by performing preprocessing
to reduce a problem’s redundant variables and constraints and to improve the
accuracy of the CQM.

Presolver
---------

.. automodule:: dwave.preprocessing.presolve

Class
~~~~~

.. autoclass:: Presolver

Methods
~~~~~~~

.. autosummary::
   :toctree: generated/

   ~Presolver.apply
   ~Presolver.clear_model
   ~Presolver.copy_model
   ~Presolver.detach_model
   ~Presolver.load_default_presolvers
   ~Presolver.restore_samples

C++ API
-------

.. doxygenclass:: dwave::presolve::Presolver
    :members:
    :project: dwave-preprocessing

.. doxygenclass:: dwave::presolve::Postsolver
    :members:
    :project: dwave-preprocessing
