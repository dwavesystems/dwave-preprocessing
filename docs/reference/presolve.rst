.. _preprocessing_presolve:

============
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
