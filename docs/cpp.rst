.. _preprocessing_cppdocs:

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