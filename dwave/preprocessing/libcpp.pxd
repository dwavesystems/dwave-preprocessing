# distutils: include_dirs = dwave/preprocessing/include/

# Copyright 2022 D-Wave Systems Inc.
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

from libcpp.vector cimport vector

from dimod.libcpp cimport ConstrainedQuadraticModel

cdef extern from "dwave/exceptions.hpp" namespace "dwave::presolve" nogil:
    pass

cdef extern from "dwave/flags.hpp" namespace "dwave::presolve" nogil:
    pass

cdef extern from "dwave/presolve.hpp" namespace "dwave::presolve" nogil:
    cdef cppclass Presolver[bias_type, index_type, assignment_type]:
        ctypedef ConstrainedQuadraticModel[bias_type, index_type] model_type

        Presolver()
        Presolver(model_type)
        void apply() except+
        model_type detach_model()
        void load_default_presolvers()
        model_type& model()
        vector[assignment_type] restore(vector[assignment_type])
