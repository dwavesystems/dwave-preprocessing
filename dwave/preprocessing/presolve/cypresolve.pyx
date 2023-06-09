# distutils: sources = dwave/preprocessing/src/presolve.cpp dwave/preprocessing/src/exceptions.cpp

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

import enum as pyenum

cimport cython

from libcpp.vector cimport vector
from libcpp.utility cimport move as cppmove

import numpy as np

import dimod

from dimod.libcpp cimport ConstrainedQuadraticModel as cppConstrainedQuadraticModel
from dimod.constrained.cyconstrained cimport cyConstrainedQuadraticModel, make_cqm
from dimod.cyutilities cimport ConstNumeric

from dwave.preprocessing.libcpp cimport Feasibility as cppFeasibility
from dwave.preprocessing.presolve.exceptions import InvalidModelError

# We want to establish a relationship between presolveimpl.hpp and this file, so that
# changes to presolveimpl.hpp will trigger a rebuild.
cdef extern from "../src/presolveimpl.hpp" namespace "dwave::presolve" nogil:
    pass


# In Cython3 we'll be able to import this from C++ directly, but for now we duplicate
class Feasibility(pyenum.Enum):
    """An :py:class:`~enum.Enum` to signal whether a model is feasible or not.

    Attributes:
        Infeasible: The model is known to be infeasible
        Feasible: The model is known to be feasible
        Unknown: It is not known if the model is feasible or not

    """
    Infeasible = 0
    Feasible = 1
    Unknown = 2


cdef class cyPresolver:
    def __cinit__(self, cyConstrainedQuadraticModel cqm, *, bint move = False):
        self._original_variables = cqm.variables.copy()  # todo: implement Variables.swap()

        if move:
            self.cpppresolver = new cppPresolver[bias_type, index_type, double](cppmove(cqm.cppcqm))

            # clear out any remaining variables etc in the original model
            cqm.clear()
        else:
            self.cpppresolver = new cppPresolver[bias_type, index_type, double](cqm.cppcqm)

        # we need this because we may detach the model later
        self._model_num_variables = self.cpppresolver.model().num_variables()

    def __init__(self, cyConstrainedQuadraticModel cqm, *, bint move = False):
        pass

    def __dealloc__(self):
        if self.cpppresolver is not NULL:
            del self.cpppresolver
            self.cpppresolver = NULL

    cpdef bint apply(self) except*:
        """Normalize and presolve the held constraint quadratic model.

        Returns:
            A boolean indicating whether the model was modified by presolve.

        Raises:
            :exc:`InvalidModelError`: If the model is ill-constructed or otherwise
                not valid. This is disctinct from infeasibility.
        """
        # Use | to avoid short-circuiting
        return self.normalize() | self.presolve()

    def clear_model(self):
        """Clear the held constrained quadratic model. This is useful to save memory."""
        self.cpppresolver.detach_model()

    def copy_model(self):
        """Return a copy of the held constrained quadratic model."""
        cdef cppConstrainedQuadraticModel[bias_type, index_type] tmp = self.cpppresolver.model()  # copy
        return make_cqm(cppmove(tmp))  # then move

    def detach_model(self):
        """Create a :class:`dimod.ConstrainedQuadraticModel` from the held model.

        Subsequent attempts to access the held model raise a :exc:`RuntimeError`.
        """
        return make_cqm(cppmove(self.cpppresolver.detach_model()))

    def feasibility(self):
        """Return the feasibility of the model."""
        # Cython3 will support this automatically but for now we just do the
        # explicit check

        if self.cpppresolver.feasibility() == cppFeasibility.Infeasible:
            return Feasibility.Infeasible
        elif self.cpppresolver.feasibility() == cppFeasibility.Unknown:
            return Feasibility.Unknown
        elif self.cpppresolver.feasibility() == cppFeasibility.Feasible:
            return Feasibility.Feasible
        else:
            # sanity check
            raise RuntimeError("unexpected Feasibility")

    def load_default_presolvers(self):
        """Load the default presolvers."""
        self.cpppresolver.load_default_presolvers()

    cpdef bint normalize(self) except*:
        """Normalize the held constrained quadratic model.

        Returns:
            A boolean indicating whether the model was modified by presolve.

        Raises:
            :exc:`InvalidModelError`: If the model is ill-constructed or otherwise
                not valid. This is disctinct from infeasibility.
        """
        cdef bint changes = False

        try:
            changes = self.cpppresolver.normalize()
        except RuntimeError as err:
            # The C++ InvalidModelError is interpreted by Cython as a RuntimeError
            # We could put in a bunch of code to reinterpret it, but because this
            # is the only error type that should be raised by normalize() we just
            # do it naively. 
            raise InvalidModelError(err)

        # Save the new size for later use in restore_samples
        self._model_num_variables = self.cpppresolver.model().num_variables()

        return changes

    cpdef bint presolve(self) except*:
        """Apply any loaded presolve techniques to the held constrained quadratic model.

        Must be called after :meth:`normalize`.

        Returns:
            A boolean indicating whether the model was modified by presolve.

        Raises:
            TypeError: If called before :class:`normalize()`.
        """
        cdef bint changes = False

        try:
            changes = self.cpppresolver.presolve()
        except RuntimeError as err:
            # The C++ logic_error is interpreted by Cython as a RuntimeError.
            # The only errors here should be for a model that's not normalized.
            raise TypeError(err)

        # Save the new size for later use in restore_samples
        self._model_num_variables = self.cpppresolver.model().num_variables()

        return changes

    @cython.boundscheck(False)
    @cython.wraparound(False)
    def _restore_samples(self, ConstNumeric[:, ::1] samples):
        cdef Py_ssize_t num_samples = samples.shape[0]
        cdef Py_ssize_t num_variables = samples.shape[1]

        cdef double[:, ::1] original_samples = np.empty((num_samples, self._original_variables.size()), dtype=np.double)

        cdef vector[double] original
        cdef vector[double] reduced
        for i in range(num_samples):
            reduced.clear()
            for vi in range(num_variables):
                reduced.push_back(samples[i, vi])

            original = self.cpppresolver.restore(reduced)

            if <Py_ssize_t>original.size() != original_samples.shape[1]:
                raise RuntimeError("unexpected reduced variables size")

            for vi in range(<Py_ssize_t>original.size()):
                original_samples[i, vi] = original[vi]

        return original_samples

    def restore_samples(self, samples_like):
        """Restore the original variable labels to a set of reduced samples.

        Args:
            samples_like: A :class:`dimod.types.SamplesLike`. The samples must
                be index-labeled.

        Returns:
            Tuple:
                A 2-tuple where the first entry is the restored samples and the second
                is the original labels.

        """
        samples, labels = dimod.as_samples(samples_like, labels_type=dimod.variables.Variables)

        if not labels.is_range:
            raise ValueError("expected samples to be integer labelled")

        if samples.shape[1] != self._model_num_variables:
            raise ValueError(f"sample(s) must have {self._model_num_variables} variables, "
                             f"given sample(s) have {samples.shape[1]}")

        # we need contiguous and unsigned. as_samples actually enforces contiguous
        # but no harm in double checking for some future-proofness
        samples = np.ascontiguousarray(
                samples,
                dtype=f'i{samples.dtype.itemsize}' if np.issubdtype(samples.dtype, np.unsignedinteger) else None,
                )

        restored = self._restore_samples(samples)

        return np.asarray(restored), self._original_variables
