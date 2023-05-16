// Copyright 2023 D-Wave Systems Inc.
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#pragma once

#ifndef _WIN32
// MSVC does not support std::experimental
#include <experimental/propagate_const>
#endif

#include <memory>
#include <utility>
#include <vector>

#include "dimod/constrained_quadratic_model.h"
#include "dwave/flags.hpp"

namespace dwave::presolve {

template <class Bias, class Index = int, class Assignment = Bias>
class Presolver {
 public:
    using model_type = dimod::ConstrainedQuadraticModel<Bias, Index>;

    using bias_type = Bias;
    using index_type = Index;
    using size_type = typename model_type::size_type;

    using assignment_type = Assignment;

    Presolver();
    ~Presolver();

    /// Construct a presolver from a constrained quadratic model.
    explicit Presolver(model_type model);

    /// Apply any loaded presolve techniques. Acts on the model() in-place.
    void apply();

    /// Detach the constrained quadratic model and return it.
    /// This clears the model from the presolver.
    model_type detach_model();

    /// Load the default presolve techniques.
    void load_default_presolvers();

    /// Return a const reference to the held constrained quadratic model.
    const model_type& model() const;

    /// Return a sample of the original CQM from a sample of the reduced CQM.
    std::vector<assignment_type> restore(std::vector<assignment_type> reduced) const;

 protected:
    class PresolverImpl_;

#ifndef _WIN32
    std::experimental::propagate_const<std::unique_ptr<PresolverImpl_>> impl_;
#else
    // MSVC does not support std::experimental
    std::unique_ptr<PresolverImpl_> impl_;
#endif
};

}  // namespace dwave::presolve
