// Copyright 2022 D-Wave Systems Inc.
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

#include "catch2/catch.hpp"
#include "dimod/constrained_quadratic_model.h"

#include "presolveimpl.hpp"

namespace dwave {

using PresolverImpl = presolve::PresolverImpl<double, int, double>;
using ConstrainedQuadraticModel = dimod::ConstrainedQuadraticModel<double, int>;

TEST_CASE("Test construction", "[presolve][impl]") {
    SECTION("Default constructor") {
        auto pre = PresolverImpl();
        CHECK_FALSE(pre.flags);
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
    }

    SECTION("Empty model") {
        auto pre = PresolverImpl(ConstrainedQuadraticModel());
        CHECK_FALSE(pre.flags);
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
    }
}
}  // namespace dwave
