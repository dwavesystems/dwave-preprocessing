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

#include "catch2/catch.hpp"
#include "dimod/constrained_quadratic_model.h"
#include "presolveimpl.hpp"

namespace dwave {

using PresolverImpl = presolve::PresolverImpl<double, int, double>;
using ConstrainedQuadraticModel = dimod::ConstrainedQuadraticModel<double, int>;

TEST_CASE("Test construction", "[presolve][impl]") {
    SECTION("Default constructor") {
        auto pre = PresolverImpl();
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
        CHECK_FALSE(pre.techniques);
        CHECK(pre.feasibility() == presolve::Feasibility::Unknown);
    }

    SECTION("Empty model") {
        auto pre = PresolverImpl(ConstrainedQuadraticModel());
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
        CHECK_FALSE(pre.techniques);
        CHECK(pre.feasibility() == presolve::Feasibility::Unknown);
    }
}

TEST_CASE("Test normalize_spin_to_binary", "[presolve][impl]") {
    SECTION("Empty model") {
        auto pre = PresolverImpl(ConstrainedQuadraticModel());
        pre.normalize_spin_to_binary();
        CHECK_FALSE(pre.model().num_variables());
        CHECK_FALSE(pre.model().num_constraints());
        CHECK_FALSE(pre.techniques);
        CHECK(pre.feasibility() == presolve::Feasibility::Unknown);
    }

    GIVEN("A CQM with a single spin variable") {
        auto cqm = ConstrainedQuadraticModel();
        auto v = cqm.add_variable(dimod::Vartype::SPIN);
        cqm.objective.set_linear(v, 1.5);
        cqm.objective.set_offset(-10);
        auto c = cqm.add_linear_constraint({v}, {-2}, dimod::Sense::EQ, 3);

        WHEN("We give it to the presolver and run normalize_spin_to_binary()") {
            auto pre = PresolverImpl(cqm);
            pre.normalize_spin_to_binary();

            THEN("The model will have one binary variable and the energies will match") {
                REQUIRE(pre.model().num_variables() == 1);
                REQUIRE(pre.model().num_constraints() == 1);

                CHECK(pre.model().vartype(v) == dimod::Vartype::BINARY);

                auto bin_sample = std::vector<int>{0};
                auto spin_sample = std::vector<int>{-1};

                CHECK(pre.model().objective.energy(bin_sample.begin()) ==
                      cqm.objective.energy(spin_sample.begin()));
                CHECK(pre.model().constraint_ref(c).energy(bin_sample.begin()) ==
                      cqm.constraint_ref(c).energy(spin_sample.begin()));
            }
        }
    }
}
}  // namespace dwave
