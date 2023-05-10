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

TEST_CASE("Test normalize_remove_self_loops", "[presolve][impl]") {
    GIVEN("A CQM with a single integer self-loop") {
        auto cqm = ConstrainedQuadraticModel();
        auto v = cqm.add_variable(dimod::Vartype::INTEGER);
        cqm.objective.add_linear(v, 2);
        cqm.objective.add_quadratic(v, v, -7);
        cqm.objective.add_offset(3);

        WHEN("We give it to the presolver and run normalize_remove_self_loops()") {
            auto pre = PresolverImpl(cqm);
            pre.normalize_remove_self_loops();

            THEN("The single integer variable is now two") {
                REQUIRE(pre.model().num_variables() == 2);
                REQUIRE(pre.model().num_constraints() == 1);

                CHECK(pre.model().objective.linear(0) == 2.);
                CHECK(pre.model().objective.linear(1) == 0.);
                CHECK(pre.model().objective.quadratic(0, 0) == 0.);
                CHECK(pre.model().objective.quadratic(1, 1) == 0.);
                CHECK(pre.model().objective.quadratic(0, 1) == -7.);
                CHECK(pre.model().objective.offset() == 3.);

                CHECK(pre.model().constraint_ref(0).linear(0) +
                              pre.model().constraint_ref(0).linear(1) ==
                      pre.model().constraint_ref(0).rhs());
                CHECK(pre.model().constraint_ref(0).linear(0));
            }
        }
    }

    GIVEN("A CQM with several self-loops") {
        auto cqm = ConstrainedQuadraticModel();
        auto i = cqm.add_variable(dimod::Vartype::INTEGER);
        auto j = cqm.add_variable(dimod::Vartype::INTEGER, -5, 5);
        auto k = cqm.add_variable(dimod::Vartype::INTEGER, -10, 10);

        auto c0 = cqm.add_constraint();
        cqm.constraint_ref(c0).add_quadratic(i, i, 2);
        cqm.constraint_ref(c0).add_quadratic(j, j, 3);

        auto c1 = cqm.add_constraint();
        cqm.constraint_ref(c1).add_quadratic(j, j, 4);
        cqm.constraint_ref(c1).add_quadratic(k, k, 5);

        WHEN("We give it to the presolver and run normalize_remove_self_loops()") {
            auto pre = PresolverImpl(cqm);
            pre.normalize_remove_self_loops();

            THEN("Three new variables and two constraints are added") {
                REQUIRE(pre.model().num_variables() == 6);
                REQUIRE(pre.model().num_constraints() == 5);
            }
        }
    }
}
}  // namespace dwave
