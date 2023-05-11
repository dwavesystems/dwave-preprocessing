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

TEST_CASE("Test normalization_check_nan", "[presolve][impl]") {
    SECTION("Linear objective with NaNs") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variable(dimod::BINARY);
        cqm.objective.set_linear(0, std::numeric_limits<double>::quiet_NaN());
        CHECK_THROWS_AS(PresolverImpl::normalization_check_nan(cqm.objective),
                        presolve::InvalidModelError);
    }
}

TEST_CASE("Test normalization_flip_constraints", "[presolve][impl]") {
    GIVEN("A CQM with three constraints of different senses") {
        auto cqm = ConstrainedQuadraticModel();
        auto i = cqm.add_variable(dimod::Vartype::INTEGER);
        auto j = cqm.add_variable(dimod::Vartype::INTEGER, -5, 5);
        auto k = cqm.add_variable(dimod::Vartype::INTEGER, -10, 10);

        auto c1 = cqm.add_linear_constraint({i, j}, {1, 2}, dimod::Sense::EQ, 3);
        auto c2 = cqm.add_linear_constraint({j, k}, {4, 5}, dimod::Sense::LE, 6);
        auto c3 = cqm.add_linear_constraint({i, k}, {7, 8}, dimod::Sense::GE, 9);

        cqm.constraint_ref(c1).add_quadratic(i, j, 10);
        cqm.constraint_ref(c2).add_quadratic(j, k, 11);
        cqm.constraint_ref(c3).add_quadratic(i, k, 12);

        cqm.constraint_ref(c1).add_offset(13);
        cqm.constraint_ref(c2).add_offset(14);
        cqm.constraint_ref(c3).add_offset(15);

        WHEN("We give it to the presolver and run normalization_remove_self_loops()") {
            auto pre = PresolverImpl(cqm);
            pre.normalization_flip_constraints();

            THEN("The EQ and LE constraints are not changed") {
                for (auto ci : {c1, c2}) {
                    auto oldconstraint = cqm.constraint_ref(ci);
                    auto newconstraint = pre.model().constraint_ref(ci);

                    CHECK(oldconstraint.num_variables() == newconstraint.num_variables());
                    CHECK(oldconstraint.num_interactions() == newconstraint.num_interactions());
                    for (std::size_t v = 0; v < cqm.num_variables(); ++v)
                        CHECK(oldconstraint.linear(v) == newconstraint.linear(v));
                    for (std::size_t u = 0; u < cqm.num_variables(); ++u)
                        for (std::size_t v = 0; v < cqm.num_variables(); ++v)
                            CHECK(oldconstraint.quadratic(u, v) == newconstraint.quadratic(u, v));
                    CHECK(oldconstraint.offset() == newconstraint.offset());
                    CHECK(oldconstraint.sense() == newconstraint.sense());
                    CHECK(oldconstraint.rhs() == newconstraint.rhs());
                }
            }

            THEN("The GE constraint is flipped") {
                auto oldconstraint = cqm.constraint_ref(c3);
                auto newconstraint = pre.model().constraint_ref(c3);

                CHECK(oldconstraint.num_variables() == newconstraint.num_variables());
                CHECK(oldconstraint.num_interactions() == newconstraint.num_interactions());
                for (std::size_t v = 0; v < cqm.num_variables(); ++v)
                    CHECK(oldconstraint.linear(v) == -newconstraint.linear(v));
                for (std::size_t u = 0; u < cqm.num_variables(); ++u)
                    for (std::size_t v = 0; v < cqm.num_variables(); ++v)
                        CHECK(oldconstraint.quadratic(u, v) == -newconstraint.quadratic(u, v));
                CHECK(oldconstraint.offset() == -newconstraint.offset());
                CHECK(dimod::Sense::LE == newconstraint.sense());
                CHECK(oldconstraint.rhs() == -newconstraint.rhs());
            }
        }
    }
}

TEST_CASE("Test normalization_remove_invalid_markers", "[presolve][impl]") {
    GIVEN("A CQM with overlapping discrete constraints") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variables(dimod::Vartype::BINARY, 8);
        auto c0 = cqm.add_linear_constraint({0, 1, 2}, {1, 1, 1}, dimod::Sense::EQ, 1);  // overlap
        auto c1 = cqm.add_linear_constraint({2, 3, 4}, {1, 1, 1}, dimod::Sense::EQ, 1);  // overlap
        auto c2 = cqm.add_linear_constraint({5, 6, 7}, {1, 1, 1}, dimod::Sense::EQ, 1);  // not

        for (auto ci : {c0, c1, c2}) cqm.constraint_ref(ci).mark_discrete();

        WHEN("We give it to the presolver and run normalization_remove_invalid_markers()") {
            auto pre = PresolverImpl(cqm);
            pre.normalization_remove_invalid_markers();

            THEN("Only the valid discrete constraints are still marked") {
                CHECK(pre.model().constraint_ref(c0).marked_discrete() ^
                      pre.model().constraint_ref(c1).marked_discrete());
                CHECK(pre.model().constraint_ref(c2).marked_discrete());
            }
        }
    }

    GIVEN("A CQM with a mis-marked constraint") {
        auto cqm = ConstrainedQuadraticModel();
        cqm.add_variables(dimod::Vartype::BINARY, 9);  // need the constraints to not overlap
        auto c0 = cqm.add_linear_constraint({0, 1, 2}, {1, 1, 1}, dimod::Sense::EQ, 1);  // valid
        auto c1 = cqm.add_linear_constraint({3, 4, 5}, {1, 2, 1}, dimod::Sense::EQ, 1);  // invalid
        auto c2 = cqm.add_linear_constraint({6, 7, 8}, {2, 2, 2}, dimod::Sense::EQ, 2);  // valid
        // NB: there are other cases we could test, but ultimately we'd just be testing
        // constraint.is_onehot() so let's rely on dimod to test that method thoroughly.

        for (auto ci : {c0, c1, c2}) cqm.constraint_ref(ci).mark_discrete();

        WHEN("We give it to the presolver and run normalization_remove_invalid_markers()") {
            auto pre = PresolverImpl(cqm);
            pre.normalization_remove_invalid_markers();

            THEN("Only the valid discrete constraints are still marked") {
                CHECK(pre.model().constraint_ref(c0).marked_discrete());
                CHECK(!pre.model().constraint_ref(c1).marked_discrete());
                CHECK(pre.model().constraint_ref(c2).marked_discrete());
            }
        }
    }
}

TEST_CASE("Test normalization_remove_self_loops", "[presolve][impl]") {
    GIVEN("A CQM with a single integer self-loop") {
        auto cqm = ConstrainedQuadraticModel();
        auto v = cqm.add_variable(dimod::Vartype::INTEGER, -5, +5);
        cqm.objective.add_linear(v, 2);
        cqm.objective.add_quadratic(v, v, -7);
        cqm.objective.add_offset(3);

        WHEN("We give it to the presolver and run normalization_remove_self_loops()") {
            auto pre = PresolverImpl(cqm);
            pre.normalization_remove_self_loops();

            THEN("The single integer variable is now two") {
                REQUIRE(pre.model().num_variables() == 2);
                REQUIRE(pre.model().num_constraints() == 1);

                CHECK(pre.model().objective.linear(0) == 2.);
                CHECK(pre.model().objective.linear(1) == 0.);
                CHECK(pre.model().objective.quadratic(0, 0) == 0.);
                CHECK(pre.model().objective.quadratic(1, 1) == 0.);
                CHECK(pre.model().objective.quadratic(0, 1) == -7.);
                CHECK(pre.model().objective.offset() == 3.);
                CHECK(pre.model().vartype(v) == dimod::Vartype::INTEGER);
                CHECK(pre.model().lower_bound(v) == -5);
                CHECK(pre.model().upper_bound(v) == +5);

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

        WHEN("We give it to the presolver and run normalization_remove_self_loops()") {
            auto pre = PresolverImpl(cqm);
            pre.normalization_remove_self_loops();

            THEN("Three new variables and two constraints are added") {
                REQUIRE(pre.model().num_variables() == 6);
                REQUIRE(pre.model().num_constraints() == 5);
            }
        }
    }
}

TEST_CASE("Test normalization_spin_to_binary", "[presolve][impl]") {
    SECTION("Empty model") {
        auto pre = PresolverImpl(ConstrainedQuadraticModel());
        pre.normalization_spin_to_binary();
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

        WHEN("We give it to the presolver and run normalization_spin_to_binary()") {
            auto pre = PresolverImpl(cqm);
            pre.normalization_spin_to_binary();

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
