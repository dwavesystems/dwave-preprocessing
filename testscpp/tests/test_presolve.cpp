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
#include "dwave/presolve.h"

namespace dwave {

SCENARIO("constrained quadratic models can be presolved") {
    GIVEN("a cqm with some trivial issues") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();

        // v0 has default bounds, but a constraint restricting it
        auto v0 = cqm.add_variable(dimod::Vartype::INTEGER);
        auto c0 = cqm.add_linear_constraint({v0}, {1}, dimod::Sense::LE, 5);  // v0 <= 5

        // v1 has bounds that implicitly fix it
        auto v1 = cqm.add_variable(dimod::Vartype::INTEGER, 5, 5);

        // v2 has default bounds, but an equality constraint that fixes it
        auto v2 = cqm.add_variable(dimod::Vartype::INTEGER);
        auto c2 = cqm.add_linear_constraint({v2}, {1}, dimod::Sense::EQ, 7);

        // v3 has default bounds, but two inequality constraints that fix it
        auto v3 = cqm.add_variable(dimod::Vartype::INTEGER);
        auto c3a = cqm.add_linear_constraint({v3}, {1}, dimod::Sense::LE, 5.5);
        auto c3b = cqm.add_linear_constraint({v3}, {1}, dimod::Sense::GE, 4.5);

        WHEN("passed into a presolver") {
            auto presolver = presolve::Presolver<double>(std::move(cqm));

            THEN("it has moved correctly") {
                const auto& newcqm = presolver.model();

                REQUIRE(newcqm.num_variables() == 4);
                REQUIRE(newcqm.num_constraints() == 4);

                CHECK(newcqm.vartype(v0) == dimod::Vartype::INTEGER);
                CHECK(newcqm.lower_bound(v0) == 0);
                CHECK(newcqm.upper_bound(v0) ==
                      dimod::vartype_limits<double, dimod::Vartype::INTEGER>::default_max());
                CHECK(newcqm.constraint_ref(c0).linear(v0) == 1);
                CHECK(newcqm.constraint_ref(c0).sense() == dimod::Sense::LE);
                CHECK(newcqm.constraint_ref(c0).rhs() == 5);

                CHECK(newcqm.vartype(v1) == dimod::Vartype::INTEGER);
                CHECK(newcqm.lower_bound(v1) == 5);
                CHECK(newcqm.upper_bound(v1) == 5);

                CHECK(newcqm.vartype(v2) == dimod::Vartype::INTEGER);
                CHECK(newcqm.lower_bound(v2) == 0);
                CHECK(newcqm.upper_bound(v2) ==
                      dimod::vartype_limits<double, dimod::Vartype::INTEGER>::default_max());
                CHECK(newcqm.constraint_ref(c2).linear(v2) == 1);
                CHECK(newcqm.constraint_ref(c2).sense() == dimod::Sense::EQ);
                CHECK(newcqm.constraint_ref(c2).rhs() == 7);

                CHECK(newcqm.vartype(v3) == dimod::Vartype::INTEGER);
                CHECK(newcqm.lower_bound(v3) == 0);
                CHECK(newcqm.upper_bound(v3) ==
                      dimod::vartype_limits<double, dimod::Vartype::INTEGER>::default_max());
                CHECK(newcqm.constraint_ref(c3a).linear(v3) == 1);
                CHECK(newcqm.constraint_ref(c3a).sense() == dimod::Sense::LE);
                CHECK(newcqm.constraint_ref(c3a).rhs() == 5.5);
                CHECK(newcqm.constraint_ref(c3b).linear(v3) == 1);
                CHECK(newcqm.constraint_ref(c3b).sense() == dimod::Sense::GE);
                CHECK(newcqm.constraint_ref(c3b).rhs() == 4.5);
            }

            AND_WHEN("the default presolving is applied") {
                presolver.load_default_presolvers();
                presolver.apply();

                THEN("most of the constraints/variables are removed") {
                    CHECK(presolver.model().num_constraints() == 0);
                    CHECK(presolver.model().num_variables() == 1);
                }

                AND_WHEN("we then undo the transformation") {
                    auto original = presolver.postsolver().apply(std::vector<int>{3});
                    CHECK(original == std::vector<int>{3, 5, 7, 5});
                }
            }

            AND_WHEN("the model is detached") {
                auto newcqm = presolver.detach_model();

                THEN("it is copied correctly") {
                    REQUIRE(newcqm.num_variables() == 4);
                    REQUIRE(newcqm.num_constraints() == 4);

                    CHECK(newcqm.vartype(v0) == dimod::Vartype::INTEGER);
                    CHECK(newcqm.lower_bound(v0) == 0);
                    CHECK(newcqm.upper_bound(v0) ==
                          dimod::vartype_limits<double, dimod::Vartype::INTEGER>::default_max());
                    CHECK(newcqm.constraint_ref(c0).linear(v0) == 1);
                    CHECK(newcqm.constraint_ref(c0).sense() == dimod::Sense::LE);
                    CHECK(newcqm.constraint_ref(c0).rhs() == 5);

                    CHECK(newcqm.vartype(v1) == dimod::Vartype::INTEGER);
                    CHECK(newcqm.lower_bound(v1) == 5);
                    CHECK(newcqm.upper_bound(v1) == 5);

                    CHECK(newcqm.vartype(v2) == dimod::Vartype::INTEGER);
                    CHECK(newcqm.lower_bound(v2) == 0);
                    CHECK(newcqm.upper_bound(v2) ==
                          dimod::vartype_limits<double, dimod::Vartype::INTEGER>::default_max());
                    CHECK(newcqm.constraint_ref(c2).linear(v2) == 1);
                    CHECK(newcqm.constraint_ref(c2).sense() == dimod::Sense::EQ);
                    CHECK(newcqm.constraint_ref(c2).rhs() == 7);

                    CHECK(newcqm.vartype(v3) == dimod::Vartype::INTEGER);
                    CHECK(newcqm.lower_bound(v3) == 0);
                    CHECK(newcqm.upper_bound(v3) ==
                          dimod::vartype_limits<double, dimod::Vartype::INTEGER>::default_max());
                    CHECK(newcqm.constraint_ref(c3a).linear(v3) == 1);
                    CHECK(newcqm.constraint_ref(c3a).sense() == dimod::Sense::LE);
                    CHECK(newcqm.constraint_ref(c3a).rhs() == 5.5);
                    CHECK(newcqm.constraint_ref(c3b).linear(v3) == 1);
                    CHECK(newcqm.constraint_ref(c3b).sense() == dimod::Sense::GE);
                    CHECK(newcqm.constraint_ref(c3b).rhs() == 4.5);
                }

                THEN("the presolver is invalidated") { CHECK_THROWS(presolver.apply()); }
            }
        }
    }

    GIVEN("a cqm with some spin variables") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        cqm.add_variables(dimod::Vartype::SPIN, 5);
        for (size_t v = 0; v < 5; ++v) {
            for (size_t u = v + 1; u < 5; ++u) {
                cqm.objective.set_quadratic(u, v, 1);
            }
        }

        WHEN("the default presolving is applied") {
            auto presolver = presolve::Presolver<double>(std::move(cqm));
            presolver.load_default_presolvers();
            presolver.apply();

            THEN("most of the constraints/variables are removed") {
                CHECK(presolver.model().num_constraints() == 0);
                CHECK(presolver.model().num_variables() == 5);

                for (size_t v = 0; v < 5; ++v) {
                    CHECK(presolver.model().vartype(v) == dimod::Vartype::BINARY);
                }

                AND_WHEN("we then undo the transformation") {
                    auto original = presolver.postsolver().apply(std::vector<int>{0, 1, 0, 1, 0});
                    CHECK(original == std::vector<int>{-1, +1, -1, +1, -1});
                }
            }
        }
    }

    GIVEN("a CQM with self-loops") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        cqm.add_variables(dimod::Vartype::INTEGER, 5);

        cqm.objective.set_quadratic(0, 0, 1.5);
        cqm.objective.set_quadratic(3, 3, 3.5);

        cqm.add_constraint();
        cqm.constraint_ref(0).add_quadratic(4, 4, 5);
        cqm.constraint_ref(0).add_quadratic(3, 3, 6);

        WHEN("presolving is applied") {
            auto presolver = presolve::Presolver<double>(std::move(cqm));
            presolver.load_default_presolvers();
            presolver.apply();

            THEN("the self-loops are removed and an equality is added") {
                auto& model = presolver.model();

                REQUIRE(model.num_variables() == 8);
                REQUIRE(model.num_constraints() == 4);
                CHECK(model.objective.num_interactions() == 2);
                CHECK(model.objective.quadratic(0, 5) == 1.5);
                CHECK(model.objective.quadratic(3, 6) == 3.5);

                CHECK(model.constraint_ref(0).num_interactions() == 2);
                CHECK(model.constraint_ref(0).quadratic(4, 7) == 5);
                CHECK(model.constraint_ref(0).quadratic(3, 6) == 6);
            }

            AND_WHEN("we then undo the transformation") {
                auto original =
                        presolver.postsolver().apply(std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8});
                CHECK(original == std::vector<int>{1, 2, 3, 4, 5});
            }
        }
    }

    GIVEN("a CQM with a single self-loop") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        auto i = cqm.add_variable(dimod::Vartype::INTEGER);
        auto& constraint = cqm.constraint_ref(cqm.add_constraint());
        constraint.add_quadratic(i, i, 1);
        constraint.set_rhs(1);
        constraint.set_sense(dimod::Sense::LE);

        WHEN("presolving is applied") {
            auto presolver = presolve::Presolver<double>(std::move(cqm));
            presolver.load_default_presolvers();
            presolver.apply();

            THEN("an equality constraint is added") {
                auto& model = presolver.model();

                // v0 == v0
                CHECK(model.constraint_ref(1).num_variables() == 2);
                CHECK(model.constraint_ref(1).is_linear());
                CHECK(model.constraint_ref(1).variables() == std::vector<int>{0, 1});
                CHECK(model.constraint_ref(1).linear(0) + model.constraint_ref(1).linear(1) == 0);
                CHECK(model.constraint_ref(1).rhs() == 0);
                CHECK(model.constraint_ref(1).sense() == dimod::Sense::EQ);
            }
        }
    }

    GIVEN("a CQM with a constraint with no variables") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        cqm.add_variable(dimod::Vartype::BINARY);

        // 5 == 5 constraint
        auto& constraint = cqm.constraint_ref(cqm.add_constraint());
        constraint.set_offset(5);
        constraint.set_rhs(5);

        WHEN("we presolve is applied") {
            auto presolver = presolve::Presolver<double>(std::move(cqm));
            presolver.load_default_presolvers();
            presolver.apply();

            THEN("the constraint is removed") { CHECK(cqm.num_constraints() == 0); }
        }
    }

    GIVEN("a CQM with 5 binary variables") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        cqm.add_variables(dimod::Vartype::BINARY, 5);

        WHEN("we add a discrete constraint that has an offset but not a rhs") {
            auto& c1 = cqm.constraint_ref(cqm.add_constraint());
            c1.set_linear(0, 1);
            c1.set_linear(1, 1);
            c1.set_offset(-1);
            c1.mark_discrete();

            AND_WHEN("we presolve is applied") {
                auto presolver = presolve::Presolver<double>(std::move(cqm));
                presolver.load_default_presolvers();
                presolver.apply();

                THEN("the constraint is still marked as discrete") {
                    REQUIRE(presolver.model().num_constraints() == 1);
                    CHECK(presolver.model().constraint_ref(0).marked_discrete());
                }
            }
        }
        WHEN("we add a discrete constraint that has an offset and a rhs") {
            auto& c1 = cqm.constraint_ref(cqm.add_constraint());
            c1.set_linear(0, 1);
            c1.set_linear(1, 1);
            c1.set_rhs(1);
            c1.set_offset(-1);
            c1.mark_discrete();

            AND_WHEN("we presolve is applied") {
                auto presolver = presolve::Presolver<double>(std::move(cqm));
                presolver.load_default_presolvers();
                presolver.apply();

                THEN("the constraint is still marked as discrete") {
                    REQUIRE(presolver.model().num_constraints() == 1);
                    CHECK(!presolver.model().constraint_ref(0).marked_discrete());
                }
            }
        }

        WHEN("we add two overlapping discrete constraints and one non-overlapping") {
            auto& c1 = cqm.constraint_ref(cqm.add_constraint());
            c1.set_linear(0, 1);
            c1.set_linear(1, 1);
            c1.set_rhs(1);
            c1.mark_discrete();
            auto& c2 = cqm.constraint_ref(cqm.add_constraint());
            c2.set_linear(2, 1);
            c2.set_linear(1, 1);
            c2.set_rhs(1);
            c2.mark_discrete();
            auto& c3 = cqm.constraint_ref(cqm.add_constraint());
            c3.set_linear(3, 1);
            c3.set_linear(4, 1);
            c3.set_rhs(1);
            c3.mark_discrete();

            AND_WHEN("we presolve is applied") {
                auto presolver = presolve::Presolver<double>(std::move(cqm));
                presolver.load_default_presolvers();
                presolver.apply();

                THEN("two will still be marked as discrete") {
                    REQUIRE(presolver.model().num_constraints() == 3);
                    CHECK(presolver.model().constraint_ref(0).marked_discrete() !=
                          presolver.model().constraint_ref(1).marked_discrete());
                    CHECK(presolver.model().constraint_ref(2).marked_discrete());
                }
            }
        }
    }

    GIVEN("a CQM with one integer variable and two bound constraitns") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        cqm.add_variable(dimod::Vartype::INTEGER);  // unbounded

        auto& constraint =
                cqm.constraint_ref(cqm.add_linear_constraint({0}, {1}, dimod::Sense::LE, 5));
        constraint.set_weight(5);  // make soft

        CHECK(constraint.is_soft());

        cqm.add_linear_constraint({0}, {1}, dimod::Sense::LE, 10);  // hard constraint

        WHEN("we presolve is applied") {
            auto presolver = presolve::Presolver<double>(std::move(cqm));
            presolver.load_default_presolvers();
            presolver.apply();

            THEN("the hard constraint is removed") {
                REQUIRE(presolver.model().num_constraints() == 1);
                CHECK(presolver.model().upper_bound(0) == 10);
            }
        }
    }
}

TEST_CASE("experiment") {
    GIVEN("a CQM with a bunch of constraints") {
        auto cqm = dimod::ConstrainedQuadraticModel<double>();
        cqm.add_constraints(10);

        auto presolver = presolve::Presolver<double>(std::move(cqm));
        presolver.apply_parallel();
    }
}

}  // namespace dwave
