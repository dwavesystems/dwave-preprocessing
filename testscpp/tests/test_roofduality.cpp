// Copyright 2021 D-Wave Systems Inc.
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
//
// =============================================================================

#include "../Catch2/single_include/catch2/catch.hpp"
#include <dimod/adjvectorbqm.h>

#include "dwave-preprocessing/fix_variables.hpp"

namespace fix_variables_ {

TEST_CASE("Tests for fixQuboVariables", "[roofduality]") {
    SECTION("Test simple case") {
        float Q [9] = {22,-4, 0,
                        0, 0, 4,
                        0, 0,-2};

        int num_vars = 3;
        auto bqm = dimod::AdjVectorBQM<int, float>(Q, num_vars);

        // Checking fixQuboVariables(PosiformInfo, ..)
        PosiformInfo<dimod::AdjVectorBQM<int, float>, capacity_type> posiform(bqm);
        std::vector<std::pair<int, int>> fixed_vars;
        fixQuboVariables(posiform, num_vars, true, fixed_vars);

        // Checking fixQuboVariables(AdjVectorBQM, ..)
        auto fixed_vars2 = fixQuboVariables(bqm, true);

        REQUIRE(fixed_vars == fixed_vars2);
        REQUIRE(fixed_vars.size() == num_vars);

        for (auto var : fixed_vars) {
            if (var.first == 2) {
                REQUIRE(var.second == 1);
            }
            else {
                REQUIRE(var.second == 0);
            }
        }
    }

    SECTION("Test zero bias case") {
        float Q [9] = {1,0,0,0,0,0,0,0,0};

        int num_vars = 3;
        auto bqm = dimod::AdjVectorBQM<int, float>(Q, num_vars);

        auto fixed_vars = fixQuboVariables(bqm, true);
        REQUIRE(fixed_vars.size() == 1);
        REQUIRE(fixed_vars[0].first == 0);
        REQUIRE(fixed_vars[0].second == 0);

        fixed_vars = fixQuboVariables(bqm, false);
        REQUIRE(fixed_vars.size() == 3);
        for (auto var : fixed_vars) {
            if (var.first == 0) {
                REQUIRE(var.second == 0);
            }
            else {
                // variables that didn't contribute should be set to 1
                REQUIRE(var.second == 1);
            }
        }
    }
}

TEST_CASE("Tests for PosiformInfo", "[roofduality]") {
    SECTION("Test basic case") {
        float Q [9] = {22,-4, 0,
                        0, 0, 4,
                        0, 0,-2};

        int num_vars = 3;
        auto bqm = dimod::AdjVectorBQM<int, float>(Q, num_vars);

        PosiformInfo<dimod::AdjVectorBQM<int, float>, capacity_type> posiform(bqm);
        REQUIRE(posiform.getNumVariables() == num_vars);
        REQUIRE(posiform.getNumLinear() == 2);

        REQUIRE(posiform.getNumQuadratic(0) == 1);
        REQUIRE(posiform.getNumQuadratic(1) == 2);
        REQUIRE(posiform.getNumQuadratic(2) == 1);

        REQUIRE(posiform.getLinear(0) > 0);
        REQUIRE(posiform.getLinear(1) == 0);
        REQUIRE(posiform.getLinear(2) < 0);

        auto span = posiform.getQuadratic(0);
        REQUIRE(std::distance(span.first, span.second) == 1);
        REQUIRE(span.first->first == 1); // neighbor var
        REQUIRE(span.first->second == -4); // quad bias

        span = posiform.getQuadratic(1);
        REQUIRE(std::distance(span.first, span.second) == 1);
        REQUIRE(span.first->first == 2); // neighbor var
        REQUIRE(span.first->second == 4); // quad bias

        span = posiform.getQuadratic(2);
        REQUIRE(std::distance(span.first, span.second) == 0);

        REQUIRE(posiform.mapVariableQuboToPosiform(0) == posiform.mapVariablePosiformToQubo(0));
        REQUIRE(posiform.mapVariableQuboToPosiform(1) == posiform.mapVariablePosiformToQubo(1));
        REQUIRE(posiform.mapVariableQuboToPosiform(2) == posiform.mapVariablePosiformToQubo(2));
    }

    SECTION("Test zero bias case") {
        float Q [9] = {1,0,0,0,0,0,0,0,0};

        int num_vars = 3;
        auto bqm = dimod::AdjVectorBQM<int, float>(Q, num_vars);
        PosiformInfo<dimod::AdjVectorBQM<int, float>, capacity_type> posiform(bqm);
        
        REQUIRE(posiform.getNumVariables() == 1);

        REQUIRE(posiform.getLinear(0) > 0);
        
        auto span = posiform.getQuadratic(0);
        REQUIRE(std::distance(span.first, span.second) == 0);

        REQUIRE(posiform.mapVariableQuboToPosiform(0) == posiform.mapVariablePosiformToQubo(0));
        REQUIRE(posiform.mapVariableQuboToPosiform(1) == -1);
        REQUIRE(posiform.mapVariableQuboToPosiform(2) == -1);
    }
}

}  // namespace fix_variables_
