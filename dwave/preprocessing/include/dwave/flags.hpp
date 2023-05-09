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

#include <cstdint>

namespace dwave::presolve {

enum Feasibility {
    Unknown,     //< It is not known if the model is feasible or not
    Infeasible,  //< The model is known to be infeasible
    Feasible,    //< The model is known to be feasible
};

enum TechniqueFlags : std::uint64_t {
    None = 0,

    // todo: individual technique flags

    All = 0xffffffffffffffffu,
};

}  // namespace dwave::presolve
