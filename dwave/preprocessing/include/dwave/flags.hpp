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

enum PresolveStatus : std::uint8_t {
    Normalized = 1 << 0,          //< The normalization step has been completed
    PartiallyPresolved = 1 << 1,  //< Presolve is still in progress or was interrupted
    Presolved = 1 << 2,           //< Presolve finished without hitting the time or round limits

    Infeasible = 1 << 6,  ///< The model is infeasible
    Detached = 1 << 7,    ///< The model has been detached from the presolver
};

inline PresolveStatus operator|(PresolveStatus a, PresolveStatus b) {
    return static_cast<PresolveStatus>(static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b));
}

enum TechniqueFlags : std::uint64_t {
    // todo: individual technique flags

    All = 0xffffffffffffffffu
};

}  // namespace dwave::presolve
