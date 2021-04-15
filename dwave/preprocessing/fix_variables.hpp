/**
# Copyright 2018 D-Wave Systems Inc.
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
#
#
================================================================================================
*/

#include "dimod/adjarraybqm.h"
#include "dimod/adjmapbqm.h"
#include "dimod/adjvectorbqm.h"
#include "implication_network.hpp"
#include "posiform_info.hpp"

#include <iostream>
using std::cout;
using std::endl;

namespace fix_variables_ {

typedef long long int capacity_type;

class compClass {
public:
  bool operator()(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    if (a.second != b.second)
      return !(a.second < b.second);
    else
      return a.first < b.first;
  }
};

// Fixes the QUBO variables.
// When sample is set to true, it fixes only the variables which correspond to
// strong persistencies. When sample is set to false, it tries to fix all the
// variables corresponding to strong and weak persistencies. The variables which
// do not contribute any coefficient to the posiform, either because their bias
// in the QUBO/BQM is zero or they are flushed to zeroes when converted to the
// posiform are set to 1 when sample is set to false.
template <class PosiformInfo>
void fixQuboVariables(PosiformInfo &posiform_info, int num_bqm_variables,
                      bool sample,
                      std::vector<std::pair<int, int>> &fixed_variables) {
  ImplicationNetwork<capacity_type> implication_network(posiform_info);
  fixed_variables.reserve(num_bqm_variables);

  // Fix the variables with respect to the posiform.
  std::vector<std::pair<int, int>> fixed_variables_posiform;
  implication_network.fixVariables(fixed_variables_posiform, sample);

  // There may not be 1 to 1 mapping from bqm variables to posiform variables,
  // so we convert the posiform variables back to bqm variables.
  for (int i = 0; i < fixed_variables_posiform.size(); i++) {
    int bqm_variable = posiform_info.mapVariablePosiformToQubo(
        fixed_variables_posiform[i].first);
    fixed_variables.push_back(
        {bqm_variable, fixed_variables_posiform[i].second});
  }

  // When we are not sampling we want to set the variables which did not
  // contribute to the posiform as they had zero bias. They can be set to either
  // 1 or 0. We choose 1.
  if (!sample) {
    for (int bqm_variable = 0; bqm_variable < num_bqm_variables;
         bqm_variable++) {
      if (posiform_info.mapVariableQuboToPosiform(bqm_variable) < 0) {
        fixed_variables.push_back({bqm_variable, 1});
      }
    }
  }

  std::sort(fixed_variables.begin(), fixed_variables.end(), compClass());
}

// Fix variables for different types of BQMs.

template <class V, class B>
std::vector<std::pair<int, int>>
fixQuboVariables(dimod::AdjVectorBQM<V, B> &bqm, bool sample) {
  int num_bqm_variables = bqm.num_variables();
  PosiformInfo<dimod::AdjVectorBQM<V, B>, capacity_type> posiform_info(bqm);
  std::vector<std::pair<int, int>> fixed_variables;
  fixQuboVariables(posiform_info, num_bqm_variables, sample, fixed_variables);
  return fixed_variables;
}

template <class V, class B>
std::vector<std::pair<int, int>> fixQuboVariables(dimod::AdjArrayBQM<V, B> &bqm,
                                                  bool sample) {
  int num_bqm_variables = bqm.num_variables();
  PosiformInfo<dimod::AdjArrayBQM<V, B>, capacity_type> posiform_info(bqm);
  std::vector<std::pair<int, int>> fixed_variables;
  fixQuboVariables(posiform_info, num_bqm_variables, sample, fixed_variables);
  return fixed_variables;
}

template <class V, class B>
std::vector<std::pair<int, int>> fixQuboVariables(dimod::AdjMapBQM<V, B> &bqm,
                                                  bool sample) {
  int num_bqm_variables = bqm.num_variables();
  PosiformInfo<dimod::AdjMapBQM<V, B>, capacity_type> posiform_info(bqm);
  std::vector<std::pair<int, int>> fixed_variables;
  fixQuboVariables(posiform_info, num_bqm_variables, sample, fixed_variables);
  return fixed_variables;
}

} // namespace fix_variables_
