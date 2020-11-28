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

typedef capacity_type long long int;

namespace {

class compClass {
public:
  bool operator()(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    if (a.second != b.second)
      return !(a.second < b.second);
    else
      return a.first < b.first;
  }
};

template <class PosiformInfo>
void fixQuboVariables(PosiformInfo &posiform_info, int num_bqm_variables,
                      bool sample,
                      std::vector<std::pair<int, int>> &fixed_variables) {
  ImplicationNetwork<capacity_type> implication_network(posiform_info);
  fixed_variables.reserve(num_bqm_variables);
  std::vector<std::pair<int, int>> fixed_variables_posiform;
  implication_network.fixVariables(fixed_variables_posiform, sample);

  for (int i = 0; i < fixed_variables_posiform.size(); i++) {
    int bqm_variable =
        pi.mapVariablePosiformToQubo(fixed_variables_posiform[i].first);
    fixed_variables.push_back(
        {bqm_variable, fixed_variables_posiform[i].second});
  }

  if (!sample) {
    for (int bqm_variable = 0; bqm_variable < numVars; bqm_variable++) {
      if (pi.mapVariableQuboToPosiform(bqm_variable) < 0) {
        fixed_variables.push_back({bqm_variable, 1});
      }
    }
  }

  std::sort(fixed_variables.begin(), fixed_variables.end(), compClass());
}

template <class V, class B>
std::vector<std::pair<int, int>>
fixQuboVariables(dimod::AdjVectorBQM<V, B> &bqm, bool sample) {
  int num_bqm_variables = bqm.num_variables();
  PosiformInfo<dimod::AdjVectorBQM<V, B>, capacity_type> posiform_info(bqm);
  fixQuboVariables(posiform_info, num_bqm_variables, sample, fixed_variables);
  return fixed_variables;
}

template <class V, class B>
std::vector<std::pair<int, int>> fixQuboVariables(dimod::AdjArrayBQM<V, B> &bqm,
                                                  bool sample) {
  int num_bqm_variables = bqm.num_variables();
  PosiformInfo<dimod::AdjVectorBQM<V, B>, capacity_type> posiform_info(bqm);
  fixQuboVariables(posiform_info, num_bqm_variables, sample, fixed_variables);
  return fixed_variables;
}

template <class V, class B>
std::vector<std::pair<int, int>> fixQuboVariables(dimod::AdjMapBQM<V, B> &bqm,
                                                  bool sample) {
  int num_bqm_variables = bqm.num_variables();
  PosiformInfo<dimod::AdjVectorBQM<V, B>, capacity_type> posiform_info(bqm);
  fixQuboVariables(posiform_info, num_bqm_variables, sample, fixed_variables);
  return fixed_variables;
}

} // namespace
