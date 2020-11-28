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

#ifndef HELPER_GRAPH_ALGORITHM_HPP_INCLUDED
#define HELPER_GRAPH_ALGORITHM_HPP_INCLUDED

#include <algorithm>
#include <atomic>
#include <iostream>
#include <vector>

#include "helper_data_structures.hpp"

// Check if the flow value in a given graph represented as an adjacency list is
// valid or not.
// @param adjacency_list the adjacency list of the flow network to test for.
// @source the source considered for calcualting flow.
// @sink the sink considered for calculating flow.
// @return std::pair<the value of flow, if flow is valid or not>
template <class EdgeType>
std::pair<typename EdgeType::capacity_type, bool>
isFlowValid(std::vector<std::vector<EdgeType>> &adjacency_list, int source,
            int sink) {
  using capacity_t = typename EdgeType::capacity_type;
  bool valid_flow = true;
  int num_vertices = adjacency_list.size();
  std::vector<capacity_t> excess(num_vertices, 0);

  std::cout << "Validating flow of flow network ..." << std::endl;

  double overflow_detector = 0;
  auto eit = adjacency_list[source].begin();
  auto eit_end = adjacency_list[source].end();
  for (; eit != eit_end; eit++) {
    overflow_detector += eit->getCapacity();
  }

  if (overflow_detector >
      static_cast<double>(std::numeric_limits<capacity_t>::max())) {
    std::cout << "WARNING : Overflow is possible in the flow network. Sum of "
                 "capacity of out edges from source exceed numeric limit of "
                 "the type used for capacity."
              << std::endl;
  }

  // Since we are validating our algorithms, we will not retrieve the
  // value of residual/capacity of a reverse edge from its counterpart
  // which we generally do for performance reasons. Here we will actually
  // access the data and verify if the flow constraints hold or not.
  for (int from_vertex = 0; from_vertex < num_vertices; from_vertex++) {
    auto eit = adjacency_list[from_vertex].begin();
    auto eit_end = adjacency_list[from_vertex].end();
    for (; eit != eit_end; eit++) {
      int to_vertex = eit->to_vertex;
      auto reverse_eit =
          adjacency_list[to_vertex].begin() + eit->reverse_edge_index;
      capacity_t edge_capacity = eit->getCapacity();
      capacity_t edge_residual = eit->residual;
      capacity_t reverse_edge_capacity = reverse_eit->getCapacity();
      capacity_t reverse_edge_residual = reverse_eit->residual;
      bool valid_edge =
          (eit->getReverseEdgeCapacity() == reverse_edge_capacity) &&
          (eit->getReverseEdgeResidual() == reverse_edge_residual) &&
          (edge_capacity >= 0) && (edge_residual >= 0);
      if (edge_capacity > 0) {
        // Residual edge having capacity 0 is a valid assumption for posiforms,
        // since no term with two variables appear multiple times with different
        // ordering of the variables. This assumption can be maintained with
        // other graphs too.
        valid_edge = valid_edge && (reverse_edge_capacity == 0) &&
                     (edge_residual <= edge_capacity) &&
                     ((edge_residual + reverse_edge_residual) == edge_capacity);

        capacity_t flow = (edge_capacity - edge_residual);
        excess[from_vertex] -= flow;
        excess[to_vertex] += flow;
      }
      if (!valid_edge) {
        std::cout << "Invalid Flow due to following edge pair :" << std::endl;
        eit->print();
        reverse_eit->print();
      }
      valid_flow = valid_flow && valid_edge;
    }
  }

  for (int vertex = 0; vertex < num_vertices; vertex++) {
    if ((vertex == source) || (vertex == sink)) {
      continue;
    }
    if (excess[vertex]) {
      std::cout << "Excess flow of " << excess[vertex]
                << " in vertex : " << vertex << std::endl;
      valid_flow = false;
    }
  }

  if (excess[sink] != -excess[source]) {
    std::cout << "Flow out of source : " << -excess[source]
              << " is not equal to flow into sink : " << excess[sink]
              << std::endl;
    std::cout << "Difference is : "
              << std::llabs(std::llabs(excess[source]) -
                            std::llabs(excess[sink]))
              << std::endl;
    valid_flow = false;
  }

  return {excess[sink], valid_flow};
}

// Perform breadth first search on a residual graph from a certain vertex, a
// depth equal to  the number of vertices means that vertex could not be reached
// from the start_vertex, since the maximum depth can be equal to number of
// vertices -1.
template <class EdgeType>
int breadthFirstSearchResidual(
    std::vector<std::vector<EdgeType>> &adjacency_list, int start_vertex,
    std::vector<int> &depth_values, bool reverse = false,
    bool print_result = false) {
  using capacity_t = typename EdgeType::capacity_type;
  int num_vertices = adjacency_list.size();
  int UNVISITED = num_vertices;
  vector_based_queue<int> vertex_queue(num_vertices);
  depth_values.resize(num_vertices);
  std::fill(depth_values.begin(), depth_values.end(), UNVISITED);

  depth_values[start_vertex] = 0;
  vertex_queue.push(start_vertex);

  // The check for whether the search should be reverse or not could be done
  // inside the innermost loop, but that would be detrimental to performance.
  if (reverse) {
    while (!vertex_queue.empty()) {
      int v_parent = vertex_queue.pop();
      int current_depth = depth_values[v_parent] + 1;
      auto eit = adjacency_list[v_parent].begin();
      auto eit_end = adjacency_list[v_parent].end();
      for (; eit != eit_end; eit++) {
        int to_vertex = eit->to_vertex;
        if (eit->getReverseEdgeResidual() &&
            depth_values[to_vertex] == UNVISITED) {
          depth_values[to_vertex] = current_depth;
          vertex_queue.push(to_vertex);
        }
      }
    }
  } else {
    while (!vertex_queue.empty()) {
      int v_parent = vertex_queue.pop();
      int current_depth = depth_values[v_parent] + 1;
      auto eit = adjacency_list[v_parent].begin();
      auto eit_end = adjacency_list[v_parent].end();
      for (; eit != eit_end; eit++) {
        int to_vertex = eit->to_vertex;
        if (eit->residual && depth_values[to_vertex] == UNVISITED) {
          depth_values[to_vertex] = current_depth;
          vertex_queue.push(to_vertex);
        }
      }
    }
  }

  if (print_result) {
    std::vector<int> level_sizes;
    std::vector<std::vector<int>> levels;
    levels.resize(num_vertices + 1);
    level_sizes.resize(num_vertices + 1, 0);
    for (int i = 0; i < depth_values.size(); i++) {
      level_sizes[depth_values[i]]++;
    }
    for (int i = 0; i < level_sizes.size(); i++) {
      levels[i].reserve(level_sizes[i]);
    }
    for (int i = 0; i < depth_values.size(); i++) {
      levels[depth_values[i]].push_back(i);
    }
    std::cout << std::endl;
    std::cout << "Printing " << (reverse ? "reverse " : "")
              << "breadth first search result starting from vertex : "
              << start_vertex << std::endl;
    std::cout << std::endl;
    for (int i = 0; i < levels.size(); i++) {
      if (!levels[i].size()) {
        continue;
      }
      std::cout << "Level " << i << " has " << levels[i].size()
                << " vertices : " << std::endl;
      for (int j = 0; j < levels[i].size(); j++) {
        std::cout << levels[i][j] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  return UNVISITED;
}

// Check if the flow value in a given graph represented as an adjacency list is
// a valid max-flow or not and also return the flow value.
template <class EdgeType>
std::pair<typename EdgeType::capacity_type, bool>
isMaximumFlow(std::vector<std::vector<EdgeType>> &adjacency_list, int source,
              int sink) {

  // If the flow follows the constraints of network flow.
  auto validity_result = isFlowValid(adjacency_list, source, sink);

  // If the flow is a maximum flow, the source will be unreachable from the sink
  // through a reverse breadth first search, meaning the source cannot reach the
  // sink through any augmenting path.
  std::vector<int> depth_values;
  int num_vertices = adjacency_list.size();
  int UNVISITED = breadthFirstSearchResidual(adjacency_list, sink, depth_values,
                                             true, true);
  return {validity_result.first,
          (validity_result.second && (depth_values[source] == UNVISITED))};
}

// Tarzan's strongly connected component algoirhtm using iterative depth first
// search. For more detail on the iterative version of the depth first search,
// look at the comments in the iterative depth first search in the
// convertPreflowToFlow function in push_relabel.hpp.
int stronglyConnectedComponents(std::vector<std::vector<int>> &adjacency_list,
                                std::vector<int> &components) {
  int num_vertices = adjacency_list.size();
  using edge_iterator = typename std::vector<int>::iterator;
  components.resize(num_vertices);
  vector_based_stack<int> component_stack(num_vertices);
  std::vector<bool> in_stack(num_vertices, false);
  int UNVISITED = num_vertices;
  std::vector<int> low_link_id(num_vertices, UNVISITED);
  std::vector<int> vertex_visit_id(num_vertices, UNVISITED);
  std::vector<int> parent(num_vertices);
  int visit_id = 0;
  int num_strong_components = 0;
  std::vector<std::pair<edge_iterator, edge_iterator>> pending_out_edges(
      num_vertices);

  for (int vertex = 0; vertex < num_vertices; vertex++) {
    pending_out_edges[vertex] = {adjacency_list[vertex].begin(),
                                 adjacency_list[vertex].end()};
  }

  // Iterative DFS.
  for (int current_vertex = 0; current_vertex < num_vertices;
       current_vertex++) {
    if (vertex_visit_id[current_vertex] == UNVISITED) {

      // The root of a DFS tree.
      int root_vertex = current_vertex;
      vertex_visit_id[current_vertex] = visit_id;
      low_link_id[current_vertex] = visit_id;
      visit_id++;
      component_stack.push(current_vertex);
      in_stack[current_vertex] = true;
      while (true) {
        for (; pending_out_edges[current_vertex].first !=
               pending_out_edges[current_vertex].second;
             pending_out_edges[current_vertex].first++) {
          int child_vertex = *(pending_out_edges[current_vertex].first);
          if (vertex_visit_id[child_vertex] == UNVISITED) {
            vertex_visit_id[child_vertex] = visit_id;
            low_link_id[child_vertex] = visit_id;
            visit_id++;
            component_stack.push(child_vertex);
            in_stack[child_vertex] = true;

            // Recursive DFS call.
            parent[child_vertex] = current_vertex;
            current_vertex = child_vertex;
            break;
          } else if (in_stack[child_vertex]) {
            low_link_id[current_vertex] = std::min(low_link_id[current_vertex],
                                                   low_link_id[child_vertex]);
          }
        }

        // Finished exploring current vertex, edges.
        if (pending_out_edges[current_vertex].first ==
            pending_out_edges[current_vertex].second) {
          if (low_link_id[current_vertex] == vertex_visit_id[current_vertex]) {
            int popped_vertex = -1;
            do {
              popped_vertex = component_stack.pop();
              in_stack[popped_vertex] = false;
              components[popped_vertex] = num_strong_components;
            } while (popped_vertex != current_vertex);
            num_strong_components++;
          }
          if (current_vertex != root_vertex) {
            int completed_vertex = current_vertex;
            current_vertex = parent[current_vertex];

            // This is the DFS callback, here the recursive function has
            // returned and we can perform the comparison with the low link id
            // of the child node.
            low_link_id[current_vertex] = std::min(
                low_link_id[current_vertex], low_link_id[completed_vertex]);
            pending_out_edges[current_vertex].first++;
          } else {
            break;
          }
        }
      }
    }
  }
  return num_strong_components;
}

void getTransposedAdjacencyList(std::vector<std::vector<int>> original,
                                std::vector<std::vector<int>> &transposed) {
  int num_vertices = original.size();
  std::atomic<int> *transposed_sizes =
      (std::atomic<int> *)malloc(num_vertices * sizeof(std::atomic<int>));
  for (int i = 0; i < num_vertices; i++) {
    transposed_sizes[i] = 0;
  }
  transposed.resize(num_vertices);

#pragma omp parallel for
  for (int i = 0; i < num_vertices; i++) {
    auto eit = original[i].begin();
    auto eit_end = original[i].end();
    for (; eit != eit_end; eit++) {
      transposed_sizes[*eit]++;
    }
  }

  for (int i = 0; i < num_vertices; i++) {
    transposed[i].reserve(transposed_sizes[i]);
  }

  for (int i = 0; i < num_vertices; i++) {
    auto eit = original[i].begin();
    auto eit_end = original[i].end();
    for (; eit != eit_end; eit++) {
      transposed[*eit].push_back(i);
    }
  }

  free(transposed_sizes);
}

// Perform breadth first search from a certain vertex, a depth equal to  the
// number of vertices means that vertex could not be reached from the
// start_vertex, since the maximum depth can be equal to number of vertices -1.
int breadthFirstSearch(std::vector<std::vector<int>> &adjacency_list,
                       int start_vertex, std::vector<int> &depth_values,
                       bool print_result = false) {
  int num_vertices = adjacency_list.size();
  int UNVISITED = num_vertices;
  vector_based_queue<int> vertex_queue(num_vertices);
  depth_values.resize(num_vertices);
  std::fill(depth_values.begin(), depth_values.end(), UNVISITED);

  depth_values[start_vertex] = 0;
  vertex_queue.push(start_vertex);

  while (!vertex_queue.empty()) {
    int v_parent = vertex_queue.pop();
    int current_depth = depth_values[v_parent] + 1;
    auto eit = adjacency_list[v_parent].begin();
    auto eit_end = adjacency_list[v_parent].end();
    for (; eit != eit_end; eit++) {
      int to_vertex = *eit;
      if (depth_values[to_vertex] == UNVISITED) {
        depth_values[to_vertex] = current_depth;
        vertex_queue.push(to_vertex);
      }
    }
  }
  return UNVISITED;
}

// We create the graph of strongly connected graphs, while processing each
// strongly connected component in parallel. Given the sizes of different
// components may vary widely, parallelism applied in this fashion is not the
// most efficient but if we try to parallelize over all the vertices of the
// residual graph where all threads would write into the adjacency matrix of
// strongly connected components whether the edges of a vertex contribute an
// entry to that matrix or not, it would require us to allocate that adjacency
// matrix which may be very large if the number of strongly connected components
// is nearly equal to the number of vertices and we would risk memory allocation
// failure.
void createGraphOfStronglyConnectedComponents(
    std::vector<int> &vertex_to_component_map,
    std::vector<std::vector<int>> &components,
    std::vector<std::vector<int>> &adjacency_list_residual,
    std::vector<std::vector<int>> &adjacency_list_components) {
  int num_components = components.size();
  adjacency_list_components.resize(num_components);
#pragma omp parallel
  {
    std::vector<int> temp_buffer(num_components);
    std::vector<bool> has_edge_to_component(num_components, false);
#pragma omp for
    for (int component = 0; component < num_components; component++) {
      int num_out_edges = 0;
      auto vit = components[component].begin();
      auto vit_end = components[component].end();
      for (; vit != vit_end; vit++) {
        int vertex = *vit;
        auto eit = adjacency_list_residual[vertex].begin();
        auto eit_end = adjacency_list_residual[vertex].end();
        for (; eit != eit_end; eit++) {
          int to_component = vertex_to_component_map[*eit];
          if ((to_component != component) &&
              (!has_edge_to_component[to_component])) {
            has_edge_to_component[to_component] = true;
            temp_buffer[num_out_edges++] = to_component;
          }
        }
      }

      adjacency_list_components[component].assign(
          temp_buffer.begin(), temp_buffer.begin() + num_out_edges);

      // We have to reinitialize the hash table, instead of using std::clear
      // this is faster as we clear only the set elements.
      for (int i = 0; i < num_out_edges; i++) {
        has_edge_to_component[temp_buffer[i]] = false;
      }
    }
  }
}

#endif // HELPER_GRAPH_ALGORITHM_HPP_INCLUDED
