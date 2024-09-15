// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/sstar.hpp"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

using std::vector, std::unordered_set, std::unordered_map, std::priority_queue;

vector<vector<Cell>> get_net_neighbors(const Map &map,
                                       const vector<Cell> &state) {
  vector<vector<Cell>> combinations;
  if (state.size() == 0) {
    return {{}};
  }

  vector<Cell> neighbors = get_neighbors(map, state[0]);
  neighbors.push_back(state[0]);  // include stationary move

  vector<vector<Cell>> sub_combinations =
      get_net_neighbors(map, vector<Cell>(state.begin() + 1, state.end()));

  for (auto &neighbor : neighbors) {
    for (auto &sub_comb : sub_combinations) {
      vector<Cell> new_combination = {neighbor};
      new_combination.insert(new_combination.end(), sub_comb.begin(),
                             sub_comb.end());

      // Check collisions
      unordered_set s(new_combination.begin(), new_combination.end());
      bool collisions = s.size() != new_combination.size();

      // TODO(sebtheiler) (this is bad, both inefficient and makes the algorithm
      // not always optimal)
      for (size_t i = 0; i < state.size(); i++) {
        auto it = find(state.cbegin(), state.cend(), new_combination[i]);
        if (it != state.end() &&
            std::distance(state.cbegin(), it) != static_cast<int>(i)) {
          collisions = true;
          break;
        }
      }

      if (!collisions) combinations.push_back(new_combination);
    }
  }

  return combinations;
}

int count_moved(const vector<Cell> &original, const vector<Cell> &changed) {
  int num_moved = 0;
  for (size_t i = 0; i < original.size(); i++) {
    if (original[i] != changed[i]) num_moved++;
  }

  return num_moved;
}

vector<vector<Cell>> reconstruct_multi_path(
    const unordered_map<vector<Cell>, vector<Cell>> &came_from,
    vector<Cell> current) {
  vector<vector<Cell>> path;
  while (came_from.at(current) != current) {
    path.push_back(current);
    current = came_from.at(current);
  }
  path.push_back(current);
  std::reverse(path.begin(), path.end());
  return path;
}

vector<vector<Cell>> sstar(const vector<Cell> &start, const vector<Cell> &goal,
                           const Map &map) {
  using PriorityQueue =
      priority_queue<PriorityVectorCell, vector<PriorityVectorCell>,
                     ComparePriorityVectorCell>;
  PriorityQueue frontier;
  unordered_map<vector<Cell>, vector<Cell>> came_from;
  unordered_map<vector<Cell>, int> cost_so_far;
  PriorityVectorCell pstart;

  pstart.cells = start;
  pstart.priority = 0;
  frontier.push(pstart);

  came_from[start] = start;
  cost_so_far[start] = 0;

  int n = static_cast<int>(start.size());

  if (invalid_multi_start(map, start, goal)) {
    std::cout << "INVALID STARTING CONDITIONS" << std::endl;
    return {};
  }

  while (!frontier.empty()) {
    vector<Cell> current = frontier.top().cells;
    frontier.pop();

    // Check if found end condition
    for (int i = 0; i < n; i++) {
      if (current[i].x != goal[i].x || current[i].y != goal[i].y) {
        break;
      } else if (i == n - 1) {
        std::cout << "FOUND GOAL" << std::endl;
        return reconstruct_multi_path(came_from, current);
      }
    }

    auto net_neighbors = get_net_neighbors(map, current);
    for (auto &next : net_neighbors) {
      int new_cost = cost_so_far[current] + count_moved(current, next);

      if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        came_from[next] = current;

        int sum_h = 0;
        for (int i = 0; i < n; i++) {
          sum_h += h(next[i], goal[i]);
        }
        int priority = new_cost + sum_h;
        PriorityVectorCell pnext;
        pnext.cells = next;
        pnext.priority = priority;
        frontier.push(pnext);
        came_from[next] = current;
      }
    }
  }

  // no path found
  std::cout << "FAILED TO FIND GOAL" << std::endl;
  return {};
}

}  // namespace control_algorithms
