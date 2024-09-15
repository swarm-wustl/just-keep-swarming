// Copyright 2024 Sebastian Theiler
#pragma once

#include <stdlib.h>

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

using std::vector, std::unordered_map;

// Get unoccupied cardinal neighbors of a cell in a map
vector<Cell> get_neighbors(const Map &map, const Cell &pos);

// heuristic
inline int h(const Cell &a, const Cell &b) {
  return abs(static_cast<int>(a.x) - static_cast<int>(b.x)) +
         abs(static_cast<int>(a.y) -
             static_cast<int>(b.y));  // manhattan distance
}

// Reconstruct path from unordered_map
vector<Cell> reconstruct_path(const unordered_map<Cell, Cell> &came_from,
                              Cell current);

// A* search algorithm
vector<Cell> astar(const Cell &start, const Cell &goal, const Map &map,
                   const vector<vector<Cell>> &map_occ = {});

}  // namespace control_algorithms
