// Copyright 2024 Sebastian Theiler
#pragma once

#include <vector>

#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

using std::vector;

// Update the occupancy map used in prioritized planning
vector<vector<Cell>> update_map_occ(const vector<vector<Cell>> &map_occ,
                                    const vector<Cell> &plan, int robot_index,
                                    const vector<Cell> &start,
                                    int time_offset = 0);

// Prioritized planning search algorithm
vector<vector<Cell>> pplan(const vector<Cell> &start, const vector<Cell> &goal,
                           const Map &map);

}  // namespace control_algorithms
