// Copyright 2024 Sebastian Theiler
#pragma once

#include <unordered_map>
#include <vector>

#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

using std::vector, std::unordered_map;

// Calculate the net neighbors function for a state
vector<vector<Cell>> get_net_neighbors(const Map &map,
                                       const vector<Cell> &state);

// Count the number of changes between two states of robots
int count_moved(const vector<Cell> &original, const vector<Cell> &changed);

vector<vector<Cell>> reconstruct_multi_path(
    const unordered_map<vector<Cell>, vector<Cell>> &came_from,
    vector<Cell> current);

// S* search algorithm
vector<vector<Cell>> sstar(const vector<Cell> &start, const vector<Cell> &goal,
                           const Map &map);

}  // namespace control_algorithms
