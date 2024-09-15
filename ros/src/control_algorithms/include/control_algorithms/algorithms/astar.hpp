// Copyright 2024 Sebastian Theiler
#pragma once

#include <stdlib.h>

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace control_algorithms {

using std::vector, std::unordered_map;
using Map = vector<vector<int>>;

// Cell containing an XY position
struct Cell {
  int x, y;

  Cell() : x(0), y(0) {}
  Cell(int x, int y) : x(x), y(y) {}
  bool operator==(const Cell &other) const;
  bool operator!=(const Cell &other) const;
};

// Cell with a priority value
struct PriorityCell {
  int x, y, priority;

  PriorityCell() : x(0), y(0), priority(0) {}
  PriorityCell(int x, int y, int priority) : x(x), y(y), priority(priority) {}
};

// Compare PriorityCells based on their priority (for use in a pq)
struct ComparePriorityCell {
  bool operator()(const PriorityCell &a, const PriorityCell &b) const;
};

// Vector of cells with a priority value
struct PriorityVectorCell {
  vector<Cell> cells;
  int priority;

  PriorityVectorCell() : priority(0) {}
};

struct ComparePriorityVectorCell {
  bool operator()(const PriorityVectorCell &a,
                  const PriorityVectorCell &b) const;
};

// Print the map to stdout
void print_map(const Map &map, const vector<Cell> &path);

// Print a map with multiple robots (state) in it
void print_multi_map(const Map &map, const vector<Cell> &state);

// Get unoccupied cardinal neighbors of a cell in a map
vector<Cell> get_neighbors(const Map &map, const Cell &pos);

// Calculate the net neighbors function for a state
vector<vector<Cell>> get_net_neighbors(const Map &map,
                                       const vector<Cell> &state);

// heuristic
inline int h(const Cell &a, const Cell &b) {
  return abs(a.x - b.x) + abs(a.y - b.y);  // manhattan distance
}

// Reconstruct path from unordered_map
vector<Cell> reconstruct_path(const unordered_map<Cell, Cell> &came_from,
                              Cell current);

// Count the number of changes between two states of robots
int count_moved(const vector<Cell> &original, const vector<Cell> &changed);

// Update the occupancy map used in prioritized planning
vector<vector<Cell>> update_map_occ(const vector<vector<Cell>> &map_occ,
                                    const vector<Cell> &plan, int robot_index,
                                    const vector<Cell> &start,
                                    int time_offset = 0);

// Check if a single start and end is invalid
bool invalid_start(const Map &map, const Cell &goal, const Cell &start);

// Check if a multi robot start and end is invalid
bool invalid_multi_start(const Map &map, const vector<Cell> &start,
                         const vector<Cell> &goal);

// A* search algorithm
vector<Cell> astar(const Cell &start, const Cell &goal, const Map &map,
                   const vector<vector<Cell>> &map_occ = {});

// S* search algorithm
vector<vector<Cell>> sstar(const vector<Cell> &start, const vector<Cell> &goal,
                           const Map &map);

// Prioritized planning search algorithm
vector<vector<Cell>> pplan(const vector<Cell> &start, const vector<Cell> &goal,
                           const Map &map);

}  // namespace control_algorithms

// std::hash for Cell
namespace std {
template <>
struct hash<control_algorithms::Cell> {
  std::size_t operator()(const control_algorithms::Cell &c) const {
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
  }
};

// std::hash for vector<Cell>
template <>
struct hash<std::vector<control_algorithms::Cell>> {
  std::size_t operator()(const vector<control_algorithms::Cell> &vc) const {
    std::hash<int> hash_int;
    std::size_t seed = 0;
    for (const auto c : vc) {
      std::size_t cell_hash = hash_int(c.x) ^ (hash_int(c.y) << 1);
      seed ^= cell_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}  // namespace std
