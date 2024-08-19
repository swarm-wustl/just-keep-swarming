// Copyright 2024 Sebastian Theiler
#pragma once

#include <stdlib.h>

#include <functional>
#include <unordered_map>
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

// Print the map to stdout
void print_map(const Map &map, const vector<Cell> &path);

// Get unoccupied cardinal neighbors of a cell in a map
vector<Cell> get_neighbors(const Map &map, const Cell &pos);

// heuristic
inline int h(const Cell &a, const Cell &b) {
  return abs(a.x - b.x) + abs(a.y - b.y);  // manhattan distance
}

// Reconstruct path from unordered_map
vector<Cell> reconstruct_path(const unordered_map<Cell, Cell> &came_from,
                              Cell current);

// A* search algorithm
vector<Cell> astar(const Cell &start, const Cell &goal, const Map &map);

}  // namespace control_algorithms

// std::hash for Cell
namespace std {
template <>
struct hash<control_algorithms::Cell> {
  std::size_t operator()(const control_algorithms::Cell &c) const {
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
  }
};
}  // namespace std
