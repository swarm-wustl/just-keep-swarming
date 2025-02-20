// Copyright 2024 Sebastian Theiler
#pragma once

#include <vector>

namespace control_algorithms {

using std::vector, std::size_t;
using Map = vector<vector<int>>;

// Cell containing an XY position
struct Cell {
  unsigned int x, y;

  Cell() : x(0), y(0) {}
  Cell(size_t x, size_t y) : x(x), y(y) {}
  bool operator==(const Cell &other) const;
  bool operator!=(const Cell &other) const;
};

// Cell with a priority value
struct PriorityCell {
  unsigned int x, y, priority;

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
  unsigned int priority;

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

// Check if a single start and end is invalid
bool invalid_start(const Map &map, const Cell &goal, const Cell &start);

// Check if a multi robot start and end is invalid
bool invalid_multi_start(const Map &map, const vector<Cell> &start,
                         const vector<Cell> &goal);

// Obsstacle the inflate according to radius parameter
bool obstacle_inflate(Map *map, int radius);

}  // namespace control_algorithms

namespace std {
// std::hash for Cell
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
