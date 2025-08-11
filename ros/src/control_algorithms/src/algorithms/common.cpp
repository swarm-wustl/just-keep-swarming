// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/common.hpp"

#include <algorithm>
#include <iostream>

namespace control_algorithms {

using std::size_t, std::find;

bool Cell::operator==(const Cell &other) const {
  return (x == other.x && y == other.y);
}

bool Cell::operator!=(const Cell &other) const { return !(*this == other); }

bool ComparePriorityCell::operator()(const PriorityCell &a,
                                     const PriorityCell &b) const {
  return a.priority > b.priority;
}

bool ComparePriorityVectorCell::operator()(const PriorityVectorCell &a,
                                           const PriorityVectorCell &b) const {
  return a.priority > b.priority;
}

void print_map(const Map &map, const vector<Cell> &path) {
  for (size_t i = 0; i < map.size(); i++) {
    for (size_t j = 0; j < map[i].size(); j++) {
      if (find(path.begin(), path.end(), Cell{i, j}) != path.end()) {
        std::cout << "*";
      } else if (map[i][j]) {
        std::cout << "#";
      } else {
        std::cout << ".";
      }
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
}

bool obstacle_inflate(Map *map, int radius) {
  Map new_map = *map;
  int rows = new_map.size();
  int cols = new_map[0].size();
  bool changed = false;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if ((*map)[i][j] == 1) {
        for (int di = -radius; di <= radius; ++di) {
          for (int dj = -radius; dj <= radius; ++dj) {
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
              if (new_map[ni][nj] != 1) {
                new_map[ni][nj] = 1;
                changed = true;
              }
            }
          }
        }
      }
    }
  }

  *map = new_map;

  return changed;
}

void print_multi_map(const Map &map, const vector<Cell> &state) {
  for (size_t i = 0; i < map.size(); i++) {
    for (size_t j = 0; j < map[i].size(); j++) {
      auto it = find(state.cbegin(), state.cend(), Cell{i, j});
      if (it != state.cend()) {
        std::cout << std::distance(state.cbegin(), it) % 10; // robot number
      } else if (map[i][j]) {
        std::cout << "#";
      } else {
        std::cout << ".";
      }
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
}

bool invalid_start(const Map &map, const Cell &goal, const Cell &start) {
  size_t rows = map.size();
  if (rows == 0)
    return true;
  size_t cols = map[0].size();
  std::cout << rows << std::endl;
  std::cout << cols << std::endl;
  if (cols == 0                                       // map size 0
      || goal.x >= rows || goal.y >= cols             // goal out of bounds
      || start.x >= rows || start.y >= cols           // start out of bounds
      || map[goal.x][goal.y] || map[start.x][start.y] // goal or start occupied
      || start == goal                                // start is goal
  ) {
    /*std::cout << "a" << (cols == 0); // map size 0*/
    /*std::cout << "b" << (goal.x >= rows);*/
    /*std::cout << "bb" << (goal.y >= cols); // goal out of bounds*/
    /*std::cout << "c" << (start.x >= rows);*/
    /*std::cout << "cc" << (start.y >= cols); // start out of bounds*/
    /*std::cout << "d" << (map[goal.x][goal.y]);*/
    /*std::cout << "dd" << (map[start.x][start.y]); // goal or start occupied*/
    /*std::cout << "e" << (start == goal);          // start is goal*/
    return true;
  }

  return false;
}

bool invalid_multi_start(const Map &map, const vector<Cell> &start,
                         const vector<Cell> &goal) {
  bool invalid = false;
  if (start.size() != goal.size()) {
    invalid = true;
  } else {
    for (size_t i = 0; i < start.size(); i++) {
      if (invalid_start(map, goal[i], start[i])) {
        invalid = true;
        std::cout << "Invalid at " << goal[i].x << ", " << goal[i].y << " or "
                  << start[i].x << ", " << start[i].y << std::endl;
        break;
      }
    }
  }

  return invalid;
}

} // namespace control_algorithms
