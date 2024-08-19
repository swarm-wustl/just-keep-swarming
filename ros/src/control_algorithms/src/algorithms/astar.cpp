// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/astar.hpp"

#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

namespace control_algorithms {

using std::vector, std::priority_queue, std::unordered_map, std::size_t;

bool Cell::operator==(const Cell &other) const {
  return (x == other.x && y == other.y);
}

bool Cell::operator!=(const Cell &other) const { return !(*this == other); }

bool ComparePriorityCell::operator()(const PriorityCell &a,
                                     const PriorityCell &b) const {
  return a.priority > b.priority;
}

void print_map(const Map &map, const vector<Cell> &path) {
  for (size_t i = 0; i < map.size(); i++) {
    for (size_t j = 0; j < map[i].size(); j++) {
      if (std::find(path.begin(), path.end(),
                    Cell{static_cast<int>(i), static_cast<int>(j)}) !=
          path.end()) {
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

vector<Cell> get_neighbors(const Map &map, const Cell &pos) {
  vector<Cell> neighbors;

  // ♫ North and South and East and West ♫
  if (pos.y > 0 && !map[pos.x][pos.y - 1]) {
    neighbors.emplace_back(pos.x, pos.y - 1);
  }
  if (static_cast<size_t>(pos.y) < map[pos.x].size() - 1 &&
      !map[pos.x][pos.y + 1]) {
    neighbors.emplace_back(pos.x, pos.y + 1);
  }
  if (pos.x > 0 && !map[pos.x - 1][pos.y]) {
    neighbors.emplace_back(pos.x - 1, pos.y);
  }
  if (static_cast<size_t>(pos.x) < map.size() - 1 && !map[pos.x + 1][pos.y]) {
    neighbors.emplace_back(pos.x + 1, pos.y);
  }

  return neighbors;
}

vector<Cell> reconstruct_path(const unordered_map<Cell, Cell> &came_from,
                              Cell current) {
  vector<Cell> path;
  while (came_from.at(current) != current) {
    path.push_back(current);
    current = came_from.at(current);
  }
  path.push_back(current);
  std::reverse(path.begin(), path.end());
  return path;
}

vector<Cell> astar(const Cell &start, const Cell &goal, const Map &map) {
  using PriorityQueue =
      priority_queue<PriorityCell, vector<PriorityCell>, ComparePriorityCell>;
  PriorityQueue frontier;
  unordered_map<Cell, Cell> came_from;
  unordered_map<Cell, int> cost_so_far;
  PriorityCell pstart = {start.x, start.y, 0};
  frontier.push(pstart);

  came_from[start] = start;
  cost_so_far[start] = 0;

  // goal occupied, start occupied, start is goal
  if (map[goal.x][goal.y] || map[start.x][start.y] || start == goal) {
    return {};
  }

  while (!frontier.empty()) {
    Cell current = {frontier.top().x, frontier.top().y};
    frontier.pop();

    if (current == goal) {
      return reconstruct_path(came_from, current);
    }

    for (const Cell &next : get_neighbors(map, current)) {
      int new_cost =
          cost_so_far[current] +
          1;  // 1 cost for getting to the next cell, regardless of direction
      if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        int priority = new_cost + h(next, goal);
        PriorityCell pnext = {next.x, next.y, priority};
        frontier.push(pnext);
        came_from[next] = current;
      }
    }
  }

  // no path found
  return {};
}

}  // namespace control_algorithms
