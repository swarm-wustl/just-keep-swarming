// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/astar.hpp"

#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <queue>
#include <unordered_map>
#include <vector>

#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

using std::vector, std::priority_queue, std::unordered_map, std::size_t,
    std::pair, std::queue;

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

vector<Cell> astar(const Cell &start, const Cell &goal, const Map &map,
                   const vector<vector<Cell>> &map_occ) {
  using PriorityQueue =
      priority_queue<PriorityCell, vector<PriorityCell>, ComparePriorityCell>;
  PriorityQueue frontier;
  unordered_map<Cell, Cell> came_from;
  unordered_map<Cell, int> cost_so_far;
  PriorityCell pstart = {start.x, start.y, 0};
  frontier.push(pstart);

  came_from[start] = start;
  cost_so_far[start] = 0;

  bool using_map_occ = map_occ.size() > 0;
  int time_step = 0;
  Map temp = map;

  // goal occupied, start occupied, start is goal
  if (invalid_start(map, goal, start)) {
    return {};
  }

  while (!frontier.empty()) {
    Cell current = {frontier.top().x, frontier.top().y};
    frontier.pop();

    if (current == goal) {
      return reconstruct_path(came_from, current);
    }

    // Update with occupied cells at this time step (and the next to stop
    // phasing)
    if (using_map_occ) {
      temp = map;

      // TODO(sebtheiler): make it so it doesn't have to reconstruct path every
      // time (or ever)
      time_step = reconstruct_path(came_from, current).size();
      vector<Cell> occ;
      if (time_step < static_cast<int>(map_occ.size()) - 1) {
        occ = map_occ[time_step];
        occ.insert(occ.end(), map_occ[time_step + 1].begin(),
                   map_occ[time_step + 1].end());
      } else {
        occ = map_occ.back();
      }
      for (const Cell &c : occ) {
        temp[c.x][c.y] = 1;
      }
    }

    for (const Cell &next : get_neighbors(temp, current)) {
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
