// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/astar.hpp"

#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace control_algorithms {

using std::vector, std::priority_queue, std::unordered_map, std::size_t,
    std::pair, std::unordered_set, std::queue;

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
      if (find(path.begin(), path.end(),
               Cell{static_cast<int>(i), static_cast<int>(j)}) != path.end()) {
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

void print_multi_map(const Map &map, const vector<Cell> &state) {
  for (size_t i = 0; i < map.size(); i++) {
    for (size_t j = 0; j < map[i].size(); j++) {
      auto it = find(state.cbegin(), state.cend(),
                     Cell{static_cast<int>(i), static_cast<int>(j)});
      if (it != state.cend()) {
        std::cout << std::distance(state.cbegin(), it) % 10;  // robot number
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

bool invalid_start(const Map &map, const Cell &goal, const Cell &start) {
  int rows = static_cast<int>(map.size());
  if (rows == 0) return true;
  int cols = static_cast<int>(map[0].size());
  if (cols == 0                                        // map size 0
      || goal.x >= rows || goal.y >= cols              // goal out of bounds
      || start.x >= rows || start.y >= cols            // start out of bounds
      || map[goal.x][goal.y] || map[start.x][start.y]  // goal or start occupied
      || start == goal                                 // start is goal
  ) {
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
        break;
      }
    }
  }

  return invalid;
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

int count_moved(const vector<Cell> &original, const vector<Cell> &changed) {
  int num_moved = 0;
  for (int i = 0; i < static_cast<int>(original.size()); i++) {
    if (original[i] != changed[i]) num_moved++;
  }

  return num_moved;
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

vector<vector<Cell>> update_map_occ(const vector<vector<Cell>> &map_occ,
                                    const vector<Cell> &plan, int robot_index,
                                    const vector<Cell> &start,
                                    int time_offset) {
  vector<vector<Cell>> new_map_occ = map_occ;
  size_t plan_size = plan.size();
  size_t map_occ_size = map_occ.size();
  for (size_t time_step = time_offset;
       time_step < std::max(plan_size, map_occ_size); time_step++) {
    if (time_step >= map_occ_size) {
      // assume robots stay at their previous position
      if (time_step > 0) {
        new_map_occ.push_back(new_map_occ[time_step - 1]);
      } else {
        new_map_occ.push_back(start);
      }
    }
    if (time_step < plan_size) {
      new_map_occ[time_step][robot_index] = plan[time_step];
    } else if (time_step > 0) {
      new_map_occ[time_step][robot_index] =
          new_map_occ[time_step - 1][robot_index];
    } else {
      new_map_occ[0][robot_index] = start[robot_index];
    }
  }

  return new_map_occ;
}

vector<vector<Cell>> pplan(const vector<Cell> &start, const vector<Cell> &goal,
                           const Map &map) {
  size_t n = start.size();

  if (invalid_multi_start(map, start, goal)) {
    std::cout << "INVALID STARTING CONDITIONS" << std::endl;
    return {};
  }

  vector<vector<Cell>> map_occ;
  queue<size_t> backlog;  // robots that couldn't find a path

  // Initial planning
  for (size_t i = 0; i < n; i++) {
    Cell robot = start[i];
    Cell end = goal[i];

    vector<Cell> plan = astar(robot, end, map, map_occ);
    if (plan.empty()) {
      backlog.push(i);
    } else {
      map_occ = update_map_occ(map_occ, plan, i, start);
    }
  }

  // Retry for failed robots
  const size_t MAX_RETRIES = 100;  // Prevent infinite loops
  size_t retry_count = 0;
  while (!backlog.empty() && retry_count < MAX_RETRIES) {
    size_t i = backlog.front();
    backlog.pop();

    const Cell &robot = start[i];
    const Cell &end = goal[i];
    size_t time_offset = 0;
    vector<Cell> plan = astar(robot, end, map, map_occ);

    while (plan.empty() && time_offset < map_occ.size() - 1) {
      // If a plan couldn't be found, try delaying start
      // This isn't perfect, but it increases the number of valid solutions
      ++time_offset;
      vector<vector<Cell>> truncated_map_occ(map_occ.begin() + time_offset,
                                             map_occ.end());
      plan = astar(robot, end, map, truncated_map_occ);
    }

    // TODO(sebtheiler): make this smarter so it doesn't "cycle" through
    // the same set of robots without any changes
    if (plan.empty() && backlog.size() > 1) {
      backlog.push(i);
    }

    map_occ = update_map_occ(map_occ, plan, i, start, time_offset);
  }

  if (!backlog.empty()) {
    std::cout << "FAILED TO FIND PATHS" << std::endl;
    return {};
  }

  return map_occ;
}

}  // namespace control_algorithms
