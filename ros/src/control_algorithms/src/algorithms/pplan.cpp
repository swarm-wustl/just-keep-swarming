// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/pplan.hpp"

#include <iostream>
#include <queue>
#include <vector>

#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {
using std::vector, std::queue;

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

    while (plan.empty() && time_offset + 1 < map_occ.size()) {
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
    ++retry_count;
  }

  if (!backlog.empty()) {
    std::cout << "FAILED TO FIND PATHS" << std::endl;
    return {};
  }

  return map_occ;
}

}  // namespace control_algorithms
