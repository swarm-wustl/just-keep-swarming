// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/pplan.hpp"

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <vector>

#include "control_algorithms/algorithms/astar.hpp"
#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {
using std::vector, std::queue, std::list, std::min;

vector<vector<Cell>> update_map_occ(const vector<vector<Cell>> &map_occ,
                                    const vector<Cell> &plan, int robot_index,
                                    const vector<Cell> &start,
                                    int time_offset) {
  vector<vector<Cell>> new_map_occ = map_occ;

  for (size_t i = 1; i < std::max(plan.size(), map_occ.size()); i++) {
    if (i >= new_map_occ.size()) {
      new_map_occ.push_back(new_map_occ[i - 1]);
    }
    if (i < plan.size()) {
      new_map_occ[i][robot_index] = plan[i];
    } else {
      new_map_occ[i][robot_index] = new_map_occ[i - 1][robot_index];
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

  vector<vector<Cell>> map_occ = {start};

  list<int> robot_order;
  vector<int> attempts(n, 0);
  for (size_t i = 0; i < n; i++) {
    robot_order.push_back(i);
  }

  const size_t MAX_RETRIES = 200;
  size_t retry_count = 0;
  while (!robot_order.empty() && retry_count < MAX_RETRIES) {
    ++retry_count;
    int robot_num = robot_order.front();
    robot_order.pop_front();
    const Cell &robot = start[robot_num];
    const Cell &end = goal[robot_num];

    vector<Cell> plan = astar(robot, end, map, map_occ);
    if (plan.empty()) {
      if (robot_order.size() == 0) {
        break;
      } else {
        size_t time_offset = 0;
        while (plan.empty() && time_offset + 1 < map_occ.size()) {
          // If a plan couldn't be found, try delaying start
          // This isn't perfect, but it increases the number of valid solutions
          ++time_offset;
          vector<vector<Cell>> truncated_map_occ(map_occ.begin() + time_offset,
                                                 map_occ.end());
          plan = astar(robot, end, map, truncated_map_occ);
        }

        if (!plan.empty()) {
          map_occ =
              update_map_occ(map_occ, plan, robot_num, start, time_offset);
          continue;
        }

        // Push the robot back
        ++attempts[robot_num];
        auto it = robot_order.begin();
        int ord =
            min(attempts[robot_num], static_cast<int>(robot_order.size()));
        std::advance(it, ord);
        robot_order.insert(it, robot_num);
      }
    } else {
      map_occ = update_map_occ(map_occ, plan, robot_num, start);
    }
  }

  if (retry_count == MAX_RETRIES) {
    std::cout << "FAILED TO FIND PATHS" << std::endl;
    return {};
  }

  return map_occ;
}

} // namespace control_algorithms
