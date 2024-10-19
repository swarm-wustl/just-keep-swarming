// Copyright 2024 Sebastian Theiler
#include "control_algorithms/algorithms/assemble_graph.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "control_algorithms/algorithms/common.hpp"

namespace control_algorithms {

using std::size_t, std::vector, std::unordered_map, std::max;

AdjMatrix init_adj_matrix(unsigned int n_robots) {
  vector<vector<NodeConnectionType>> adj_matrix;
  for (unsigned int i = 0; i < n_robots; i++) {
    adj_matrix.push_back({});
    for (unsigned int j = 0; j < n_robots; j++) {
      adj_matrix[i].push_back(UNCONNECTED);
    }
  }

  return adj_matrix;
}

void connect_robots(AdjMatrix *adj_matrix, unsigned int a, unsigned int b,
                    NodeConnectionType connectionA,
                    NodeConnectionType connectionB) {
  (*adj_matrix)[a][b] = connectionA;
  (*adj_matrix)[b][a] = connectionB;
}

AdjMatrix load_structure(StructureType structure, size_t num_robots) {
  AdjMatrix mat = init_adj_matrix(num_robots);
  switch (structure) {
    case (MOBILE_MANIPULATOR): {
      if (num_robots < 8) {
        return {};
      }

      // Top
      connect_robots(&mat, 0, 1, LEFT, RIGHT);
      connect_robots(&mat, 1, 3, FRONT, FRONT);
      connect_robots(&mat, 1, 5, BACK, FRONT);

      // Bottom
      connect_robots(&mat, 0, 2, RIGHT, LEFT);
      connect_robots(&mat, 2, 4, FRONT, FRONT);
      connect_robots(&mat, 2, 6, BACK, FRONT);

      // Arm
      connect_robots(&mat, 0, 7, FRONT, FRONT);
      for (size_t i = 8; i < num_robots; i++) {
        connect_robots(&mat, i - 1, i, BACK, FRONT);
      }
      break;
    }
    case (WALKER): {
      if (num_robots < 11) {
        return {};
      }

      // Left half
      connect_robots(&mat, 0, 1, BACK, FRONT);
      connect_robots(&mat, 1, 3, LEFT, LEFT);
      connect_robots(&mat, 3, 4, FRONT, FRONT);
      connect_robots(&mat, 1, 5, RIGHT, RIGHT);
      connect_robots(&mat, 5, 6, FRONT, FRONT);

      // Right half
      connect_robots(&mat, 0, 2, FRONT, FRONT);
      connect_robots(&mat, 2, 7, LEFT, RIGHT);
      connect_robots(&mat, 7, 8, FRONT, FRONT);
      connect_robots(&mat, 2, 9, RIGHT, LEFT);
      connect_robots(&mat, 9, 10, FRONT, FRONT);

      // TODO(sebtheiler): Add longer arms in groups of 4
      break;
    }
    default:
      break;
  }

  return mat;
}

int calculate_node_height(const AdjMatrix &adj_matrix, int node,
                          unordered_map<int, int> *heights) {
  if (heights->find(node) != heights->end()) {
    return (*heights)[node];
  }

  int max_child_height = 0;
  for (size_t i = 0; i < adj_matrix.size(); i++) {
    if (adj_matrix[node][i] != UNCONNECTED && !(*heights)[node]) {
      // If there's a connection (child node), recursively compute height
      max_child_height =
          max(max_child_height, calculate_node_height(adj_matrix, i, heights));
    }
  }

  (*heights)[node] = 1 + max_child_height;
  return (*heights)[node];
}

unordered_map<int, int> calculate_matrix_heights(const AdjMatrix &adj_matrix,
                                                 int root) {
  unordered_map<int, int> heights;

  // Start from the root node, which by definition must calculate the heights of
  // all other nodes
  calculate_node_height(adj_matrix, root, &heights);

  return heights;
}

void calculate_node_children_locations(int node, const Cell &pos,
                                       const AdjMatrix &adj_matrix,
                                       const unordered_map<int, int> &heights,
                                       int radius,
                                       unordered_map<int, Cell> *node_to_pos) {
  int height = heights.at(node);
  vector<int> children;
  for (size_t i = 0; i < adj_matrix[node].size(); i++) {
    if (adj_matrix[node][i] != UNCONNECTED && heights.at(i) < height) {
      children.push_back(i);
    }
  }

  for (int child : children) {
    Cell new_pos;
    switch (adj_matrix[node][child]) {
      case LEFT: {
        new_pos = {pos.x - radius, pos.y};
        break;
      }
      case RIGHT: {
        new_pos = {pos.x + radius, pos.y};
        break;
      }
      case FRONT: {
        new_pos = {pos.x, pos.y + radius};
        break;
      }
      case BACK: {
        new_pos = {pos.x, pos.y - radius};
        break;
      }
      default:
        continue;
    }

    (*node_to_pos)[child] = new_pos;

    calculate_node_children_locations(child, new_pos, adj_matrix, heights,
                                      radius, node_to_pos);
  }
}

vector<Cell> unfold_graph(const AdjMatrix &adj_matrix, const Cell &root_cell,
                          int root_index, int radius) {
  unordered_map<int, int> heights =
      calculate_matrix_heights(adj_matrix, root_index);

  unordered_map<int, Cell> node_to_pos;
  vector<Cell> unfolded;
  node_to_pos[root_index] = root_cell;
  calculate_node_children_locations(root_index, root_cell, adj_matrix, heights,
                                    radius, &node_to_pos);

  for (size_t node = 0; node < node_to_pos.size(); node++) {
    unfolded.push_back(node_to_pos[node]);
  }

  return unfolded;
}

}  // namespace control_algorithms
