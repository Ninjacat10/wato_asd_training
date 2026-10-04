#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "planner_core.hpp"

namespace robot
{

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score; // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

bool PlannerCore::search(const nav_msgs::msg::OccupancyGrid &map,
                          int start_x, int start_y,
                          int goal_x, int goal_y,
                          std::vector<CellIndex> &path_cells) const {
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  auto isBlocked = [&](int x, int y) {
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return true;
    }
    int8_t value = map.data[y * width + x];
    return value >= OCCUPIED_THRESHOLD;
  };

  CellIndex start(start_x, start_y);
  CellIndex goal(goal_x, goal_y);

  if (isBlocked(start.x, start.y) || isBlocked(goal.x, goal.y)) {
    RCLCPP_WARN(logger_, "A*: start or goal cell is blocked/out of bounds");
    return false;
  }

  auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      // Reconstruct path
      path_cells.clear();
      CellIndex node = current;
      while (!(node == start)) {
        path_cells.push_back(node);
        node = came_from[node];
      }
      path_cells.push_back(start);
      std::reverse(path_cells.begin(), path_cells.end());
      return true;
    }

    if (closed_set.count(current)) {
      continue;
    }
    closed_set.insert(current);

    for (int i = 0; i < 8; ++i) {
      CellIndex neighbor(current.x + dxs[i], current.y + dys[i]);

      if (isBlocked(neighbor.x, neighbor.y) || closed_set.count(neighbor)) {
        continue;
      }

      double step_cost = (dxs[i] != 0 && dys[i] != 0) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current] + step_cost;

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.emplace(neighbor, f);
      }
    }
  }

  RCLCPP_WARN(logger_, "A*: no path found");
  return false;
}

}
