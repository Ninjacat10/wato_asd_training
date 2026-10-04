#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Runs A* from (start_x, start_y) to (goal_x, goal_y) grid cells on the given map.
    // Returns true and fills path_cells (ordered start -> goal) if a path was found.
    bool search(const nav_msgs::msg::OccupancyGrid &map,
                int start_x, int start_y,
                int goal_x, int goal_y,
                std::vector<CellIndex> &path_cells) const;

  private:
    rclcpp::Logger logger_;

    static constexpr int8_t OCCUPIED_THRESHOLD = 50;
};

}

#endif
