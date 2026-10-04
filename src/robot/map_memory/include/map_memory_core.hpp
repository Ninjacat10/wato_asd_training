#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Fill the global map with a fresh, all-free grid
    void initializeMap();

    // Transform the given costmap (expressed in the robot's local frame) into
    // the global map using the robot's current pose, overwriting any cells it covers.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap,
                           double robot_x, double robot_y, double robot_yaw);

    nav_msgs::msg::OccupancyGrid toMsg(const rclcpp::Time &stamp) const;

  private:
    rclcpp::Logger logger_;

    static constexpr int width_ = 300;        // cells
    static constexpr int height_ = 300;       // cells
    static constexpr double resolution_ = 0.1; // meters/cell
    static constexpr double origin_x_ = -(width_ * resolution_) / 2.0;
    static constexpr double origin_y_ = -(height_ * resolution_) / 2.0;

    std::vector<std::vector<int8_t>> grid_;
};

}

#endif
