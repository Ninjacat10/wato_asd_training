#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {
  initializeMap();
}

void MapMemoryCore::initializeMap() {
  grid_.assign(height_, std::vector<int8_t>(width_, 0));
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap,
                                      double robot_x, double robot_y, double robot_yaw) {
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  for (unsigned int cy = 0; cy < costmap.info.height; ++cy) {
    for (unsigned int cx = 0; cx < costmap.info.width; ++cx) {
      int8_t value = costmap.data[cy * costmap.info.width + cx];

      // Local (robot-frame) coordinates of the center of this costmap cell
      double local_x = costmap.info.origin.position.x + (cx + 0.5) * costmap.info.resolution;
      double local_y = costmap.info.origin.position.y + (cy + 0.5) * costmap.info.resolution;

      // Rotate + translate into the global map frame using the robot's pose
      double global_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      double global_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      int mx = static_cast<int>((global_x - origin_x_) / resolution_);
      int my = static_cast<int>((global_y - origin_y_) / resolution_);

      if (mx < 0 || mx >= width_ || my < 0 || my >= height_) {
        continue;
      }

      // New data from the costmap is always "known" in our implementation,
      // so it always overwrites the global map's previous value for that cell.
      grid_[my][mx] = value;
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::toMsg(const rclcpp::Time &stamp) const {
  nav_msgs::msg::OccupancyGrid message;

  message.header.stamp = stamp;
  message.header.frame_id = "sim_world";

  message.info.resolution = resolution_;
  message.info.width = width_;
  message.info.height = height_;
  message.info.origin.position.x = origin_x_;
  message.info.origin.position.y = origin_y_;
  message.info.origin.position.z = 0.0;

  message.data.reserve(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      message.data.push_back(grid_[y][x]);
    }
  }

  return message;
}

}
