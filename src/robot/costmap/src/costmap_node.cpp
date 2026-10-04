#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  costmap_.initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      if (costmap_.convertToGrid(range, angle, x_grid, y_grid)) {
        costmap_.markObstacle(x_grid, y_grid);
      }
    }
  }

  // Step 3: Inflate obstacles
  costmap_.inflateObstacles();

  // Step 4: Publish costmap
  publishCostmap(scan->header.stamp);
}

void CostmapNode::publishCostmap(const rclcpp::Time &stamp) {
  auto message = nav_msgs::msg::OccupancyGrid();

  message.header.stamp = stamp;
  message.header.frame_id = "sim_world";

  message.info.resolution = costmap_.getResolution();
  message.info.width = costmap_.getWidth();
  message.info.height = costmap_.getHeight();
  message.info.origin.position.x = costmap_.getOriginX();
  message.info.origin.position.y = costmap_.getOriginY();
  message.info.origin.position.z = 0.0;

  const auto &grid = costmap_.getGrid();
  message.data.reserve(costmap_.getWidth() * costmap_.getHeight());
  for (int y = 0; y < costmap_.getHeight(); ++y) {
    for (int x = 0; x < costmap_.getWidth(); ++x) {
      message.data.push_back(grid[y][x]);
    }
  }

  costmap_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
