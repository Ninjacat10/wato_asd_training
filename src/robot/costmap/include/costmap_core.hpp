#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Reset the grid to all-free (0) before processing a new scan
    void initializeCostmap();

    // Convert a polar lidar reading (range, angle) into grid cell indices.
    // Returns false if the resulting cell falls outside the grid.
    bool convertToGrid(double range, double angle, int &x_grid, int &y_grid) const;

    // Mark a cell as occupied (high cost)
    void markObstacle(int x_grid, int y_grid);

    // Spread cost outward from every occupied cell so the robot keeps a safety margin
    void inflateObstacles();

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    const std::vector<std::vector<int8_t>>& getGrid() const { return grid_; }

  private:
    rclcpp::Logger logger_;

    static constexpr int width_ = 100;       // cells
    static constexpr int height_ = 100;      // cells
    static constexpr double resolution_ = 0.1; // meters/cell
    // Origin is the bottom-left corner of the grid, in world coordinates,
    // so that the robot sits at the center of the map.
    static constexpr double origin_x_ = -(width_ * resolution_) / 2.0;
    static constexpr double origin_y_ = -(height_ * resolution_) / 2.0;

    static constexpr int8_t OCCUPIED_COST = 100;
    static constexpr double INFLATION_RADIUS = 1.0; // meters

    std::vector<std::vector<int8_t>> grid_;
};

}

#endif
