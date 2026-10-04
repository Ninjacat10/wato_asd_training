#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
  initializeCostmap();
}

void CostmapCore::initializeCostmap() {
  grid_.assign(height_, std::vector<int8_t>(width_, 0));
}

bool CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) const {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>((x - origin_x_) / resolution_);
  y_grid = static_cast<int>((y - origin_y_) / resolution_);

  return x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    grid_[y_grid][x_grid] = OCCUPIED_COST;
  }
}

void CostmapCore::inflateObstacles() {
  // Work from a snapshot so inflation doesn't feed on cells we already inflated.
  std::vector<std::vector<int8_t>> inflated = grid_;

  int radius_cells = static_cast<int>(std::ceil(INFLATION_RADIUS / resolution_));

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y][x] != OCCUPIED_COST) {
        continue;
      }

      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          int nx = x + dx;
          int ny = y + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
            continue;
          }

          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
          if (distance > INFLATION_RADIUS) {
            continue;
          }

          int cost = static_cast<int>(OCCUPIED_COST * (1.0 - distance / INFLATION_RADIUS));
          if (cost > inflated[ny][nx]) {
            inflated[ny][nx] = static_cast<int8_t>(cost);
          }
        }
      }
    }
  }

  grid_ = inflated;
}

}
