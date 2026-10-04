#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Initialize parameters
  lookahead_distance_ = 1.0;  // Lookahead distance
  goal_tolerance_ = 0.1;      // Distance to consider the goal reached
  linear_speed_ = 0.5;        // Constant forward speed

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    return;
  }

  // Stop if we're within tolerance of the final goal
  const auto &goal_point = current_path_->poses.back().pose.position;
  if (computeDistance(robot_odom_->pose.pose.position, goal_point) < goal_tolerance_) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return; // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto &robot_pos = robot_odom_->pose.pose.position;

  for (const auto &pose_stamped : current_path_->poses) {
    double distance = computeDistance(robot_pos, pose_stamped.pose.position);
    if (distance >= lookahead_distance_) {
      return pose_stamped;
    }
  }

  // No point far enough away was found (robot is near the end of the path) -
  // fall back to the final waypoint so the robot keeps heading toward the goal.
  if (!current_path_->poses.empty()) {
    return current_path_->poses.back();
  }

  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  double yaw = extractYaw(robot_odom_->pose.pose.orientation);
  const auto &robot_pos = robot_odom_->pose.pose.position;

  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y;

  // Transform the target into the robot's local frame
  double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

  double distance_to_target = std::sqrt(local_x * local_x + local_y * local_y);
  if (distance_to_target < 1e-6) {
    return cmd_vel; // avoid division by zero; target is essentially at the robot
  }

  // Pure pursuit curvature: kappa = 2y / L^2
  double curvature = 2.0 * local_y / (distance_to_target * distance_to_target);

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = curvature * linear_speed_;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) const {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                     1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
