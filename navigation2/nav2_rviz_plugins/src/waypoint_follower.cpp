#include "nav2_rviz_plugins/waypoint_follower.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <chrono>

namespace nav2_rviz_plugins
{

WaypointFollower::WaypointFollower(QWidget * parent)
: rviz_common::Panel(parent)
{
  raw_node_ = std::make_shared<rclcpp::Node>("waypoint_follower_panel");

  pub_ = raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(10));

  button_ = new QPushButton("Send Goal", this);
  QVBoxLayout * layout = new QVBoxLayout;
  layout->addWidget(button_);
  setLayout(layout);

  connect(button_, SIGNAL(clicked()), this, SLOT(onButtonClicked()));
}

void WaypointFollower::onButtonClicked()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = raw_node_->now();
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 1.0;
  pose.pose.orientation.w = 1.0;

  pub_->publish(pose);
}

}  // namespace nav2_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::WaypointFollower, rviz_common::Panel)
