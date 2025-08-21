#ifndef NAV2_RVIZ_PLUGINS__WAYPOINT_FOLLOWER_HPP_
#define NAV2_RVIZ_PLUGINS__WAYPOINT_FOLLOWER_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz_common/panel.hpp"
#include "QPushButton"
#include "QVBoxLayout"

namespace nav2_rviz_plugins
{

class WaypointFollower : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointFollower(QWidget * parent = nullptr);

protected Q_SLOTS:
  void onButtonClicked();

protected:
  QPushButton * button_;
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__WAYPOINT_FOLLOWER_HPP_
