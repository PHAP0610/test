#ifndef NAV2_RVIZ_PLUGINS__MULTI_GOAL_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__MULTI_GOAL_TOOL_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace nav2_rviz_plugins
{

class MultiGoalTool : public rviz_common::Tool
{
public:
  MultiGoalTool();
  ~MultiGoalTool() override = default;

  void onInitialize() override;
  void activate() override {}
  void deactivate() override {}

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  rclcpp::Node::SharedPtr raw_node_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;

  void sendGoals();
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__MULTI_GOAL_TOOL_HPP_
