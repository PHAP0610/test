#include <OgreCamera.h>
#include <OgreRay.h>
#include "nav2_rviz_plugins/multi_goal_tool.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/render_panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <QCursor>
#include <QMessageBox>

namespace nav2_rviz_plugins
{

MultiGoalTool::MultiGoalTool()
{
  shortcut_key_ = 'm';
}

void MultiGoalTool::onInitialize()
{
  raw_node_ = std::make_shared<rclcpp::Node>("multi_goal_tool_rviz");
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    raw_node_, "navigate_through_poses");
}

int MultiGoalTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftDown())
  {
    Ogre::Vector3 pos;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0);

    float scalex = 0.985;
    float scaley = 0.98;
    Ogre::Ray mouse_ray = context_->getViewManager()->getCurrent()->getCamera()->getCameraToViewportRay(
        static_cast<float>(event.x) / static_cast<float>(event.panel->width()*scalex),
        static_cast<float>(event.y) / static_cast<float>(event.panel->height()*scaley));

    std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
    if (!intersection.first)
      return rviz_common::Tool::Finished;

    pos = mouse_ray.getPoint(intersection.second);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = context_->getFixedFrame().toStdString();
    pose.header.stamp = raw_node_->now();

    pose.pose.position.x = pos.x;
    pose.pose.position.y = pos.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    goals_.push_back(pose);
    RCLCPP_INFO(raw_node_->get_logger(), "✅ Added goal at (%.2f, %.2f)", pos.x, pos.y);

    // Gửi nếu đã có ít nhất 3 điểm
//    if (goals_.size() >= 3)
//    {
//      sendGoals();
//      goals_.clear();
//    }
  }
  else if (event.rightDown())
  {
    if (goals_.empty()) return rviz_common::Tool::Finished;
    sendGoals();
    goals_.clear();
  }

  return rviz_common::Tool::Render;
}


void MultiGoalTool::sendGoals()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(raw_node_->get_logger(), "NavigateThroughPoses action server not available!");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
  goal_msg.poses = goals_;
  goal_msg.behavior_tree = "";

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [](auto result)
  {
    RCLCPP_INFO(rclcpp::get_logger("MultiGoalTool"),
      "Navigation finished. Status code = %d", static_cast<int>(result.code));

  };



  action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(raw_node_->get_logger(), "Sending %zu goals to NavigateThroughPoses...", goals_.size());
}

}  // namespace nav2_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::MultiGoalTool, rviz_common::Tool)
