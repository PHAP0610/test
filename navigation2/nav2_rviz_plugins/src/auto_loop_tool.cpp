// auto_loop_tool.cpp
#include "rviz_common/viewport_mouse_event.hpp"

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreVector3.h>

#include "nav2_rviz_plugins/auto_loop_tool.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/render_panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <QCursor>
#include <QMessageBox>
#include "rclcpp_action/rclcpp_action.hpp"

rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;

namespace nav2_rviz_plugins
{

AutoLoopTool::AutoLoopTool()
{
  shortcut_key_ = 'a';
}

AutoLoopTool::~AutoLoopTool()
{
  loop_active_ = false;
  if (loop_thread_.joinable()) loop_thread_.join();
  if (executor_ && spinner_.joinable()) {
    executor_->cancel();
    spinner_.join();
  }
}

void AutoLoopTool::onInitialize()
{
  raw_node_ = std::make_shared<rclcpp::Node>("auto_loop_tool_node_" + std::to_string(rand()));
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    raw_node_, "navigate_through_poses");
  publisher_ = raw_node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "/multi_goal_tool/pose_array",
    rclcpp::QoS(10).transient_local().reliable());
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(raw_node_);
  spinner_ = std::thread([this]() { executor_->spin(); });
}

int AutoLoopTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
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

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x;
    pose.position.y = pos.y;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;

    poses_.push_back(pose);

    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = context_->getFixedFrame().toStdString();
    pose_array_msg.header.stamp = raw_node_->now();
    pose_array_msg.poses = poses_;

    publisher_->publish(pose_array_msg);
  }
  else if (event.rightDown())
  {
    if (poses_.empty()) return rviz_common::Tool::Finished;
    sendGoals();
  }

  return rviz_common::Tool::Render;
}

int AutoLoopTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel)
{
  (void)panel;
  if (event->key() == Qt::Key_D) {
    RCLCPP_WARN(raw_node_->get_logger(), "Loop stopped by user.");
    loop_active_ = false;
    if (loop_thread_.joinable()) loop_thread_.join();

    poses_.clear();

    geometry_msgs::msg::PoseArray empty_msg;
    empty_msg.header.frame_id = context_->getFixedFrame().toStdString();
    empty_msg.header.stamp = raw_node_->now();
    publisher_->publish(empty_msg);
  }
  return rviz_common::Tool::Render;
}

void AutoLoopTool::sendGoals()
{
  loop_active_ = true;
  loop_thread_ = std::thread([this]() {
    while (loop_active_)
    {
      if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(raw_node_->get_logger(), "NavigateThroughPoses action server not available!");
        return;
      }

      auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
      goal_msg.behavior_tree = "";

      for (const auto & pose : poses_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = context_->getFixedFrame().toStdString();
        ps.header.stamp = raw_node_->now();
        ps.pose = pose;
        goal_msg.poses.push_back(ps);
      }

      std::promise<void> finished;
      auto future = finished.get_future();

      auto send_goal_options = rclcpp_action::Client<
        nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
      send_goal_options.result_callback = [&, this](auto result) {
        RCLCPP_INFO(rclcpp::get_logger("AutoLoopTool"),
          "Navigation finished. Status code = %d", static_cast<int>(result.code));
        finished.set_value();
      };

      action_client_->async_send_goal(goal_msg, send_goal_options);
      RCLCPP_INFO(raw_node_->get_logger(), "Sending %zu goals to NavigateThroughPoses...", poses_.size());

      if (future.valid()) {
        future.wait();
      }

//      std::rotate(poses_.begin(), poses_.end() - 1, poses_.end());
      std::reverse(poses_.begin(), poses_.end());

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
}

void AutoLoopTool::activate() {}
void AutoLoopTool::deactivate() {}

}  // namespace nav2_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::AutoLoopTool, rviz_common::Tool)

