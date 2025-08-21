// auto_loop_tool.hpp
#ifndef NAV2_RVIZ_PLUGINS__AUTO_LOOP_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__AUTO_LOOP_TOOL_HPP_

#include <memory>
#include <vector>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/render_panel.hpp"
#include <QKeyEvent>

namespace nav2_rviz_plugins
{

class AutoLoopTool : public rviz_common::Tool
{
public:
  AutoLoopTool();
  ~AutoLoopTool() override;

  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel) override;
  void activate() override;
  void deactivate() override;

private:
  void sendGoals();

  std::shared_ptr<rclcpp::Node> raw_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
  std::thread loop_thread_;
  bool loop_active_ = false;

  std::vector<geometry_msgs::msg::Pose> poses_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__AUTO_LOOP_TOOL_HPP_

