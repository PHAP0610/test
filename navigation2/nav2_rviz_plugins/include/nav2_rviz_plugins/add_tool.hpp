#pragma once

#include <memory>
#include <thread>
#include <vector>
#include <future>

#include "rviz_common/tool.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_rviz_plugins
{

class AddTool : public rviz_common::Tool
{
public:
  AddTool();
  ~AddTool() override;

  void onInitialize() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel) override;

  void activate() override {}
  void deactivate() override {}

private:
  void publishPoseArray();
  bool appendPosesToFile();

  std::shared_ptr<rclcpp::Node> raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;

  std::vector<geometry_msgs::msg::Pose> poses_;
  std::string file_path_;
};

}  // namespace nav2_rviz_plugins

