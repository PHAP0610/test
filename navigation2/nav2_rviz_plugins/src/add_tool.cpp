#include "nav2_rviz_plugins/add_tool.hpp"

#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/render_panel.hpp"

#include <QKeyEvent>
#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreVector3.h>

#include <QCursor>
#include <QMessageBox>

#include <cstdlib>
#include <fstream>
#include <filesystem>

using nav2_rviz_plugins::AddTool;

namespace nav2_rviz_plugins
{

AddTool::AddTool()
{
  // Pick a convenient shortcut if you like
  shortcut_key_ = 's';
}

AddTool::~AddTool()
{
  if (executor_) executor_->cancel();
  if (spinner_.joinable()) spinner_.join();
}

void AddTool::onInitialize()
{
  // Node
  raw_node_ = std::make_shared<rclcpp::Node>("add_tool_node_" + std::to_string(rand()));

  // Publisher with new topic name for visualization
  publisher_ = raw_node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "/add_tool/pose_array",
      rclcpp::QoS(10).transient_local().reliable());

  // Spin the node
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(raw_node_);
  spinner_ = std::thread([this]() { executor_->spin(); });

  // Resolve file path
  const char* env_path = std::getenv("ADD_TOOL_FILE");
  if (env_path && std::string(env_path).size() > 0) {
    file_path_ = env_path;
  } else {
    const char* home = std::getenv("HOME");
    std::string base = home ? home : "";
    if (base.empty()) base = "/tmp";
    file_path_ = base + "/.ros/add_tool_points.txt";
  }

  // Ensure directory exists
  try {
    std::filesystem::create_directories(std::filesystem::path(file_path_).parent_path());
  } catch (...) {
    RCLCPP_WARN(raw_node_->get_logger(), "Could not create parent dir for %s", file_path_.c_str());
  }

  RCLCPP_INFO(raw_node_->get_logger(), "AddTool ready. Writing to: %s", file_path_.c_str());
  RCLCPP_INFO(raw_node_->get_logger(), "Left click: add point + publish. Right click: append all to file.");
}

int AddTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftDown()) {
    // Project click onto ground plane (Z=0) — reuse your scale tweak if needed
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0);
    float scalex = 0.985f;
    float scaley = 0.98f;

    Ogre::Ray mouse_ray = context_->getViewManager()->getCurrent()->getCamera()
      ->getCameraToViewportRay(
        static_cast<float>(event.x) / static_cast<float>(event.panel->width() * scalex),
        static_cast<float>(event.y) / static_cast<float>(event.panel->height() * scaley));

    auto intersection = mouse_ray.intersects(ground_plane);
    if (!intersection.first) {
      return rviz_common::Tool::Finished;
    }

    Ogre::Vector3 pos = mouse_ray.getPoint(intersection.second);

    geometry_msgs::msg::Pose p;
    p.position.x = pos.x;
    p.position.y = pos.y;
    p.position.z = 0.0;
    p.orientation.w = 1.0;  // yaw = 0 by default; you can add yaw UI later

    poses_.push_back(p);
    publishPoseArray();

  } else if (event.rightDown()) {
    if (poses_.empty()) {
      RCLCPP_WARN(raw_node_->get_logger(), "No points to write.");
      return rviz_common::Tool::Finished;
    }
    if (appendPosesToFile()) {
      RCLCPP_INFO(raw_node_->get_logger(), "Appended %zu points to %s", poses_.size(), file_path_.c_str());
    } else {
      RCLCPP_ERROR(raw_node_->get_logger(), "Failed to append to %s", file_path_.c_str());
    }
  }

  return rviz_common::Tool::Render;
}

int AddTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel * /*panel*/)
{
  // Press 'D' to clear current buffer (keeps file intact)
  if (event->key() == Qt::Key_D) {
    poses_.clear();
    publishPoseArray(); // publish empty to clear display
    RCLCPP_WARN(raw_node_->get_logger(), "Cleared in-memory points.");
  }
  return rviz_common::Tool::Render;
}

void AddTool::publishPoseArray()
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = raw_node_->now();
  msg.poses = poses_;
  publisher_->publish(msg);
}

bool AddTool::appendPosesToFile()
{
  // Format: one pose per line: frame_id x y z qx qy qz qw
  // (Easy to parse later for your AutoLoopPanel)
  std::ofstream ofs(file_path_, std::ios::app);
  if (!ofs.is_open()) return false;

  const std::string frame = context_->getFixedFrame().toStdString();
  for (const auto& p : poses_) {
    ofs << frame << " "
        << p.position.x << " " << p.position.y << " " << p.position.z << " "
        << p.orientation.x << " " << p.orientation.y << " "
        << p.orientation.z << " " << p.orientation.w << "\n";
  }
  ofs.close();
  return true;
}

} // namespace nav2_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::AddTool, rviz_common::Tool)

