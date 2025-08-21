#pragma once

#include <memory>
#include <thread>
#include <vector>
#include <future>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "rviz_common/panel.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace nav2_rviz_plugins
{

class AutoLoopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  AutoLoopPanel(QWidget * parent = nullptr);
  ~AutoLoopPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onAuto();
  void onStop();
  void onLoad();
  void onClear();

private:
  // helpers
  std::string resolveFilePath() const;
  bool loadFromFile(std::vector<geometry_msgs::msg::PoseStamped>& out);
  void publishPoseArray(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
  void clearFileAndPublishEmpty();

  void startLoop(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
  void stopLoop();

  // UI
  QPushButton* btn_auto_;
  QPushButton* btn_stop_;
  QPushButton* btn_load_;
  QPushButton* btn_clear_;
  QLabel*      lbl_status_;

  // ROS
  std::shared_ptr<rclcpp::Node> raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;

  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;

  // loop
  std::thread loop_thread_;
  std::atomic_bool loop_active_{false};
  
  // keep last/default frame id for publishing (tránh đụng context_ private)
  std::string default_frame_id_ = "map";
};

} // namespace nav2_rviz_plugins

