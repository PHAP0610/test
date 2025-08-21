#include "nav2_rviz_plugins/auto_loop_panel.hpp"

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <filesystem>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace nav2_rviz_plugins
{

AutoLoopPanel::AutoLoopPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  auto layout = new QVBoxLayout;

  auto row = new QHBoxLayout;
  btn_auto_  = new QPushButton("Auto");
  btn_stop_  = new QPushButton("Stop");
  btn_load_  = new QPushButton("Load");
  btn_clear_ = new QPushButton("Clear");
  row->addWidget(btn_auto_);
  row->addWidget(btn_stop_);
  row->addWidget(btn_load_);
  row->addWidget(btn_clear_);
  layout->addLayout(row);

  lbl_status_ = new QLabel("Idle");
  layout->addWidget(lbl_status_);

  setLayout(layout);

  connect(btn_auto_,  &QPushButton::clicked, this, &AutoLoopPanel::onAuto);
  connect(btn_stop_,  &QPushButton::clicked, this, &AutoLoopPanel::onStop);
  connect(btn_load_,  &QPushButton::clicked, this, &AutoLoopPanel::onLoad);
  connect(btn_clear_, &QPushButton::clicked, this, &AutoLoopPanel::onClear);
}

AutoLoopPanel::~AutoLoopPanel()
{
  stopLoop();
  if (executor_) executor_->cancel();
  if (spinner_.joinable()) spinner_.join();
}

void AutoLoopPanel::onInitialize()
{
  raw_node_ = std::make_shared<rclcpp::Node>("auto_loop_panel_" + std::to_string(std::rand()));

  // Publisher to show points again in RViz
  pose_pub_ = raw_node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "/add_tool/pose_array",
      rclcpp::QoS(10).transient_local().reliable());

  // Action client
  action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      raw_node_, "navigate_through_poses");

  // Spin
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(raw_node_);
  spinner_ = std::thread([this]() { executor_->spin(); });

  lbl_status_->setText(QString("Ready. File: %1").arg(QString::fromStdString(resolveFilePath())));
}

std::string AutoLoopPanel::resolveFilePath() const
{
  const char* env_path = std::getenv("ADD_TOOL_FILE");
  if (env_path && std::string(env_path).size() > 0) return env_path;

  const char* home = std::getenv("HOME");
  std::string base = home ? home : "";
  if (base.empty()) base = "/tmp";
  return base + "/.ros/add_tool_points.txt";
}

bool AutoLoopPanel::loadFromFile(std::vector<geometry_msgs::msg::PoseStamped>& out)
{
  out.clear();
  std::ifstream ifs(resolveFilePath());
  if (!ifs.is_open()) {
    RCLCPP_ERROR(raw_node_->get_logger(), "Cannot open file: %s", resolveFilePath().c_str());
    return false;
  }

  std::string frame;
  double x,y,z,qx,qy,qz,qw;
  std::string line;
  bool first_valid_line = true;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    if (!(iss >> frame >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      RCLCPP_WARN(raw_node_->get_logger(), "Skip malformed line: %s", line.c_str());
      continue;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = raw_node_->now();
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
    ps.pose.orientation.x = qx;
    ps.pose.orientation.y = qy;
    ps.pose.orientation.z = qz;
    ps.pose.orientation.w = qw;
    out.push_back(ps);
    
    if (first_valid_line) {
      default_frame_id_ = frame.empty() ? "map" : frame;
      first_valid_line = false;
    }
  }
  return !out.empty();
}

void AutoLoopPanel::publishPoseArray(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = poses.empty()
      ? default_frame_id_
      : (poses.front().header.frame_id.empty() ? default_frame_id_ : poses.front().header.frame_id);
  msg.header.stamp = raw_node_->now();
  msg.poses.reserve(poses.size());
  for (const auto& ps : poses) {
    msg.poses.push_back(ps.pose);
  }
  pose_pub_->publish(msg);
}

void AutoLoopPanel::clearFileAndPublishEmpty()
{
  // truncate file
  {
    std::ofstream ofs(resolveFilePath(), std::ios::trunc);
  }
  // publish empty to clear RViz view
  geometry_msgs::msg::PoseArray empty;
  empty.header.frame_id = default_frame_id_;
  empty.header.stamp = raw_node_->now();
  pose_pub_->publish(empty);
}

void AutoLoopPanel::startLoop(const std::vector<geometry_msgs::msg::PoseStamped>& input)
{
  if (input.empty()) {
    lbl_status_->setText("No points to run.");
    return;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
    lbl_status_->setText("navigate_through_poses not available.");
    return;
  }

  stopLoop(); // ensure previous loop is stopped
  loop_active_ = true;

  // Copy poses to local buffer
  auto poses = input;

  loop_thread_ = std::thread([this, poses]() mutable {
    RCLCPP_INFO(raw_node_->get_logger(), "Auto loop started with %zu poses", poses.size());
    lbl_status_->setText("Running...");

    while (loop_active_) {
      // build goal
      NavigateThroughPoses::Goal goal;
      goal.behavior_tree = "";
      goal.poses = poses;

      std::promise<void> done;
      auto fut = done.get_future();

      auto options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      options.result_callback = [this, &done](auto result) {
        RCLCPP_INFO(raw_node_->get_logger(), "Nav result code: %d", static_cast<int>(result.code));
        done.set_value();
      };

      action_client_->async_send_goal(goal, options);
      RCLCPP_INFO(raw_node_->get_logger(), "Sent %zu poses.", goal.poses.size());

      if (fut.valid()) fut.wait();
      if (!loop_active_) break;

      // đảo ngược để chạy về lại
      std::reverse(poses.begin(), poses.end());

      // nghỉ 1s giữa các vòng
      for (int i=0; i<10 && loop_active_; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    lbl_status_->setText("Stopped.");
    RCLCPP_INFO(raw_node_->get_logger(), "Auto loop stopped.");
  });
}

void AutoLoopPanel::stopLoop()
{
  loop_active_ = false;
  if (loop_thread_.joinable()) loop_thread_.join();
}

void AutoLoopPanel::onAuto()
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (!loadFromFile(poses)) {
    lbl_status_->setText("Load failed or empty file.");
    return;
  }
  // đảm bảo hiển thị
  publishPoseArray(poses);
  startLoop(poses);
}

void AutoLoopPanel::onStop()
{
  stopLoop();
}

void AutoLoopPanel::onLoad()
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (!loadFromFile(poses)) {
    lbl_status_->setText("Load failed or empty file.");
    // vẫn publish empty để clear nếu cần
    publishPoseArray(poses);
    return;
  }
  publishPoseArray(poses);
  lbl_status_->setText(QString("Loaded %1 poses.").arg(poses.size()));
}

void AutoLoopPanel::onClear()
{
  stopLoop();
  clearFileAndPublishEmpty();
  lbl_status_->setText("File cleared.");
}

} // namespace nav2_rviz_plugins

PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::AutoLoopPanel, rviz_common::Panel)

