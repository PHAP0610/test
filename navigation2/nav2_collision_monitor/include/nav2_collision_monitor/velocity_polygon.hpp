#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <utility>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/types.hpp" 

namespace nav2_collision_monitor
{

struct SubPolygonParameter
{
  std::vector<Point> poly_;
  std::string name_;
  double linear_min_{0.0};
  double linear_max_{0.0};
  double theta_min_{0.0};
  double theta_max_{0.0};
  double direction_end_angle_{M_PI};
  double direction_start_angle_{-M_PI};
};

class VelocityPolygon : public Polygon
{
public:
  VelocityPolygon();

  VelocityPolygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);

  ~VelocityPolygon();

  bool getParameters(std::string & polygon_pub_topic);

  void updatePolygon(const Velocity & cmd_vel_in);

  bool isInRange(const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon);

private:
  bool getPolygonFromString(const std::string & s, std::vector<Point> & out);
  bool loadVelocityPolygons();

private:
  std::vector<SubPolygonParameter> sub_polygons_;
  bool holonomic_{false};

  std::string polygon_name_;

  nav2_util::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace nav2_collision_monitor

