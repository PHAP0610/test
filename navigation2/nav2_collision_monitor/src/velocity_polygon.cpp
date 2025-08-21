#include "nav2_collision_monitor/velocity_polygon.hpp"

#include <algorithm>
#include <cctype>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_collision_monitor
{

// Default ctor cho pluginlib: gọi base với tham số "dummy"
VelocityPolygon::VelocityPolygon()
: Polygon(
    nav2_util::LifecycleNode::WeakPtr{},     // node rỗng (chỉ lock sau khi onConfigure)
    std::string{},                           // polygon_name rỗng
    std::shared_ptr<tf2_ros::Buffer>{},      // tf_buffer null
    std::string{},                           // base_frame_id rỗng
    tf2::durationFromSec(0.0)                // tolerance 0s
  ),
  polygon_name_(""),
  node_(nav2_util::LifecycleNode::WeakPtr{}),
  tf_buffer_(nullptr)
{
  // Không làm gì ở đây: chưa có node/logger hợp lệ để dùng
}

VelocityPolygon::~VelocityPolygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying VelocityPolygon (backport)", polygon_name_.c_str());
}

bool VelocityPolygon::getPolygonFromString(const std::string & s, std::vector<Point> & out)
{
  std::string str = s;
  str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());
  if (str.size() < 5 || str.front() != '[') {
    return false;
  }

  out.clear();
  size_t i = 0;
  while (i < str.size()) {
    size_t lb = str.find('[', i);
    if (lb == std::string::npos) break;
    size_t comma = str.find(',', lb + 1);
    size_t rb = str.find(']', lb + 1);
    if (comma == std::string::npos || rb == std::string::npos || comma > rb) {
      return false;
    }
    try {
      float x = std::stof(str.substr(lb + 1, comma - (lb + 1)));
      float y = std::stof(str.substr(comma + 1, rb - (comma + 1)));
      out.push_back(Point{ x, y });
    } catch (...) {
      return false;
    }
    i = rb + 1;
  }
  return !out.empty();
}

bool VelocityPolygon::getParameters(std::string & polygon_pub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  clock_ = node->get_clock();

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  try {
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".velocity_polygons",
      rclcpp::ParameterValue(std::vector<std::string>{}));
    std::vector<std::string> velocity_polygons;
    node->get_parameter(polygon_name_ + ".velocity_polygons", velocity_polygons);

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".holonomic", rclcpp::ParameterValue(false));
    holonomic_ = node->get_parameter(polygon_name_ + ".holonomic").as_bool();

    sub_polygons_.clear();
    sub_polygons_.reserve(velocity_polygons.size());

    for (const std::string & vp_name : velocity_polygons) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + vp_name + ".points",
        rclcpp::ParameterValue(std::string("[]")));
      const std::string poly_string =
        node->get_parameter(polygon_name_ + "." + vp_name + ".points").as_string();

      std::vector<Point> poly;
      if (!getPolygonFromString(poly_string, poly)) {
        RCLCPP_ERROR(
          logger_, "[%s.%s] 'points' must be a JSON-like string of pairs [[x,y],...]",
          polygon_name_.c_str(), vp_name.c_str());
        return false;
      }

      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + vp_name + ".linear_min", rclcpp::ParameterValue(0.0));
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + vp_name + ".linear_max", rclcpp::ParameterValue(0.0));
      const double linear_min =
        node->get_parameter(polygon_name_ + "." + vp_name + ".linear_min").as_double();
      const double linear_max =
        node->get_parameter(polygon_name_ + "." + vp_name + ".linear_max").as_double();

      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + vp_name + ".theta_min", rclcpp::ParameterValue(0.0));
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + "." + vp_name + ".theta_max", rclcpp::ParameterValue(0.0));
      const double theta_min =
        node->get_parameter(polygon_name_ + "." + vp_name + ".theta_min").as_double();
      const double theta_max =
        node->get_parameter(polygon_name_ + "." + vp_name + ".theta_max").as_double();

      double direction_end_angle = M_PI;
      double direction_start_angle = -M_PI;
      if (holonomic_) {
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + vp_name + ".direction_end_angle",
          rclcpp::ParameterValue(M_PI));
        nav2_util::declare_parameter_if_not_declared(
          node, polygon_name_ + "." + vp_name + ".direction_start_angle",
          rclcpp::ParameterValue(-M_PI));
        direction_end_angle =
          node->get_parameter(polygon_name_ + "." + vp_name + ".direction_end_angle").as_double();
        direction_start_angle =
          node->get_parameter(polygon_name_ + "." + vp_name + ".direction_start_angle").as_double();
      }

      SubPolygonParameter sub {
        poly, vp_name, linear_min, linear_max, theta_min, theta_max,
        direction_end_angle, direction_start_angle
      };
      sub_polygons_.push_back(std::move(sub));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_, "[%s]: Error while getting polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool VelocityPolygon::isInRange(const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon)
{
  bool in_range =
    (cmd_vel_in.x  <= sub_polygon.linear_max_ && cmd_vel_in.x  >= sub_polygon.linear_min_ &&
     cmd_vel_in.tw <= sub_polygon.theta_max_  && cmd_vel_in.tw >= sub_polygon.theta_min_);

  if (holonomic_) {
    const double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
    if (sub_polygon.direction_start_angle_ <= sub_polygon.direction_end_angle_) {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ &&
         direction <= sub_polygon.direction_end_angle_);
    } else {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ ||
         direction <= sub_polygon.direction_end_angle_);
    }
  }

  return in_range;
}

void VelocityPolygon::updatePolygon(const Velocity & cmd_vel_in)
{
  for (auto & sub_polygon : sub_polygons_) {
    if (isInRange(cmd_vel_in, sub_polygon)) {
      poly_ = sub_polygon.poly_;

      polygon_.points.clear();
      for (const Point & p : poly_) {
        geometry_msgs::msg::Point32 p_s;
        p_s.x = p.x;
        p_s.y = p.y;
        polygon_.points.push_back(p_s);
      }
      return;
    }
  }

  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 2.0,
    "Velocity not covered by any velocity polygon. x: %.3f y: %.3f tw: %.3f",
    cmd_vel_in.x, cmd_vel_in.y, cmd_vel_in.tw);
  return;
}

// pluginlib export
PLUGINLIB_EXPORT_CLASS(nav2_collision_monitor::VelocityPolygon,
                       nav2_collision_monitor::Polygon)

}  // namespace nav2_collision_monitor

