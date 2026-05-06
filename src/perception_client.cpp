#include "industrial_bt_framework/perception_client.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace industrial_bt_framework {

using namespace std::chrono_literals;

PerceptionClient::PerceptionClient(
  rclcpp::Node::SharedPtr node, const std::string & pose_topic)
: node_(node), pose_topic_(pose_topic)
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    pose_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PerceptionClient::onPoses, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "PerceptionClient subscribed to '%s'", pose_topic_.c_str());
}

void PerceptionClient::onPoses(geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  latest_ = msg;
}

std::optional<geometry_msgs::msg::PoseStamped>
PerceptionClient::waitForPose(
  std::size_t object_index, std::chrono::duration<double> timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (latest_ && latest_->poses.size() > object_index) {
        geometry_msgs::msg::PoseStamped out;
        out.header = latest_->header;
        out.pose   = latest_->poses[object_index];
        return out;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) break;
    std::this_thread::sleep_for(20ms);
  }

  RCLCPP_WARN(node_->get_logger(),
    "PerceptionClient::waitForPose timed out on '%s' (index %zu)",
    pose_topic_.c_str(), object_index);
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Pose>
PerceptionClient::transformToFrame(
  const geometry_msgs::msg::PoseStamped & in,
  const std::string & target_frame,
  std::chrono::duration<double> tf_timeout)
{
  const auto tf_timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf_timeout);

  try {
    geometry_msgs::msg::PoseStamped out =
      tf_buffer_->transform(in, target_frame, tf_timeout_ns);
    return out.pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "tf2 exact-stamp lookup failed (%s), retrying at latest time...", ex.what());
  }

  try {
    geometry_msgs::msg::PoseStamped latest_in = in;
    latest_in.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    geometry_msgs::msg::PoseStamped out =
      tf_buffer_->transform(latest_in, target_frame, tf_timeout_ns);
    return out.pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "tf2 transform '%s' -> '%s' failed at latest time too: %s",
      in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

}  // namespace industrial_bt_framework
