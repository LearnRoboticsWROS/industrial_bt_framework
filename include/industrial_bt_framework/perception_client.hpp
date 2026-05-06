#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace industrial_bt_framework {

// Subscribes to a PoseArray topic (typically published by a 6D pose estimator)
// and offers blocking access to the latest pose plus a tf2 transform helper.
//
// Thread-safety: the latest message is cached under a mutex; the tf2 Buffer is
// internally thread-safe. The class assumes the owning rclcpp::Node is spun
// by an executor elsewhere (MultiThreadedExecutor in bt_runner_node).
class PerceptionClient
{
public:
  PerceptionClient(rclcpp::Node::SharedPtr node, const std::string & pose_topic);

  // Wait until the cached PoseArray contains at least `object_index + 1` poses,
  // or `timeout` elapses. Returns the requested pose with the array's header
  // (frame + stamp) so callers can tf-transform it against the right time.
  std::optional<geometry_msgs::msg::PoseStamped> waitForPose(
    std::size_t object_index,
    std::chrono::duration<double> timeout);

  // Transform `in` into `target_frame` using tf2. First attempts the exact
  // stamp carried by `in.header`; if that fails, retries with
  // tf2::TimePointZero (latest available) to be robust against PC/Jetson
  // clock skew.
  std::optional<geometry_msgs::msg::Pose> transformToFrame(
    const geometry_msgs::msg::PoseStamped & in,
    const std::string & target_frame,
    std::chrono::duration<double> tf_timeout);

private:
  void onPoses(geometry_msgs::msg::PoseArray::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::string pose_topic_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mtx_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr latest_;
};

}  // namespace industrial_bt_framework
