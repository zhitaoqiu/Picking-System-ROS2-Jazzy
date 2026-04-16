#ifndef PICKING_SYSTEM_CORE__CAMERA_LIFECYCLE_NODE_HPP_
#define PICKING_SYSTEM_CORE__CAMERA_LIFECYCLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace picking_system_core
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CameraLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CameraLifecycleNode(const rclcpp::NodeOptions & options);
  ~CameraLifecycleNode() override = default;

protected:
  // 重载生命周期状态转换的回调函数
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // 模拟的图像发布者
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void timer_callback();
};

}  // namespace picking_system_core

#endif  // PICKING_SYSTEM_CORE__CAMERA_LIFECYCLE_NODE_HPP_