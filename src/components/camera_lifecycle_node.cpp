#include "picking_system_core/components/camera_lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace picking_system_core
{

CameraLifecycleNode::CameraLifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("camera_node", options)
{
  RCLCPP_INFO(this->get_logger(), "节点已创建，处于 Unconfigured 状态。");
}

CallbackReturn CameraLifecycleNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "进入 [Configuring] 阶段：正在初始化相机驱动和分配内存...");
  
  // 初始化发布者，注意此时还没有真正开始发布
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CameraLifecycleNode::timer_callback, this));
    
  return CallbackReturn::SUCCESS; // 如果硬件找不到，可以返回 FAILURE，系统将报错截断
}

CallbackReturn CameraLifecycleNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "进入 [Activating] 阶段：开启数据流！");
  image_pub_->on_activate(); // 必须显式激活发布者
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "进入 [Deactivating] 阶段：暂停数据发布。");
  image_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "进入 [CleaningUp] 阶段：释放硬件资源。");
  image_pub_.reset();
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "系统关闭中...");
  return CallbackReturn::SUCCESS;
}

void CameraLifecycleNode::timer_callback()
{
  // 只有在 Active 状态下，发布者才会真正发数据出去
  if (image_pub_->is_activated()) {
    sensor_msgs::msg::Image msg;
    // msg 填充逻辑...
    image_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "发布了一帧图像");
  }
}

}  // namespace picking_system_core

// 注册组件！这是最关键的一步，让 ROS 2 能找到它
RCLCPP_COMPONENTS_REGISTER_NODE(picking_system_core::CameraLifecycleNode)