#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp" // 【新增】布尔开关消息
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class VisionProcessor : public rclcpp::Node
{
public:
  VisionProcessor() : Node("vision_processor"), is_vision_active_(true) // 默认开启视觉追踪
  {
    // 下发 3D 位姿的发布者
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
    
    // 【新增】接收行为树“静音”指令的订阅者
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vision/enable", 10,
      std::bind(&VisionProcessor::enable_callback, this, std::placeholders::_1));

    // 接收图像的订阅者
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&VisionProcessor::image_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "视觉处理节点已启动！静音开关已接入 (/vision/enable)。");
  }

private:
  // 【新增】开关回调函数
  void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_vision_active_ = msg->data;
    if (is_vision_active_) {
      RCLCPP_INFO(this->get_logger(), "大脑指令：恢复视觉追踪！");
    } else {
      RCLCPP_WARN(this->get_logger(), "大脑指令：视觉已静音，等待抓取动作完成...");
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::Mat hsv_frame, mask1, mask2, final_mask;

    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv_frame, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, final_mask);

    cv::Moments M = cv::moments(final_mask);
    if (M.m00 > 0) {
      int cx = int(M.m10 / M.m00);
      int cy = int(M.m01 / M.m00);

      cv::circle(frame, cv::Point(cx, cy), 15, cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(frame, cv::Point(cx, cy), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);

      double anchor_world_x = 0.700;
      double anchor_world_y = 0.000;
      double anchor_pixel_x = 200.0; 
      double anchor_pixel_y = 119.0; 
      
      double target_z = 0.050 + 0.010; 
      
      double pixel_to_meter_x = 0.00714; 
      double pixel_to_meter_y = 0.00769; 

      double world_x = anchor_world_x - (cx - anchor_pixel_x) * pixel_to_meter_x;
      double world_y = anchor_world_y - (cy - anchor_pixel_y) * pixel_to_meter_y;

      // 【核心修改】：只有在视觉处于激活状态时，才向下发坐标
      if (is_vision_active_) {
        geometry_msgs::msg::Pose target_msg;
        target_msg.position.x = world_x;
        target_msg.position.y = world_y;
        target_msg.position.z = target_z;
        target_msg.orientation.w = 1.0; 

        pose_pub_->publish(target_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
          "精确追踪: [X=%.3f, Y=%.3f] (视觉引导中)", world_x, world_y);
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "视野中丢失红色目标！");
    }

    cv::imshow("Camera View", frame);
    cv::imshow("Red Mask", final_mask);
    cv::waitKey(1);
  }

  bool is_vision_active_; // 开关标志位
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionProcessor>());
  rclcpp::shutdown();
  return 0;
}