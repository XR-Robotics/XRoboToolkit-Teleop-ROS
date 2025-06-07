#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>
#include <algorithm>

#include "arm_control/msg/pos_cmd.hpp"
#include "arm_control/msg/joint_control.hpp"
#include "xr_msgs/msg/custom.hpp"
#include "xr_to_arx.h"

using xr_msgs::msg::Custom;
using arm_control::msg::PosCmd;

/**
 *  /xr_pose: sub xr pose
 * 
 *     xr2arx_: xr_pose to arx_pose   
 *   
 *  /ARX_VR_L, /ARX_VR_R: pub arx_pose to arx
 *
 */
class MainV1Node : public rclcpp::Node {
public:
  MainV1Node() : Node("main_v1_node") {
    xr2arx_ = std::make_shared<Xr2Arx>();
    xr_sub_ = this->create_subscription<Custom>(
      "/xr_pose", 10,
      [this](Custom::SharedPtr msg) { xr2arx_->xrPoseCallback(msg); }
    );
    left_ee_pub_ = this->create_publisher<PosCmd>("/ARX_VR_L", 10);
    right_ee_pub_ = this->create_publisher<PosCmd>("/ARX_VR_R", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        frame_count++;
        bool active;
        arm_control::msg::PosCmd left_msg, right_msg;
        xr2arx_->getPosCmd(left_msg, right_msg, active);
        if (!active) {
          xr2arx_->reset();
          xr2arx_->getPosCmd(left_msg, right_msg, active);
        }
        left_ee_pub_->publish(left_msg);
        right_ee_pub_->publish(right_msg);

        if (shouldPrint()) {
          RCLCPP_INFO(rclcpp::get_logger("xr_robot"), "active: %d", active);
          RCLCPP_INFO(rclcpp::get_logger("xr_robot"), "left_ee_pose: %f, %f, %f", left_msg.x, left_msg.y, left_msg.z);
          RCLCPP_INFO(rclcpp::get_logger("xr_robot"), "right_ee_pose: %f, %f, %f", right_msg.x, right_msg.y, right_msg.z);
        }
      }
    );
  }
private:
  std::shared_ptr<Xr2Arx> xr2arx_;
  rclcpp::Subscription<Custom>::SharedPtr xr_sub_;
  rclcpp::Publisher<PosCmd>::SharedPtr left_ee_pub_, right_ee_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_count = 0;
  int print_interval = 10;
  inline bool shouldPrint() { return frame_count % print_interval == 0; }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainV1Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 