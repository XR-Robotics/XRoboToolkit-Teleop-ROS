#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "arm_control/msg/joint_control.hpp"
#include "UrdfVizClient.h"

using arm_control::msg::JointControl;

/**
 * UrdfViz node to control arx dual-armed robot in urdf_viz.
 *  
 *    /joint_control: subscribe left arm joint control
 *    /joint_control2: subscribe right arm joint control
 * 
 *    joint3, joint4: head
 *    joint5-11: left arm
 *    joint14-20: right arm
 * 
 */

class UrdfViz : public rclcpp::Node {
public:
  UrdfViz() : Node("urdf_viz") {
    std::string urdf_ip_addr = std::getenv("URDF_VIZ_IP_ADDR");
    std::cout << "urdf-viz ip: " << urdf_ip_addr << std::endl;
    robot_ = std::make_shared<URDF_VIZ::Robot>(urdf_ip_addr);

    left_sub_ = this->create_subscription<JointControl>("/joint_control", 10,
      [this](JointControl::SharedPtr msg) { 
        frame_count++;
        std::vector<std::string> left_joint_names = {"joint5", "joint6", "joint7", "joint8", "joint9", "joint10", "joint11"};
        std::vector<double> left_joint_pos(7);
        for (int i = 0; i < 7; i++) {
          left_joint_pos[i] = msg->joint_pos[i];
        }
        auto result = this->robot_->set_joint_positions(left_joint_names, left_joint_pos);
        if (shouldPrint()) {
          RCLCPP_INFO(rclcpp::get_logger("urdf_viz"), "left: %f %f %f %f %f %f %f", msg->joint_pos[0], msg->joint_pos[1], msg->joint_pos[2], msg->joint_pos[3], msg->joint_pos[4], msg->joint_pos[5], msg->joint_pos[6]);
          RCLCPP_INFO(rclcpp::get_logger("urdf_viz"), "result: %s", result.dump(2).c_str());
        }
      }
    );
    right_sub_ = this->create_subscription<JointControl>("/joint_control2", 10,
      [this](JointControl::SharedPtr msg) { 
        std::vector<std::string> right_joint_names = {"joint14", "joint15", "joint16", "joint17", "joint18", "joint19", "joint20"};
        std::vector<double> right_joint_pos(7);
        for (int i = 0; i < 7; i++) {
          right_joint_pos[i] = msg->joint_pos[i];
        }
        auto result = this->robot_->set_joint_positions(right_joint_names, right_joint_pos);
        if (shouldPrint()) {
          RCLCPP_INFO(rclcpp::get_logger("urdf_viz"), "right: %f %f %f %f %f %f %f", msg->joint_pos[0], msg->joint_pos[1], msg->joint_pos[2], msg->joint_pos[3], msg->joint_pos[4], msg->joint_pos[5], msg->joint_pos[6]);
          RCLCPP_INFO(rclcpp::get_logger("urdf_viz"), "result: %s", result.dump(2).c_str());
        }
      }
    );
  }
private:
  rclcpp::Subscription<JointControl>::SharedPtr left_sub_, right_sub_;
  std::shared_ptr<URDF_VIZ::Robot> robot_;
  int frame_count = 0;
  int print_interval = 100;
  inline bool shouldPrint() { return frame_count % print_interval == 0; }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UrdfViz>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 
