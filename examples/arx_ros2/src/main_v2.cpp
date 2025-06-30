#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include "placo/model/robot_wrapper.h"
#include "placo/kinematics/kinematics_solver.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>
#include <algorithm>
#include <map>

#include "arm_control/msg/pos_cmd.hpp"
#include "arm_control/msg/joint_control.hpp"
#include "xr_msgs/msg/custom.hpp"
#include "xr_to_arx.h"

using xr_msgs::msg::Custom;
using arm_control::msg::PosCmd;
using arm_control::msg::JointControl;

class PlacoIk {
public:
  PlacoIk(const std::string &urdf_path, const std::string &left_effector_name, const std::string &right_effector_name)
    : robot(urdf_path), solver(robot), left_effector_name(left_effector_name), right_effector_name(right_effector_name) {
    solver.dt = 0.01;
    solver.add_kinetic_energy_regularization_task(1e-6);

    // init robot state
    solver.mask_fbase(true);
    solver.enable_velocity_limits(true);
    solver.enable_joint_limits(true);
    robot.update_kinematics();
    std::cout << "init robot.state.q: " << robot.state.q.transpose() << std::endl;

    left_effector_pose_init = robot.get_T_world_frame(left_effector_name);
    left_effector_task = solver.add_frame_task(left_effector_name, left_effector_pose_init);
    left_effector_task.configure(left_effector_name, "soft", 1.0);
    auto left_manipulability = solver.add_manipulability_task(left_effector_name, "both", 1.0);
    left_manipulability.configure("manipulability", "soft", 5e-2);
    std::cout << "init left_effector_pose: " << left_effector_pose_init.translation().transpose() << std::endl;

    right_effector_pose_init = robot.get_T_world_frame(right_effector_name);
    right_effector_task = solver.add_frame_task(right_effector_name, right_effector_pose_init);
    right_effector_task.configure(right_effector_name, "soft", 1.0);
    auto right_manipulability = solver.add_manipulability_task(right_effector_name, "both", 1.0);
    right_manipulability.configure("manipulability", "soft", 5e-2);
    std::cout << "init right_effector_pose: " << right_effector_pose_init.translation().transpose() << std::endl;

    auto joints_task = solver.add_joints_task();
    std::map<std::string, double> joints;
    for (const auto& joint : robot.joint_names()) {
      joints_task.set_joint(joint, 0.0);
    }
    joints_task.configure("joints_regularization", "soft", 5e-4);
  }
  void set_target_pose(const Eigen::Affine3d &left_target_pose, const Eigen::Affine3d &right_target_pose) {
    left_effector_task.set_T_world_frame(left_target_pose);
    right_effector_task.set_T_world_frame(right_target_pose);
    solver.solve(true);
    robot.update_kinematics();
  }
  void get_joint_positions(Eigen::VectorXd &left_q, Eigen::VectorXd &right_q) {
    auto q = robot.state.q;
    q = q.tail(q.size() - 7);
    left_q = q.tail(7);
    right_q = q.head(7);
  }
  void apply_relative_pose(
    const Eigen::Affine3d &left_relative_pose,
    const Eigen::Affine3d &right_relative_pose,
    Eigen::Affine3d &left_target_pose,
    Eigen::Affine3d &right_target_pose) {
    
    auto left_q_init = Eigen::Quaterniond(left_effector_pose_init.rotation());
    auto left_q_relative = Eigen::Quaterniond(left_relative_pose.rotation());
    left_target_pose.translation() = left_effector_pose_init.translation() + left_q_init * left_relative_pose.translation();
    left_target_pose.linear() = (left_q_init.inverse() * left_q_relative).toRotationMatrix();

    auto right_q_init = Eigen::Quaterniond(right_effector_pose_init.rotation());
    auto right_q_relative = Eigen::Quaterniond(right_relative_pose.rotation());
    right_target_pose.translation() = right_effector_pose_init.translation() + right_q_init * right_relative_pose.translation();
    right_target_pose.linear() = (right_q_relative * right_q_init).toRotationMatrix();
  }
  void reset() {
    robot.reset();
    robot.update_kinematics();
    left_effector_pose_init = robot.get_T_world_frame(left_effector_name);
    right_effector_pose_init = robot.get_T_world_frame(right_effector_name);
  }
private:
  placo::model::RobotWrapper robot;
  placo::kinematics::KinematicsSolver solver;
  placo::kinematics::FrameTask left_effector_task, right_effector_task;
  std::string left_effector_name, right_effector_name;
  Eigen::Affine3d left_effector_pose_init, right_effector_pose_init;
};

/**
 *  /xr_pose: sub xr pose
 * 
 *     xr2arx_: xr_pose to arx_pose   
 *
 *     placo_ik_: arx_pose to joint positions
 *   
 *  /joint_control, /joint_control2: pub joint positions to arx
 *
 */
class MainV2Node : public rclcpp::Node {
public:
  MainV2Node() : Node("main_v2_node") {
    xr2arx_ = std::make_shared<Xr2Arx>();
    placo_ik_ = std::make_shared<PlacoIk>("src/arx_ros2/urdf/X7S_arm_only.urdf", "link11", "link20");
    xr_sub_ = this->create_subscription<Custom>(
      "/xr_pose", 10,
      [this](Custom::SharedPtr msg) { xr2arx_->xrPoseCallback(msg); }
    );
    left_pub_ = this->create_publisher<JointControl>("/joint_control", 10);
    right_pub_ = this->create_publisher<JointControl>("/joint_control2", 10);
    left_ee_pub_ = this->create_publisher<PosCmd>("/ARX_VR_L", 10);
    right_ee_pub_ = this->create_publisher<PosCmd>("/ARX_VR_R", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        frame_count++;
        bool active;
        arm_control::msg::PosCmd left_msg, right_msg;
        JointControl left_joint_msg, right_joint_msg;
        xr2arx_->getPosCmd(left_msg, right_msg, active);
        if (active) {
          Eigen::Affine3d left_relative_pose = Eigen::Affine3d::Identity();
          left_relative_pose.translation() = Eigen::Vector3d(left_msg.x, left_msg.y, left_msg.z);
          left_relative_pose.rotate(Eigen::AngleAxisd(left_msg.roll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(left_msg.pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(left_msg.yaw, Eigen::Vector3d::UnitZ()));

          Eigen::Affine3d right_relative_pose = Eigen::Affine3d::Identity();
          right_relative_pose.translation() = Eigen::Vector3d(right_msg.x, right_msg.y, right_msg.z);
          right_relative_pose.rotate(Eigen::AngleAxisd(right_msg.roll, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(right_msg.pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(right_msg.yaw, Eigen::Vector3d::UnitZ()));

          Eigen::Affine3d left_target_pose, right_target_pose;
          placo_ik_->apply_relative_pose(left_relative_pose, right_relative_pose, left_target_pose, right_target_pose);
          placo_ik_->set_target_pose(left_target_pose, right_target_pose);
          Eigen::VectorXd left_joint_positions, right_joint_positions;
          placo_ik_->get_joint_positions(left_joint_positions, right_joint_positions);

          if (shouldPrint()) {
            // std::cout << "left_pose: " << left_relative_pose.translation().transpose() << std::endl;
            // std::cout << "left_joint_positions: " << left_joint_positions.transpose() << std::endl;
            std::cout << "right_msg rpy: " << right_msg.roll << " " << right_msg.pitch << " " << right_msg.yaw << std::endl;
            std::cout << "right_joint_positions: " << right_joint_positions.transpose() << std::endl;
          }

          for (int i = 0; i < 7; i++) {
            left_joint_msg.joint_pos[i] = left_joint_positions[i];
            right_joint_msg.joint_pos[i] = right_joint_positions[i];
          }
          left_joint_msg.joint_pos[7] = left_msg.gripper;
          right_joint_msg.joint_pos[7] = right_msg.gripper;
          left_joint_msg.mode = 0;
          right_joint_msg.mode = 0;
        } else {
          placo_ik_->reset();
          xr2arx_->reset();
          xr2arx_->getPosCmd(left_msg, right_msg, active);
        }
        left_pub_->publish(left_joint_msg);
        right_pub_->publish(right_joint_msg);
        left_ee_pub_->publish(left_msg);
        right_ee_pub_->publish(right_msg);
      }
    );
  }
private:
  std::shared_ptr<Xr2Arx> xr2arx_;
  std::shared_ptr<PlacoIk> placo_ik_;
  rclcpp::Subscription<Custom>::SharedPtr xr_sub_;
  rclcpp::Publisher<JointControl>::SharedPtr left_pub_, right_pub_;
  rclcpp::Publisher<PosCmd>::SharedPtr left_ee_pub_, right_ee_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_count = 0;
  int print_interval = 10;
  inline bool shouldPrint() { return frame_count % print_interval == 0; }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MainV2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 
