#include <Eigen/Eigen>
#include "placo/model/robot_wrapper.h"
#include "placo/kinematics/kinematics_solver.h"

#include <ros/ros.h>
#include <pos_cmd_msg/PosCmd.h>
#include <xr_msgs/Custom.h>
#include <arm_control/JointControl.h>
#include <boost/array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <atomic>
#include <iostream>


struct RawPose {
  float x;
  float y;
  float z;
  float rx;
  float ry;
  float rz;
  float rw;

  RawPose() : x(0), y(0), z(0), rx(0), ry(0), rz(0), rw(1) {} // 默认构造函数

  RawPose(const boost::array<float, 7>& pose) {
    x = pose[0];
    y = pose[1];
    z = pose[2];
    rx = pose[3];
    ry = pose[4];
    rz = pose[5];
    rw = pose[6];
  }

  void GetRPY(double& roll, double& pitch, double& yaw) {
    tf2::Quaternion q(rx, ry, rz, rw);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
  }

  void Reset() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    rx = 0.0;
    ry = 0.0;
    rz = 0.0;
    rw = 1.0;
  }
};


void computeRelativePose(const RawPose& origin, const RawPose& current, RawPose& relative) {
  relative.x = current.x - origin.x;
  relative.y = current.y - origin.y;
  relative.z = current.z - origin.z;

  // compute relative rotation in quaternion form
  tf2::Quaternion q_origin(origin.rx, origin.ry, origin.rz, origin.rw);
  tf2::Quaternion q_current(current.rx, current.ry, current.rz, current.rw);
  tf2::Quaternion q_relative = q_current * q_origin.inverse();
  relative.rx = q_relative.x();
  relative.ry = q_relative.y();
  relative.rz = q_relative.z();
  relative.rw = q_relative.w();
}

/**
 * @brief Convert XR pose to ARX pose, (x, y, z) => (-z, -x, y)
 * 
 * @param vr_pose: x,y,z,rx,ry,rz,rw
 * @param arx_pose: x,y,z,roll,pitch,yaw
 */
void XR2ARX(const RawPose& vr_pose, RawPose& arx_pose, float scale = 1.0) {
  arx_pose.x = -vr_pose.z * scale;
  arx_pose.y = -vr_pose.x * scale; 
  arx_pose.z = vr_pose.y * scale;

  tf2::Quaternion q_vr(vr_pose.rx, vr_pose.ry, vr_pose.rz, vr_pose.rw);
  tf2::Matrix3x3 R_vr(q_vr);
  tf2::Matrix3x3 T(0, 0, -1, -1, 0, 0, 0, 1, 0);
  tf2::Matrix3x3 R_arx = T * R_vr * T.transpose();
  tf2::Quaternion q_arx;
  R_arx.getRotation(q_arx);
  arx_pose.rx = q_arx.x();
  arx_pose.ry = q_arx.y();
  arx_pose.rz = q_arx.z();
  arx_pose.rw = q_arx.w();
}

/**
 *                ------                 ---------- 
 *        _______|      |_______________|          |__________
 *                      true                        true
 * cur    00000000111111000000000000000001111111111000000000000              
 * prev   00000000011111100000000000000000111111111100000000000
 * active 00000000000000100000000000000000000000000100000000000               
 */
class Trigger {
  int clicked_count;
  int clicked_interval;
  bool clicked_valid;
  bool prev_status;
  bool cur_status;

 public:
  Trigger(): clicked_count(0), clicked_interval(50), prev_status(false), cur_status(false), clicked_valid(false) {}

  bool isActive(float left_trigger, float right_trigger) {
    clicked_valid = clicked_count > clicked_interval;

    if (left_trigger == 1.0 && right_trigger == 1.0) {
      clicked_count++;
      cur_status = true;
    } else {
      clicked_count = 0;
      cur_status = false;
    }

    bool active = prev_status && !cur_status;

    // std::cout << "clicked_count: " << clicked_count << ", clicked_interval: " << clicked_interval << std::endl;
    // std::cout << "cur_status: " << cur_status << ", prev_status: " << prev_status << ", active: " << active << std::endl;

    if (clicked_valid) {
      if (active) {
        clicked_count = 0;
        prev_status = false;
        cur_status = false;
        clicked_valid = false;

      }
      prev_status = cur_status;
      return active;
    } else {
      prev_status = cur_status;
      return false;
    }
  }

};

class XrRobot {
  std::mutex pos_cmd_mutex;
  pos_cmd_msg::PosCmd global_pos_cmd_left;
  pos_cmd_msg::PosCmd global_pos_cmd_right;

  std::atomic<bool> is_running;
  int is_running_count = 0;
  int frame_count = 0;
  int print_interval = 100;
  std::shared_ptr<Trigger> trigger;
  RawPose left_pose_origin;
  RawPose right_pose_origin;
  float body_height = 0.0;
  float height_speed = 0.05;

  float body_x = 0.0;
  float body_y = 0.0;
  float body_rot = 0.0;
  float body_speed = 0.5;

 public:
  XrRobot() {
    trigger = std::make_shared<Trigger>();
    reset();
  }
  ~XrRobot() {}

  void reset();

  // Get XR pose
  void xrPoseCallback(const xr_msgs::Custom::ConstPtr& msg);

  // Set ARX pose
  void getPosCmd(pos_cmd_msg::PosCmd& left_msg, pos_cmd_msg::PosCmd& right_msg, bool& active);

 private:
  bool shouldPrint() { return frame_count % print_interval == 0; }


};

void XrRobot::reset() {
  std::lock_guard<std::mutex> lock(pos_cmd_mutex);
  global_pos_cmd_left.x = 0.0;
  global_pos_cmd_left.y = 0.0;
  global_pos_cmd_left.z = 0.0;
  global_pos_cmd_left.roll = 0.0;
  global_pos_cmd_left.pitch = 0.0;
  global_pos_cmd_left.yaw = 0.0;
  global_pos_cmd_left.gripper = 0.0;
  global_pos_cmd_left.chx = 0.0;
  global_pos_cmd_left.chy = 0.0;
  global_pos_cmd_left.chz = 0.0;
  global_pos_cmd_left.height = 0.0;
  global_pos_cmd_left.head_pit = 0.0;
  global_pos_cmd_left.head_yaw = 0.0;

  global_pos_cmd_right.x = 0.0;
  global_pos_cmd_right.y = 0.0;
  global_pos_cmd_right.z = 0.0;
  global_pos_cmd_right.roll = 0.0;
  global_pos_cmd_right.pitch = 0.0;
  global_pos_cmd_right.yaw = 0.0;
  global_pos_cmd_right.gripper = 0.0;

  frame_count = 0;
  body_height = 0.0;
  body_x = 0.0;   // forward/backward
  body_y = 0.0;   // left/right
  body_rot = 0.0; // rotate
}

void XrRobot::xrPoseCallback(const xr_msgs::Custom::ConstPtr& msg)
{
  frame_count++;
  RawPose head_pose, left_pose, right_pose;
  XR2ARX(RawPose(msg->head.pose), head_pose);
  XR2ARX(RawPose(msg->left_controller.pose), left_pose);
  XR2ARX(RawPose(msg->right_controller.pose), right_pose);

  if (shouldPrint()) {
    ROS_INFO("Received XR msg, timestamp: %ld", msg->timestamp_ns);
  } 

  bool isTriggerActive = trigger->isActive(msg->left_controller.trigger, msg->right_controller.trigger);
  if (isTriggerActive) {
    // trigger for start tracking
    if (!is_running.load()) {
      is_running.store(true);
      left_pose_origin = left_pose;
      right_pose_origin = right_pose;
    } else {
      // trigger for stop tracking
      is_running.store(false);
      left_pose_origin.Reset();
      right_pose_origin.Reset();
      reset();
    }
  }

  if (is_running.load()) {
    RawPose left_pose_relative;
    RawPose right_pose_relative;
    computeRelativePose(left_pose_origin, left_pose, left_pose_relative);
    computeRelativePose(right_pose_origin, right_pose, right_pose_relative);
    double left_roll, left_pitch, left_yaw;
    double right_roll, right_pitch, right_yaw;
    double head_roll, head_pitch, head_yaw;
    left_pose_relative.GetRPY(left_roll, left_pitch, left_yaw);
    right_pose_relative.GetRPY(right_roll, right_pitch, right_yaw);
    head_pose.GetRPY(head_roll, head_pitch, head_yaw);

    if (msg->left_controller.axis_y > 0.5) {
      body_height += 1.0 * height_speed;
    } else if (msg->left_controller.axis_y < -0.5) {
      body_height += -1.0 * height_speed;
    }
    body_height = std::max(0.0f, std::min(body_height, 18.0f));
    if (msg->right_controller.axis_x > 0.5) {
      body_y += 1.0 * body_speed;
    } else if (msg->right_controller.axis_x < -0.5) {
      body_y += -1.0 * body_speed;
    }
    if (msg->right_controller.axis_y > 0.5) {
      body_x += 1.0 * body_speed;
    } else if (msg->right_controller.axis_y < -0.5) {
      body_x += -1.0 * body_speed;
    }
    if (msg->right_controller.primary_button) {
      body_rot += 1.0 * body_speed;
    } else if (msg->right_controller.secondary_button) {
      body_rot += -1.0 * body_speed;
    }
    if (shouldPrint()) {
      ROS_INFO("body_height: %f, body_x: %f, body_y: %f, body_rot: %f", body_height, body_x, body_y, body_rot);
    }

    {
      std::lock_guard<std::mutex> lock(pos_cmd_mutex);

      global_pos_cmd_left.head_pit = head_pitch * -1;
      global_pos_cmd_left.head_yaw = head_yaw;
      global_pos_cmd_left.height = body_height;

      global_pos_cmd_left.chx = body_x;
      global_pos_cmd_left.chy = body_y;
      global_pos_cmd_left.chz = body_rot;

      global_pos_cmd_left.x = left_pose_relative.x;
      global_pos_cmd_left.y = left_pose_relative.y;
      global_pos_cmd_left.z = left_pose_relative.z;
      global_pos_cmd_left.roll = left_roll;
      global_pos_cmd_left.pitch = left_pitch;
      global_pos_cmd_left.yaw = left_yaw;
      global_pos_cmd_left.gripper = msg->left_controller.gripper * 5.0;

      global_pos_cmd_right.x = right_pose_relative.x;
      global_pos_cmd_right.y = right_pose_relative.y;
      global_pos_cmd_right.z = right_pose_relative.z;
      global_pos_cmd_right.roll = right_roll;
      global_pos_cmd_right.pitch = right_pitch;
      global_pos_cmd_right.yaw = right_yaw;
      global_pos_cmd_right.gripper = msg->right_controller.gripper * 5.0;
    }
  }
  if (is_running.load()) {
    is_running_count++;
  }
}

void XrRobot::getPosCmd(pos_cmd_msg::PosCmd& left_msg, pos_cmd_msg::PosCmd& right_msg, bool& active) {
  active = is_running.load();
  {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex);
    left_msg = global_pos_cmd_left;
    right_msg = global_pos_cmd_right;
  }
}

class ArmController {
  public:
    ArmController(const std::string &urdf_path, const std::string &effector_name) 
      : robot(urdf_path), solver(robot) {
      solver.dt = 0.001;
      solver.add_kinetic_energy_regularization_task(1e-6);
      solver.mask_fbase(true); // no floating_base
      solver.enable_velocity_limits(true);
      solver.enable_joint_limits(false);

      Eigen::Affine3d effector_pose = Eigen::Affine3d::Identity();
      effector_task = solver.add_frame_task(effector_name, effector_pose);
      effector_task.configure(effector_name, "soft", 1.0);
      auto manipulability = solver.add_manipulability_task(effector_name, "both", 1.0);
      manipulability.configure("manipulability", "soft", 5e-2);

      std::cout << "joint names: " << std::endl;
      for (auto name : robot.joint_names()) {
        std::cout << "  " << name << std::endl;
      }
    }

    void set_target_pose(const Eigen::Affine3d &target_pose) {
      effector_task.set_T_world_frame(target_pose);
      solver.solve(true);
      robot.update_kinematics();
    }

    void get_joint_positions(Eigen::VectorXd &q) {
      q = robot.state.q;
      q = q.tail(q.size() - 7); // ignore base in top7
    }

  private:
    placo::model::RobotWrapper robot;
    placo::kinematics::KinematicsSolver solver;
    placo::kinematics::FrameTask effector_task;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "picoxr_arx_main_v2");
    ros::NodeHandle n;
    auto xrRobot = std::make_shared<XrRobot>();

    // Subscriber from XR
    ros::Subscriber xr_sub = n.subscribe("/xr_pose", 10, &XrRobot::xrPoseCallback, xrRobot.get());

    // Publisher to ARX
    ros::Publisher left_pub = n.advertise<arm_control::JointControl>("/joint_control", 10);
    ros::Publisher right_pub = n.advertise<arm_control::JointControl>("/joint_control2", 10);
    ros::Rate loop_rate(10); // 10Hz

    std::string left_arm_urdf = "src/arx_x7s/urdf/X7Sleft1.urdf";
    std::string right_arm_urdf = "src/arx_x7s/urdf/x7sRIGHT.urdf";
    std::string effector_name = "joint8";

    auto left_arm = ArmController(left_arm_urdf, effector_name);
    auto right_arm = ArmController(right_arm_urdf, effector_name);

    while (ros::ok())
    {
      bool active;
      pos_cmd_msg::PosCmd left_msg, right_msg;
      arm_control::JointControl left_joint_msg, right_joint_msg;

      xrRobot->getPosCmd(left_msg, right_msg, active);
      if (active) {
        Eigen::Affine3d left_pose = Eigen::Affine3d::Identity();
        left_pose.translation() = Eigen::Vector3d(left_msg.x, left_msg.y, left_msg.z);
        left_pose.rotate(Eigen::AngleAxisd(left_msg.roll, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(left_msg.pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(left_msg.yaw, Eigen::Vector3d::UnitZ()));
        left_arm.set_target_pose(left_pose);
        Eigen::VectorXd left_joint_positions;
        left_arm.get_joint_positions(left_joint_positions);

        Eigen::Affine3d right_pose = Eigen::Affine3d::Identity();
        right_pose.translation() = Eigen::Vector3d(right_msg.x, right_msg.y, right_msg.z);
        right_pose.rotate(Eigen::AngleAxisd(right_msg.roll, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(right_msg.pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(right_msg.yaw, Eigen::Vector3d::UnitZ()));
        right_arm.set_target_pose(right_pose);
        Eigen::VectorXd right_joint_positions;
        right_arm.get_joint_positions(right_joint_positions);

        std::cout << "left_joint_positions: " << left_joint_positions.transpose() << std::endl;
        std::cout << "right_joint_positions: " << right_joint_positions.transpose() << std::endl;

        left_joint_msg.joint_pos[0] = -0.2;
        left_joint_msg.joint_pos[1] = -0.2;
        left_joint_msg.joint_pos[2] = -0.2;
        left_joint_msg.joint_pos[3] = -0.2;
        left_joint_msg.joint_pos[4] = -0.2;
        left_joint_msg.joint_pos[5] = -0.2;
        left_joint_msg.joint_pos[6] = -0.2;
        left_joint_msg.joint_pos[7] = 2.0;    // gripper

        for (int i = 0; i < 7; i++) {
          right_joint_msg.joint_pos[i] = right_joint_positions[i+2];
        }
        right_joint_msg.joint_pos[7] = 2.0;    // gripper
        right_joint_msg.mode = 0;
      }
      left_pub.publish(left_joint_msg);
      right_pub.publish(right_joint_msg);
      ROS_INFO("Published left&right joint control message");

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
