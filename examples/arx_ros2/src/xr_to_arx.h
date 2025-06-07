/*
 * Convert xr pose to arx pose.
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>
#include <algorithm>

#include "xr_msgs/msg/custom.hpp"       // xr pose
#include "arm_control/msg/pos_cmd.hpp"  // arx pose

using xr_msgs::msg::Custom;
using arm_control::msg::PosCmd;

struct RawPose {
  float x, y, z, rx, ry, rz, rw;
  RawPose() : x(0), y(0), z(0), rx(0), ry(0), rz(0), rw(1) {}
  RawPose(const std::array<float, 7>& pose) {
    x = pose[0]; y = pose[1]; z = pose[2];
    rx = pose[3]; ry = pose[4]; rz = pose[5]; rw = pose[6];
  }
  void GetRPY(double& roll, double& pitch, double& yaw) {
    tf2::Quaternion q(rx, ry, rz, rw);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
  }
  void Reset() { x = y = z = rx = ry = rz = 0.0; rw = 1.0; }
};

void computeRelativePose(const RawPose& origin, const RawPose& current, RawPose& relative);

void XR2ARX(const RawPose& vr_pose, RawPose& arx_pose, float scale = 1.0);

class Trigger {
  int clicked_count = 0, clicked_interval = 50;
  bool clicked_valid = false, prev_status = false, cur_status = false;
public:
  bool isActive(float left_trigger, float right_trigger);
};

/**
 * Xr2Arx:
 *   xrPoseCallback -> xr_pose -> convert to arx_pose -> getPosCmd
 *
 * Actual subscription and publisher is defined in Node class, where Xr2Arx
 * is initialized.
 */
class Xr2Arx {
  std::mutex pos_cmd_mutex;
  arm_control::msg::PosCmd global_pos_cmd_left, global_pos_cmd_right;
  std::atomic<bool> is_running{false};
  int frame_count = 0, print_interval = 100;
  std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>();
  RawPose left_pose_origin, right_pose_origin;
  float body_height = 0.0, height_speed = 0.05, body_x = 0.0, body_y = 0.0, body_rot = 0.0, body_speed = 0.5;
  inline bool shouldPrint() { return frame_count % print_interval == 0; }
public:
  void reset();
  void xrPoseCallback(const Custom::SharedPtr msg);
  void getPosCmd(PosCmd& left_msg, PosCmd& right_msg, bool& active);
};
