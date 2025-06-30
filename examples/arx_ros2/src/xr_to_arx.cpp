#include "xr_to_arx.h"

void computeRelativePose(const RawPose& origin, const RawPose& current, RawPose& relative) {
  // relative.x = current.x - origin.x;
  // relative.y = current.y - origin.y;
  // relative.z = current.z - origin.z;
  // tf2::Quaternion q_origin(origin.rx, origin.ry, origin.rz, origin.rw);
  // tf2::Quaternion q_current(current.rx, current.ry, current.rz, current.rw);
  // tf2::Quaternion q_relative = q_current * q_origin.inverse();
  // relative.rx = q_relative.x();
  // relative.ry = q_relative.y();
  // relative.rz = q_relative.z();
  // relative.rw = q_relative.w();

  // 旋转部分
  tf2::Quaternion q_origin(origin.rx, origin.ry, origin.rz, origin.rw);
  tf2::Quaternion q_current(current.rx, current.ry, current.rz, current.rw);
  tf2::Quaternion q_relative = q_current * q_origin.inverse();

  // 平移部分
  tf2::Vector3 t_origin(origin.x, origin.y, origin.z);
  tf2::Vector3 t_current(current.x, current.y, current.z);
  // 先将 current 的平移变换到 origin 坐标系下
  tf2::Vector3 t_relative = tf2::quatRotate(q_origin.inverse(), t_current - t_origin);

  relative.x = t_relative.x();
  relative.y = t_relative.y();
  relative.z = t_relative.z();
  relative.rx = q_relative.x();
  relative.ry = q_relative.y();
  relative.rz = q_relative.z();
  relative.rw = q_relative.w();
}

void convertXrToArx(const RawPose& vr_pose, RawPose& arx_pose, float scale = 1.0) {
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

bool Trigger::isActive(float left_trigger, float right_trigger) {
    clicked_valid = clicked_count > clicked_interval;
    if (left_trigger == 1.0 && right_trigger == 1.0) {
      clicked_count++; cur_status = true;
    } else {
      clicked_count = 0; cur_status = false;
    }
    bool active = prev_status && !cur_status;
    if (clicked_valid) {
      if (active) { clicked_count = 0; prev_status = false; cur_status = false; clicked_valid = false; }
      prev_status = cur_status;
      return active;
    } else {
      prev_status = cur_status;
      return false;
    }
}

void Xr2Arx::reset() {
    std::lock_guard<std::mutex> lock(pos_cmd_mutex);
    global_pos_cmd_left = arm_control::msg::PosCmd();
    global_pos_cmd_right = arm_control::msg::PosCmd();
    frame_count = 0; body_height = 0.0; body_x = 0.0; body_y = 0.0; body_rot = 0.0;
  }
  
void Xr2Arx::xrPoseCallback(const Custom::SharedPtr msg) {
    frame_count++;
    RawPose head_pose, left_pose, right_pose;
    convertXrToArx(RawPose(msg->head.pose), head_pose);
    convertXrToArx(RawPose(msg->left_controller.pose), left_pose);
    convertXrToArx(RawPose(msg->right_controller.pose), right_pose);
    if (shouldPrint()) {
      RCLCPP_INFO(rclcpp::get_logger("xr_robot"), "Received XR msg, timestamp: %ld", msg->timestamp_ns);
    }
    bool isTriggerActive = trigger->isActive(msg->left_controller.trigger, msg->right_controller.trigger);
    if (isTriggerActive) {
      if (!is_running.load()) {
        is_running.store(true);
        left_pose_origin = left_pose;
        right_pose_origin = right_pose;
      } else {
        is_running.store(false);
        left_pose_origin.Reset();
        right_pose_origin.Reset();
        reset();
      }
    }
    if (is_running.load()) {
      RawPose left_pose_relative, right_pose_relative;
      computeRelativePose(left_pose_origin, left_pose, left_pose_relative);
      computeRelativePose(right_pose_origin, right_pose, right_pose_relative);

      double left_roll, left_pitch, left_yaw;
      double right_roll, right_pitch, right_yaw;
      double head_roll, head_pitch, head_yaw;
      left_pose_relative.GetRPY(left_roll, left_pitch, left_yaw);
      right_pose_relative.GetRPY(right_roll, right_pitch, right_yaw);
      head_pose.GetRPY(head_roll, head_pitch, head_yaw);

      if (msg->left_controller.axis_y > 0.5) body_height += 1.0 * height_speed;
      else if (msg->left_controller.axis_y < -0.5) body_height += -1.0 * height_speed;
      body_height = std::max(0.0f, std::min(body_height, 18.0f));
      if (msg->right_controller.axis_x > 0.5) body_y += 1.0 * body_speed;
      else if (msg->right_controller.axis_x < -0.5) body_y += -1.0 * body_speed;
      if (msg->right_controller.axis_y > 0.5) body_x += 1.0 * body_speed;
      else if (msg->right_controller.axis_y < -0.5) body_x += -1.0 * body_speed;
      if (msg->right_controller.primary_button) body_rot += 1.0 * body_speed;
      else if (msg->right_controller.secondary_button) body_rot += -1.0 * body_speed;

      // if (shouldPrint()) {
      //   std::cout << "left_pose_ori(xyz): " << left_pose_origin.x << ", " << left_pose_origin.y << ", " << left_pose_origin.z << std::endl;
      //   std::cout << "  left_pose_cur(xyz): " << left_pose.x << ", " << left_pose.y << ", " << left_pose.z << std::endl;
      //   std::cout << "    left_pose_rel(xyz): " << left_pose_relative.x << ", " << left_pose_relative.y << ", " << left_pose_relative.z << std::endl;
      //   RCLCPP_INFO(rclcpp::get_logger("xr_robot"),
      //    "left_RPY: %f, %f, %f, right_RPY: %f, %f, %f", 
      //     left_roll, left_pitch, left_yaw, right_roll, right_pitch, right_yaw);
      // }

      // update pos_cmd
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
}

void Xr2Arx::getPosCmd(PosCmd& left_msg, PosCmd& right_msg, bool& active) {
    active = is_running.load();
    std::lock_guard<std::mutex> lock(pos_cmd_mutex);
    left_msg = global_pos_cmd_left;
    right_msg = global_pos_cmd_right;
}
