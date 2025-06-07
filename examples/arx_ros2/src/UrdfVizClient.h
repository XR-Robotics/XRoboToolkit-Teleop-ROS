#pragma once
#include "httplib.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>

namespace URDF_VIZ {

class Robot {
 public:
  Robot(const std::string& base_url);

  std::vector<std::string> get_joint_names();

  nlohmann::json get_joint_positions();

  nlohmann::json set_joint_positions(const std::vector<std::string>& names, const std::vector<double>& positions);

  nlohmann::json set_robot_origin(const std::vector<double>& position, const std::vector<double>& quaternion);

  nlohmann::json get_robot_origin();

  std::string get_urdf_text();

  void reset();

  inline const std::vector<std::string>& joint_names() const { return joint_names_; }

 private:
  std::string base_url_;
  std::vector<std::string> joint_names_;
  std::shared_ptr<httplib::Client> cli_;
  std::mutex cli_mutex_;
};

}  // namespace URDF_VIZ
