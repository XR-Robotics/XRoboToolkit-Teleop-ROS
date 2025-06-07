#include "UrdfVizClient.h"

namespace URDF_VIZ {

Robot::Robot(const std::string& base_url) : base_url_(base_url) {
  cli_ = std::make_shared<httplib::Client>(base_url_.c_str());
  joint_names_ = get_joint_names();

  std::cout << "urdf-viz joint_names: ";
  for (const auto& name : joint_names_) {
    std::cout << name << " ";
  }
  std::cout << std::endl;
}

std::vector<std::string> Robot::get_joint_names() {
  std::lock_guard<std::mutex> lock(cli_mutex_);
  auto res = cli_->Get("/get_joint_positions");
  if (res && res->status == 200) {
    auto j = nlohmann::json::parse(res->body);
    return j["names"].get<std::vector<std::string>>();
  }
  return {};
}

nlohmann::json Robot::get_joint_positions() {
  std::lock_guard<std::mutex> lock(cli_mutex_);
  auto res = cli_->Get("/get_joint_positions");
  if (res && res->status == 200) {
    return nlohmann::json::parse(res->body);
  }
  return {};
}

nlohmann::json Robot::set_joint_positions(const std::vector<std::string>& names, const std::vector<double>& positions) {
  nlohmann::json j;
  j["names"] = names;
  j["positions"] = positions;
  std::lock_guard<std::mutex> lock(cli_mutex_);
  auto res = cli_->Post("/set_joint_positions", j.dump(), "application/json");
  if (res && res->status == 200) {
    return nlohmann::json::parse(res->body);
  }
  return {};
}

nlohmann::json Robot::set_robot_origin(const std::vector<double>& position, const std::vector<double>& quaternion) {
  nlohmann::json j;
  j["position"] = position;
  j["quaternion"] = quaternion;
  std::lock_guard<std::mutex> lock(cli_mutex_);
  httplib::Headers headers = {{"Content-Type", "application/json"}};
  auto res = cli_->Post("/set_robot_origin", j.dump(), "application/json");
  if (res && res->status == 200) {
    return nlohmann::json::parse(res->body);
  }
  return {};
}

nlohmann::json Robot::get_robot_origin() {
  std::lock_guard<std::mutex> lock(cli_mutex_);
  auto res = cli_->Get("/get_robot_origin");
  if (res && res->status == 200) {
    return nlohmann::json::parse(res->body);
  }
  return {};
}

std::string Robot::get_urdf_text() {
  std::lock_guard<std::mutex> lock(cli_mutex_);
  auto res = cli_->Get("/get_urdf_text");
  if (res && res->status == 200) {
    return res->body;
  }
  return {};
}

void Robot::reset() {
  std::vector<double> positions(joint_names_.size(), 0.0);
  set_joint_positions(joint_names_, positions);
}

}  // namespace URDF_VIZ
