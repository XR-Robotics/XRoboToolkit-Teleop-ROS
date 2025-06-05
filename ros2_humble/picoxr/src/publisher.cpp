#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <vector>
#include <sstream>
#include <string>
#include "xr_msgs/msg/custom.hpp"
#include "xr_msgs/msg/head.hpp"
#include "xr_msgs/msg/controller.hpp"

#include "PXREARobotSDK.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;

std::mutex g_callback_mutex;

void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData) {
  std::lock_guard<std::mutex> lock(g_callback_mutex);
  if (g_callback) {
    g_callback(context, type, status, userData);
  }
}

void print_json(const json& j, int indent=1) {
    // 根据缩进级别设置空格
    std::string indent_str(indent * 2, ' ');
    
    if (j.is_object()) {
        std::cout << indent_str << "{\n";
        for (const auto& [key, value] : j.items()) {
            std::cout << indent_str << "  \"" << key << "\": ";
            print_json(value, indent + 1);
            std::cout << ",\n";
        }
        // 删除最后一个多余的逗号
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "}";
    } else if (j.is_array()) {
        std::cout << indent_str << "[\n";
        for (const auto& item : j) {
            std::cout << indent_str << "  ";
            print_json(item, indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "]";
    } else if (j.is_string()) {
        std::cout << "\"" << j.get<std::string>() << "\"";
    } else if (j.is_boolean()) {
        std::cout << (j.get<bool>() ? "true" : "false");
    } else if (j.is_number_integer()) {
        std::cout << j.get<int64_t>();
    } else if (j.is_number_unsigned()) {
        std::cout << j.get<uint64_t>();
    } else if (j.is_number_float()) {
        std::cout << j.get<double>();
    } else if (j.is_null()) {
        std::cout << "null";
    } else {
        std::cout << "unknown type";
    }
}


std::vector<float> stringToFloatVector(const std::string& input) {
    std::vector<float> result;
    std::stringstream ss(input);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            result.push_back(std::stof(token));
        } catch (const std::exception& e) {
            std::cerr << "转换错误: " << token << " -> " << e.what() << std::endl;
        }
    }
    return result;
}


class XRNode : public rclcpp::Node
{
public:
  XRNode() : Node("xr_publisher")
  {
    publisher_ = this->create_publisher<xr_msgs::msg::Custom>("xr_pose", 10);
  }

  void OnPXREAClientCallback(void* context, PXREAClientCallbackType type,int status,void* userData)
  {
    (void)context;
    switch (type)
    {
      case PXREAServerConnect:
          std::cout <<"server connect"  << std::endl;
          break;
      case PXREAServerDisconnect:
          std::cout  <<"server disconnect"  << std::endl;
          break;
      case PXREADeviceFind:
          std::cout << "device find"<< (const char*)userData << std::endl;
          break;
      case PXREADeviceMissing:
          std::cout <<"device missing"<<(const char*)userData<<  std::endl;
          break;
      case PXREADeviceConnect:
          std::cout <<"device connect"<<(const char*)userData<<status<< std::endl;
          break;
      case PXREADeviceStateJson: {
          auto& dsj = *((PXREADevStateJson*)userData);
          // std::cout <<"device data"<< dsj.stateJson << std::endl;
          try {
            auto json_obj = json::parse(dsj.stateJson);
				  	// print_json(json_obj);
            auto value_str = json_obj["value"].get<std::string>();
            auto value_obj = json::parse(value_str);

            auto custom_msg = xr_msgs::msg::Custom();
            custom_msg.timestamp_ns = value_obj["timeStampNs"].get<uint64_t>();
            custom_msg.input = value_obj["Input"].get<int>();

            // head
            auto head_msg = xr_msgs::msg::Head();
            if (value_obj.contains("Head")) {
              auto head_j = value_obj["Head"];
              std::vector<float> head_pose = stringToFloatVector(head_j["pose"].get<std::string>());
              if (head_pose.size() != 7) {
                std::cerr << "Parse failed: head pose data length != 7" << std::endl;
              }
              std::copy(head_pose.begin(), head_pose.end(), head_msg.pose.data());
              head_msg.status = head_j["status"].get<int>();
            } else {
              head_msg.status = -1;
            }
            custom_msg.head = head_msg;

            // controller
            if (value_obj.contains("Controller")) {
              for (auto& element : value_obj["Controller"].items()) {
                auto controller_msg = xr_msgs::msg::Controller();
                auto ctrl_j = element.value();

                controller_msg.axis_x = ctrl_j["axisX"].get<float>();
                controller_msg.axis_y = ctrl_j["axisY"].get<float>();
                controller_msg.axis_click = ctrl_j["axisClick"].get<bool>();
                controller_msg.gripper = ctrl_j["grip"].get<float>();
                controller_msg.trigger = ctrl_j["trigger"].get<float>();
                controller_msg.primary_button = ctrl_j["primaryButton"].get<bool>();
                controller_msg.secondary_button = ctrl_j["secondaryButton"].get<bool>();
                controller_msg.menu_button = ctrl_j["menuButton"].get<bool>();
                std::vector<float> ctrl_pose = stringToFloatVector(ctrl_j["pose"].get<std::string>());
                if (ctrl_pose.size() != 7) {
                  std::cerr << "Parse failed: ctrl pose data length != 7" << std::endl;
                }
                std::copy(ctrl_pose.begin(), ctrl_pose.end(), controller_msg.pose.data());
                controller_msg.status = 3;

                if (element.key() == "left") {
                  custom_msg.left_controller = controller_msg;
                } else {
                  custom_msg.right_controller = controller_msg;
                }
              }
            } else {
              auto left_controller_msg = xr_msgs::msg::Controller();
              auto right_controller_msg = xr_msgs::msg::Controller();
              left_controller_msg.status = -1;
              right_controller_msg.status = -1;
              custom_msg.left_controller = left_controller_msg;
              custom_msg.right_controller = right_controller_msg;
            }

            // hand

            // body

            publisher_->publish(custom_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing timestamp_ns '%ld'", custom_msg.timestamp_ns);

          } catch (const std::exception& e) {
            std::cerr << "Parse failed: " << e.what() << std::endl;
          }
          break;
      }
      case PXREADeviceCustomMessage:
          std::cout << "device custom message" << std::endl;
          break;
      case PXREAFullMask:
          std::cout << "full mask" << std::endl;
          break;
    }
  }

private:
  rclcpp::Publisher<xr_msgs::msg::Custom>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto xrNode = std::make_shared<XRNode>();
  g_callback = [&xrNode] (void* context, PXREAClientCallbackType type,int status,void* userData) {
    xrNode->OnPXREAClientCallback(context, type, status, userData);
  };
  PXREAInit(NULL, callbackForwarder, PXREAFullMask);

  rclcpp::spin(xrNode);

  PXREADeinit();

  rclcpp::shutdown();
  return 0;
}
