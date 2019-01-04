/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <geometry_msgs/msg/twist.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy, const std::string& which_map);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int> axis_angular_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  bool sent_disable_msg;

  void printParameters()
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", enable_button);
    ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", enable_turbo_button);

    for (std::map<std::string, int>::iterator it = axis_linear_map.begin();
         it != axis_linear_map.end(); ++it)
    {
      ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
        it->first.c_str(), it->second, scale_linear_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), scale_linear_map["turbo"][it->first]);
    }

    for (std::map<std::string, int>::iterator it = axis_angular_map.begin();
         it != axis_angular_map.end(); ++it)
    {
      ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
        it->first.c_str(), it->second, scale_angular_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), scale_angular_map["turbo"][it->first]);
    }
  }
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy() : Node("teleop_twist_joy_node")
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
    rmw_qos_profile_default);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy",
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1),
    rmw_qos_profile_default);

  // TODO(clalancette): node->get_parameter(s) doesn't support getting a map of
  // values yet.  https://github.com/ros2/rclcpp/issues/529
  int_parameters.push_back({"enable_button", 5, pimpl_->enable_button});
  int_parameters.push_back({"enable_turbo_button", -1, pimpl_->enable_turbo_button});
  int_parameters.push_back({"axis_linear_x", 5, pimpl_->axis_linear_map["x"]});
  int_parameters.push_back({"axis_angular_yaw", 2, pimpl_->axis_angular_map["yaw"]});

  double_parameters.push_back({"scale_linear_x", 0.5, pimpl_->scale_linear_map["normal"]["x"]});
  double_parameters.push_back({"scale_linear_turbo_x", 1.0, pimpl_->scale_linear_map["turbo"]["x"]});
  double_parameters.push_back({"scale_angular_yaw", 0.5, pimpl_->scale_angular_map["normal"]["yaw"]});
  double_parameters.push_back({"scale_angular_turbo_yaw", 0.5, pimpl_->scale_angular_map["turbo"]["yaw"]});

  for(const auto & p : int_parameters) {
    this->set_parameter_if_not_set(p.name, p.default_value);
    this->get_parameter(p.name, p.variable);
  }

  for(const auto & p : double_parameters) {
    this->set_parameter_if_not_set(p.name, p.default_value);
    this->get_parameter(p.name, p.variable);
  }

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        try {
          std::string name = param.get_name();
          auto int_it = std::find_if(
            int_parameters.begin(), int_parameters.end(),
            [&name](const JoyParameter<int> & j) -> bool {
              return name == j.name;
            });
          if (int_it != int_parameters.end()) {
            int_it->variable = param.as_int();
          } else {
            auto double_it = std::find_if(
              double_parameters.begin(), double_parameters.end(),
              [&name](const JoyParameter<double> & j) -> bool {
                return name == j.name;
              });
            if (double_it != double_parameters.end()) {
              double_it->variable = param.as_double();
            }
          }
        } catch (const std::runtime_error & err) {
          result.successful = false;
          result.reason = err.what();
          break;
        }
      }
      if (result.successful) {
        this->pimpl_->printParameters();
      }
      return result;
    };
  this->register_param_change_callback(param_change_callback);

  pimpl_->printParameters();

  pimpl_->sent_disable_msg = false;
}

TeleopTwistJoy::~TeleopTwistJoy()
{
  delete pimpl_;
}

double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= static_cast<size_t>(axis_map.at(fieldname))) {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default
  auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

  cmd_vel_msg->linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  cmd_vel_msg->linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  cmd_vel_msg->angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
  cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

  cmd_vel_pub->publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

  if (enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > static_cast<size_t>(enable_turbo_button) &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > static_cast<size_t>(enable_button) &&
           joy_msg->buttons[enable_button])
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default
      auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();
      cmd_vel_pub->publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
