// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_serial_driver/protocol/sentry_protocol.hpp"
// ros2
#include <geometry_msgs/msg/twist.hpp>

namespace fyt::serial_driver::protocol {
ProtocolSentry::ProtocolSentry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<23>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) {
  packet.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  packet.loadData<float>(static_cast<float>(data.pitch), 2);
  packet.loadData<float>(static_cast<float>(data.yaw), 6);
  packet.loadData<float>(static_cast<float>(data.distance), 10);
  packet_tool_->sendPacket(packet);
}

void ProtocolSentry::send(const rm_interfaces::msg::ChassisCmd &data) {
  packet.loadData<float>(static_cast<float>(data.twist.linear.x), 14);
  packet.loadData<float>(static_cast<float>(data.twist.linear.y), 18);
  packet_tool_->sendPacket(packet);
}

bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<23> packet;
  if (packet_tool_->recvPacket(packet)) {
    packet.unloadData(data.mode, 1);
    packet.unloadData(data.pitch, 2);
    packet.unloadData(data.roll, 6);
    packet.unloadData(data.yaw, 10);
    packet.unloadData(data.judge_system_data.game_type, 14);
    packet.unloadData(data.judge_system_data.game_progress, 15);
    packet.unloadData(data.judge_system_data.stage_remain_time, 16);
    packet.unloadData(data.judge_system_data.friendly_supply_non_zone_exchange, 18);
    packet.unloadData(data.judge_system_data.center_gain_point, 19);
    packet.unloadData(data.judge_system_data.current_hp, 21);
    return true;
  } else {
    return false;
  }
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  auto sub3 = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
    "/cmd_vel",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::ChassisCmd::SharedPtr msg) { this->send(*msg); });
  return {sub1, sub2,sub3};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolSentry::getClients(
  rclcpp::Node::SharedPtr node) const {
  return {node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                           rmw_qos_profile_services_default)};
}
}  // namespace fyt::serial_driver::protocol
