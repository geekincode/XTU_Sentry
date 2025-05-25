// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include "auto_aim_interfaces/msg/serial_packet.hpp"

using namespace std::chrono_literals;

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);
  aim_time_info_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("/time_info/aim", 10);
  buff_time_info_pub_ =
    this->create_publisher<buff_interfaces::msg::TimeInfo>("/time_info/buff", 10);
  record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("/record_controller", 10);
  current_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/serial_driver/current_yaw", 10);

  // 串口数据包发布
  serial_packet_pub_ = this->create_publisher<auto_aim_interfaces::msg::SerialPacket>("serial_packet", 10);

  // Create Subscriber
  fired_info_sub_ = this->create_subscription<auto_aim_interfaces::msg::FiredInfo>("/fired_info",10,std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1));


  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client 
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  // Target change service cilent
  change_target_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/change");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // timer_ = this->create_wall_timer(8ms, std::bind(&RMSerialDriver::on_timer, this));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::cmd_vel_callback, this, std::placeholders::_1));

  // Create Subscription
  // aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1));

}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  //为data预留空间刚好存储一帧消息包
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      //调用serial_driver_对象port方法获取串口对象并且将接收到的数据（1字节）存储到header中
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        //接收到包头，调用serial_driver_对象port方法获取串口对象并将剩下的数据（sizeof(ReceivePacket) - 1）存储到data中
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);
        //将包头加入到data的开头
        data.insert(data.begin(), header[0]);

        //将data转换为ReceivePacket结构体
        ReceivePacket packet = fromVector(data);

        pubSerialPacket(packet);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // RCLCPP_INFO(get_logger(), "CRC ok!");
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }

          if (packet.change_target) {
            changeTarget();
          }

          std_msgs::msg::String task;
          std::string theory_task;
          // if (
          //   (packet.game_time >= 329 && packet.game_time <= 359) ||
          //   (packet.game_time >= 239 && packet.game_time <= 269)) {
          //   theory_task = "small_buff";
          // } else if (
          //   (packet.game_time >= 149 && packet.game_time <= 179) ||
          //   (packet.game_time >= 74 && packet.game_time <= 104) ||
          //   (packet.game_time > 0 && packet.game_time <= 29)) {
          //   theory_task = "large_buff";
          // } else {
          //   theory_task = "aim";
          // }
          theory_task = "aim";

          if (packet.task_mode == 0) {
            task.data = theory_task;
          } else if (packet.task_mode == 1) {
            task.data = "aim";
          } else if (packet.task_mode == 2) {
            if (theory_task == "aim") {
              task.data = "auto";
            } else {
              task.data = theory_task;
            }
          } else {
            task.data = "aim";
          }
          task_pub_->publish(task);

          RCLCPP_DEBUG(
            get_logger(), "Game time: %d, Task mode: %d, Theory task: %s", packet.game_time,
            packet.task_mode, theory_task.c_str());

          std_msgs::msg::String record_controller;
          record_controller.data = packet.is_play ? "start" : "stop";
          record_controller_pub_->publish(record_controller);

          //定义tf消息 t
          geometry_msgs::msg::TransformStamped t;
          std_msgs::msg::Float32 current_yaw;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
          //父坐标系为odom，子坐标系为gimbal_link
          t.header.frame_id = "map";
          t.child_frame_id = "gimbal_link";
          current_yaw.data = packet.yaw;
          current_yaw_pub_->publish(current_yaw);

          packet.roll = packet.roll * M_PI / 180.0;
          packet.pitch = packet.pitch * M_PI / 180.0;
          packet.yaw = packet.yaw * M_PI / 180.0;
          // printf("roll:%f pitch:%f yaw:%f\n", packet.roll, packet.pitch, packet.yaw);

          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw);
          // q.setRPY(packet.roll, packet.pitch, 0);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          // publish time
          auto_aim_interfaces::msg::TimeInfo aim_time_info;
          buff_interfaces::msg::TimeInfo buff_time_info;
          aim_time_info.header = t.header;
          aim_time_info.time = 0;
          buff_time_info.header = t.header;
          buff_time_info.time = 0;
          aim_time_info_pub_->publish(aim_time_info);
          buff_time_info_pub_->publish(buff_time_info);

          // RCLCPP_INFO(get_logger(), "Robot_HP: %d", packet.robot_HP);
          // RCLCPP_INFO(get_logger(), "gam_time: %d", packet.game_time);

        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }
}

void RMSerialDriver::pubSerialPacket(ReceivePacket & packet)
{
  auto msg = std::make_shared<auto_aim_interfaces::msg::SerialPacket>();

  msg->header = packet.header;
  msg->detect_color = packet.detect_color;
  msg->task_mode = packet.task_mode;
  msg->reset_tracker = packet.reset_tracker;
  msg->is_play = packet.is_play;
  msg->change_target = packet.change_target;
  msg->reserved = packet.reserved;
  msg->roll = packet.roll;
  msg->pitch = packet.pitch;
  msg->yaw = packet.yaw;
  msg->robot_hp = packet.robot_HP;
  msg->game_time = packet.game_time;
  msg->checksum = packet.checksum;

  RCLCPP_INFO(this->get_logger(), "发布packet消息");
  serial_packet_pub_->publish(*msg);
}
void RMSerialDriver::sendArmorData(const auto_aim_interfaces::msg::FiredInfo::ConstSharedPtr msg)
{

  try {

    /* 这里放火控部分 */
    packet.header = 0xA5;
    packet.state = (uint8_t)(msg->state);
    packet.fire_flag = (uint8_t)(msg->auto_fire_flag);
    packet.pitch = msg->aim_pitch;
    packet.yaw = msg->aim_yaw;
    // RCLCPP_INFO(this->get_logger(), "-----state: %d", msg->state);

    sendPacket(&packet);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendBuffData(
  const buff_interfaces::msg::Rune::ConstSharedPtr rune,
  const buff_interfaces::msg::TimeInfo::ConstSharedPtr time_info)
{
/*
  try {
    SendPacket packet;
    packet.state = rune->tracking ? 2 : 0;
    packet.id = rune->offset_id;
    packet.armors_num = rune->offset_id;
    packet.x = rune->position.x;
    packet.y = rune->position.y;
    packet.z = rune->position.z;
    packet.yaw = rune->theta;
    packet.vx = rune->a;
    packet.vy = rune->b;
    packet.vz = rune->w;
    packet.v_yaw = 0.0;
    packet.r1 = 0.0;
    packet.r2 = 0.0;
    packet.dz = 0.0;
    packet.cap_timestamp = time_info->time;
    if (rune->w == 0) {
      packet.t_offset = 0;
    } else {
      int T = abs(2 * 3.1415926 / rune->w * 1000);
      int offset = (rune->t_offset - time_info->time % T) % T;
      if (offset < 0) {
        packet.t_offset = T + offset;
      }
    }
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - rune->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
*/
}

void RMSerialDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  // RCLCPP_INFO(this->get_logger(),"发送/cmd_vel");
  
  packet.header = 0xA5;
  packet.nav_x = cmd_vel_msg->linear.x;
  packet.nav_y = cmd_vel_msg->linear.y;
  packet.nav_z = cmd_vel_msg->angular.z;

  sendPacket(&packet);
}

void RMSerialDriver::sendPacket(SendPacket *packet)
{
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(packet), sizeof(*packet));
    std::vector<uint8_t> data = toVector(*packet);
    serial_driver_->port()->send(data);
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or "
        "hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void RMSerialDriver::changeTarget()
{
  if (!change_target_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping target change");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  change_target_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Change target!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
