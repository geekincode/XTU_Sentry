#ifndef __FIRE_CONTROL_HPP__
#define __FIRE_CONTROL_HPP__

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/time_info.hpp"
#include "auto_aim_interfaces/msg/fired_info.hpp"
#include "buff_interfaces/msg/rune.hpp"
#include "buff_interfaces/msg/time_info.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>



namespace rm_fire_control
{

class FireController : public rclcpp::Node
{
public:
    FireController(const rclcpp::NodeOptions & options);
    void Trajectory_Solution(void);	
    void ParamInit(void);
    void ParamUpdate(void);
    void Choose_board(  
      const auto_aim_interfaces::msg::Target::ConstSharedPtr msg,
      const auto_aim_interfaces::msg::TimeInfo::ConstSharedPtr time_info);


private:

    float aim_point_x;
    float aim_point_y;
    float aim_point_z;
    float aim_point_yaw;

    double time_delay;
    double Fly_time;

    //最终输出
    double aim_pitch;
    double aim_yaw;
    bool auto_fire_flag;  //自动开火标志位

    double pre_aim_pitch;
    double pre_aim_yaw;

    double current_yaw;

    //参数
    double Gravity;
    double Z_Shifting;
    double Shoot_Speed;
    double Communicate_delay;
    bool Ignore_R_Diff;
    bool Ignore_Z_Diff;
    double Large_Armor_Width;
    double Small_Armor_Width;
    double Yaw_Res_Speed;



    //subscriber    
    message_filters::Subscriber<auto_aim_interfaces::msg::Target> aim_sub_;
    message_filters::Subscriber<auto_aim_interfaces::msg::TimeInfo> aim_time_info_sub_;
  
    typedef message_filters::sync_policies::ApproximateTime<
      auto_aim_interfaces::msg::Target, auto_aim_interfaces::msg::TimeInfo>
      aim_syncpolicy;
    typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;
    std::shared_ptr<AimSync> aim_sync_;
  
    message_filters::Subscriber<buff_interfaces::msg::Rune> rune_sub_;
    message_filters::Subscriber<buff_interfaces::msg::TimeInfo> buff_time_info_sub_;
  
    typedef message_filters::sync_policies::ApproximateTime<
      buff_interfaces::msg::Rune, buff_interfaces::msg::TimeInfo>
      buff_syncpolicy;
    typedef message_filters::Synchronizer<buff_syncpolicy> BuffSync;
    std::shared_ptr<BuffSync> buff_sync_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_yaw_sub_;
    void Current_yaw_callback(const std_msgs::msg::Float32::SharedPtr msg);

    //publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::FiredInfo>::SharedPtr fired_info_pub_;
    
    // For debug usage
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

    // Aimimg point receiving from serial port for visualization
    visualization_msgs::msg::Marker aiming_point_;
};



}   // namespace rm_fire_control


#endif