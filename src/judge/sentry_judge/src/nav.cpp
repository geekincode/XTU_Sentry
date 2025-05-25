#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class Nav2Navigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Navigator() : Node("nav2_navigator_node")
  {
    // 创建动作客户端
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");
  }

  void goToPose(const geometry_msgs::msg::PoseStamped& goal_pose)
  {
    // 等待动作服务器可用
    if (!action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    // 创建目标消息
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    // 设置发送选项
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server");
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Remaining distance: %.2f meters",
                   feedback->distance_remaining);
      };

    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
        }
      };

    // 发送目标
    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  bool isTaskComplete()
  {
    return task_complete_;
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  bool task_complete_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Nav2Navigator>();
  
  // 等待导航系统激活（示例等待时间）
  rclcpp::sleep_for(5s);

  // 创建目标位姿
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = node->now();
  goal_pose.pose.position.x = 4.2;
  goal_pose.pose.position.y = -2.5;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.0;
  goal_pose.pose.orientation.z = 0.0;
  goal_pose.pose.orientation.w = 1.0;

  // 发送导航目标
  node->goToPose(goal_pose);

  // 循环检查任务状态
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok() && !node->isTaskComplete()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // 获取最终结果
  auto result = node->getResult();
  RCLCPP_INFO(node->get_logger(), "Navigation completed");

  rclcpp::shutdown();
  return 0;
}