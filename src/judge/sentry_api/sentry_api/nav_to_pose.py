from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator()  #   节点
    nav.waitUntilNav2Active()   #   等待导航可用

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 4.2
    goal_pose.pose.position.y = -2.5
    goal_pose.pose.position.z = 0.0

    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # nav.get_logger().info(f"剩余距离：{feedback.distance_remaining}")

    result = nav.getResult()
    # nav.get_logger().info(f"导航结果：{result}")


