#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from auto_aim_interfaces.msg import SerialPacket


class SerialPacketSubscriber(Node):
    def __init__(self):
        super().__init__('serial_packet_subscriber')
        self.subscription = self.create_subscription(
            SerialPacket,
            '/serial_packet',
            self.listener_callback,
            10  # QoS history depth
        )

    
        self.nav = BasicNavigator()     # 节点
        self.get_logger().info("Listening for SerialPacket messages on /serial_packet...")

    def listener_callback(self, msg: SerialPacket):
        self.get_logger().info(
            f"""
            Received SerialPacket:
            --------------------------------
            header: {hex(msg.header)}
            detect_color: {msg.detect_color}  # {'red' if msg.detect_color == 0 else 'blue'}
            task_mode: {msg.task_mode}        # {self.task_mode_to_str(msg.task_mode)}
            reset_tracker: {msg.reset_tracker}
            is_play: {msg.is_play}
            change_target: {msg.change_target}
            reserved: {msg.reserved}
            roll: {msg.roll:.2f}
            pitch: {msg.pitch:.2f}
            yaw: {msg.yaw:.2f}
            robot_hp: {msg.robot_hp}
            game_time: {msg.game_time} seconds
            checksum: {hex(msg.checksum)}
            --------------------------------
            """,
            throttle_duration_sec=0.1  # 控制日志输出频率
        )

        if msg.game_time == 1:
             self.set_goal_pose(1.2, 1.2)
             self.set_goal_pose(4.5, 1.2)
             self.set_goal_pose(4.5, 0.1)
        #    self.set_goal_pose(1.0, 0.0)

    

    def set_goal_pose(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        self.nav.cancelTask()
        

    def set_goal_pose_home(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        self.nav.cancelTask()
        # result = self.nav.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Goal succeeded!')
        # elif result == TaskResult.CANCELED:
        #     print('Goal was canceled!')
        # elif result == TaskResult.FAILED:
        #     print('Goal failed!')

    def task_mode_to_str(self, mode):
        if mode == 0:
            return "手打"
        elif mode == 1:
            return "自瞄"
        elif mode == 2:
            return "大符"


def main(args=None):
    rclpy.init(args=args)
    serial_packet_sub = SerialPacketSubscriber()

    # serial_packet_sub.nav.waitUntilNav2Active()   #   `等待导航可用`
    rclpy.spin(serial_packet_sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()