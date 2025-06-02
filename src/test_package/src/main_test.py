#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from test_package.msg import ManipulatorPose  # 메시지 이름에 따라 경로 조정

import time

class MainProcessNode(Node):
    def __init__(self):
        super().__init__('main_process_node')
        self.publisher_ = self.create_publisher(ManipulatorPose, '/manipulator_pose', 10)

        # 타이머 설정: 1초마다 메시지 발행
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Main Process Node started. Publishing to /manipulator_pose...")

        # 테스트용 버튼 타입 리스트
        self.test_buttons = ['button_up', 'button_down', 'button5', 'none']
        self.index = 0

    def timer_callback(self):
        msg = ManipulatorPose()
        msg.button_type = self.test_buttons[self.index]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.button_type}")

        # self.index = (self.index + 1) % len(self.test_buttons)  # 순환

def main(args=None):
    rclpy.init(args=args)
    node = MainProcessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

