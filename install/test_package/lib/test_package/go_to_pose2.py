#!/usr/bin/env python3
"""
direct_joint_control.py ― 관절 각도를 라디안 단위로 직접 지정하여 제어
(ROS 2 Humble + OpenMANIPULATOR-X)
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import math


class DirectJointControl(Node):
    def __init__(self):
        super().__init__('direct_joint_control')

        # --- Trajectory 액션 클라이언트 -----------------------------------
        self.traj_cli = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        # --- 목표 관절 각도 (라디안 단위) ---------------------------------
        # 여기서 원하는 각도를 라디안 단위로 직접 설정하세요!
        self.target_joints = {
            'joint1': 0.000,    # 약 30도 (rad)
            'joint2': -1.203,   # 약 -45도 (rad)
            'joint3': 1.102,    # 약 60도 (rad)
            'joint4': 0.121        # 0도 (rad)
        }

        # 실행
        self.move_to_joints()

    # -----------------------------------------------------------------------
    def move_to_joints(self):
        """직접 지정한 관절 각도로 이동"""
        
        # 1) 관절 이름과 각도 리스트 생성 -----------------------------------
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4']
        target_positions = [self.target_joints[joint] for joint in arm_joints]
        
        self.get_logger().info('Target joint angles:')
        for joint, angle_rad in self.target_joints.items():
            angle_deg = math.degrees(angle_rad)
            self.get_logger().info(f'  {joint}: {angle_rad:.4f} rad ({angle_deg:.1f}°)')

        # 2) 궤적 메시지 생성 ----------------------------------------------
        traj = JointTrajectory()
        traj.joint_names = arm_joints
        
        # 궤적 포인트 생성
        pt = JointTrajectoryPoint()
        pt.positions = target_positions
        pt.time_from_start = Duration(sec=3)  # 3초 동안 이동
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal(trajectory=traj)

        # 3) 액션 전송 & 결과 대기 -----------------------------------------
        self.traj_cli.wait_for_server()
        self.get_logger().info('Sending joint trajectory goal...')
        
        gh_future = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_future)
        gh = gh_future.result()
        
        if not gh.accepted:
            self.get_logger().error('Trajectory goal rejected')
            rclpy.shutdown()
            return

        # 결과 대기
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        err = res_future.result().result.error_code
        self.get_logger().info(f'Execution result (0=SUCCEEDED): {err}')
        rclpy.shutdown()


def main():
    rclpy.init()
    DirectJointControl()


if __name__ == '__main__':
    main()
