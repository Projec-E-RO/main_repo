#!/usr/bin/env python3
"""
direct_joint_control.py ― 관절 각도를 라디안 단위로 직접 지정하여 제어 + 그리퍼 open
(ROS 2 Humble + OpenMANIPULATOR-X)
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math


class DirectJointControlWithGripper(Node):
    def __init__(self):
        super().__init__('direct_joint_control_with_gripper')

        # --- Trajectory 액션 클라이언트 -----------------------------------
        self.traj_cli = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        # --- 그리퍼 액션 클라이언트 -------------------------------------
        self.gripper_cli = ActionClient(
            self, GripperCommand,
            '/gripper_controller/gripper_cmd')

        # --- 목표 관절 각도 (라디안 단위) ---------------------------------
        # 여기서 원하는 각도를 라디안 단위로 직접 설정하세요!
        self.target_joints = {
            'joint1': -2.976,    # 약 30도 (rad)
            'joint2': -0.245,   # 약 -45도 (rad)
            'joint3': 0.07,    # 약 60도 (rad)
            'joint4': 0.296        # 0도 (rad)
        }

        # --- 그리퍼 상수 -----------------------------------------------
        self.OPEN_POS = 0.03
        self.MAX_EFFORT = 0.5

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

        # 4) 팔 이동 완료 후 그리퍼 open 동작 실행 -------------------------
        if err == 0:  # 성공적으로 완료된 경우에만 그리퍼 동작
            self._control_gripper(self.OPEN_POS, 'open')
        
        # 그리퍼 동작 완료 후 노드 종료
        rclpy.shutdown()

    def _control_gripper(self, position: float, tag: str):
        """그리퍼 제어 함수"""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.MAX_EFFORT

        self.get_logger().info(f'{tag.capitalize()} … (pos={position})')

        # 그리퍼 서버 대기
        self.gripper_cli.wait_for_server()

        # 1) goal 전송
        send_future = self.gripper_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'{tag.capitalize()} goal rejected')
            return

        # 2) 결과 대기
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        wrapped = res_future.result()
        status = wrapped.status
        reached = wrapped.result.reached_goal
        stalled = wrapped.result.stalled

        if status == GoalStatus.STATUS_SUCCEEDED and reached:
            self.get_logger().info(f'{tag.capitalize()} done')
        else:
            self.get_logger().error(
                f'{tag.capitalize()} failed (status={status}, stalled={stalled})')


def main():
    rclpy.init()
    DirectJointControlWithGripper()


if __name__ == '__main__':
    main()
