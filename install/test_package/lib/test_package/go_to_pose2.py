#!/usr/bin/env python3
"""
go_to_pose.py ― 한 줄 실행으로
1) /compute_ik  → 역기구학
2) /arm_controller/follow_joint_trajectory → 궤적 실행
(ROS 2 Humble + MoveIt 2 + OpenMANIPULATOR-X)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient


class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose_demo')

        # --- IK 서비스 / Trajectory 액션 클라이언트 -------------------------
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_cli.wait_for_service(1.0):
            self.get_logger().info('waiting for /compute_ik ...')

        self.traj_cli = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        # --- 목표 Pose -------------------------------------------------------
        self.target_pose = Pose()
        self.target_pose.position.x = 0.2
        self.target_pose.position.y = 0.10
        self.target_pose.position.z = 0.15
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = 0.0
        self.target_pose.orientation.w = 1.0

        # 실행
        self.move()

    # -----------------------------------------------------------------------
    def move(self):
        # 1) IK 요청 ---------------------------------------------------------
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = self.target_pose
        req.ik_request.timeout = Duration(sec=2)

        future = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().error_code.val != 1:   # SUCCESS = 1
            self.get_logger().error('IK failed'); rclpy.shutdown(); return

        js_names = future.result().solution.joint_state.name
        js_pos   = future.result().solution.joint_state.position
        joint_map = dict(zip(js_names, js_pos))
        self.get_logger().info(f'IK OK: {joint_map}')

        # 2) 4개 관절만 궤적 작성 ------------------------------------------
        arm_jnts = ['joint1', 'joint2', 'joint3', 'joint4']
        pos_arm  = [joint_map[j] for j in arm_jnts]
        self.get_logger().info(f'1')

        traj = JointTrajectory()
        traj.joint_names = arm_jnts
        pt = JointTrajectoryPoint()
        pt.positions = pos_arm
        pt.time_from_start = Duration(sec=2)
        traj.points.append(pt)
        self.get_logger().info(f'2')

        goal = FollowJointTrajectory.Goal(trajectory=traj)

        # 3) 액션 전송 & 결과 대기 ------------------------------------------
        self.traj_cli.wait_for_server()
        self.get_logger().info(f'3')
        gh_future = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_future)
        gh = gh_future.result()
        self.get_logger().info(f'4')
        if not gh.accepted:
            self.get_logger().info(f'4-1')
            self.get_logger().error('Trajectory goal rejected'); rclpy.shutdown(); return

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        err = res_future.result().result.error_code
        self.get_logger().info(f'Execution result (0=SUCCEEDED): {err}')
        rclpy.shutdown()


def main():
    rclpy.init()
    GoToPose()


if __name__ == '__main__':
    main()

