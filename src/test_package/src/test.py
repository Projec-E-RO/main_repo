#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose_demo')

        # IK 서비스 / Trajectory 액션 클라이언트
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_cli.wait_for_service(1.0):
            self.get_logger().info('waiting for /compute_ik ...')
        self.traj_cli = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # TF2 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 카메라로 인식한 좌표 (예시: x_end, y_end, z_end = 0.1, 0.1, 0.1)
        self.camera_pose = (0.1, 0.1, 0.1)

        # init 마지막에 move(카메라 좌표 + TF 변환) 실행
        self.get_logger().info('테스트 시작: 카메라 좌표 + TF 변환 후 이동')
        self.move_with_tf()

    def move_with_tf(self):
        # 카메라 좌표 (x_end, y_end, z_end)
        x_end, y_end, z_end = self.camera_pose

        # TF 변환: end_effector_link → link1 (예시)
        # 실제로는 아래처럼 lookup_transform으로 가져와야 함
        # (여기서는 예시로 dx, dy, dz를 임의로 설정)
        from_frame = 'link1'
        to_frame = 'end_effector_link'
        try:
            now = rclpy.time.Time()
            self.get_logger().info('1')
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                to_frame, from_frame, now)
            self.get_logger().info('2')
            tf_x = transform.transform.translation.x
            tf_y = transform.transform.translation.y
            tf_z = transform.transform.translation.z

            # 카메라 좌표 + TF 오프셋 (실제 로봇 기준 좌표)
            x_final = x_end + tf_x
            y_final = y_end + tf_y
            z_final = z_end + tf_z

            self.get_logger().info(
                f'카메라 좌표: x={x_end:.3f}, y={y_end:.3f}, z={z_end:.3f}\n'
                f'TF 오프셋: dx={tf_x:.3f}, dy={tf_y:.3f}, dz={tf_z:.3f}\n'
                f'최종 좌표: x={x_final:.3f}, y={y_final:.3f}, z={z_final:.3f}'
            )

            # Pose 생성
            target_pose = Pose()
            target_pose.position.x = x_final
            target_pose.position.y = y_final
            target_pose.position.z = z_final
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 1.0

            self.move(target_pose)

        except Exception as e:
            self.get_logger().error(f'TF 변환 실패: {str(e)}')

    def move(self, target_pose):
        self.get_logger().info(f'move 함수 시작: 목표 좌표 x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'world'  # 실제 로봇 기준 프레임에 맞게 수정
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.timeout = Duration(sec=2)

        future = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().error_code.val != 1:
            self.get_logger().error('IK failed')
            return

        js_names = future.result().solution.joint_state.name
        js_pos = future.result().solution.joint_state.position
        joint_map = dict(zip(js_names, js_pos))
        self.get_logger().info(f'IK OK: {joint_map}')

        arm_jnts = ['joint1', 'joint2', 'joint3', 'joint4']
        pos_arm = [joint_map[j] for j in arm_jnts]

        traj = JointTrajectory()
        traj.joint_names = arm_jnts
        pt = JointTrajectoryPoint()
        pt.positions = pos_arm
        pt.time_from_start = Duration(sec=2)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal(trajectory=traj)
        self.traj_cli.wait_for_server()
        gh_future = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_future)
        gh = gh_future.result()
        if not gh.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        err = res_future.result().result.error_code
        self.get_logger().info(f'Execution result (0=SUCCEEDED): {err}')
        self.get_logger().info(f'최종 이동 완료: 목표 좌표 x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')

def main():
    rclpy.init()
    node = GoToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
