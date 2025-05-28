#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from test_package.msg import RobotPose

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose_demo')
        
        # 1. 콜백 그룹 생성
        self.sub_cb_group = MutuallyExclusiveCallbackGroup() # 서로 다른 콜백이 동시에 실행되지 않도록
        self.service_cb_group = MutuallyExclusiveCallbackGroup() 
        
        # 2. 서비스 클라이언트 초기화 (별도 콜백 그룹 할당)
        self.ik_cli = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.service_cb_group
        )
        while not self.ik_cli.wait_for_service(1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        
        # 3. 액션 클라이언트 초기화
        self.traj_cli = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 4. 구독자 생성 (별도 콜백 그룹 할당)
        self.subscription = self.create_subscription(
            RobotPose, 'object_position_with_tf', 
            self.pose_callback, 10,
            callback_group=self.sub_cb_group
        )
        
        self.get_logger().info('노드 초기화 완료')

    def pose_callback(self, msg):
        self.get_logger().info(f'수신 좌표: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')
        target_pose = Pose()
        target_pose.position.x = msg.x
        target_pose.position.y = msg.y
        target_pose.position.z = msg.z
        target_pose.orientation.w = 1.0
        
        # 비동기 서비스 호출
        future = self.ik_cli.call_async(
            self._create_ik_request(target_pose)
        )
        future.add_done_callback(
            lambda fut: self._handle_ik_response(fut, target_pose)
        )

    def _create_ik_request(self, pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout = Duration(sec=2)
        return req

    def _handle_ik_response(self, future, target_pose):
        try:
            response = future.result()
            if response.error_code.val != 1:
                raise RuntimeError('IK 계산 실패')
            
            joint_map = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position
            ))
            self.get_logger().info(f'관절 각도: {joint_map}')
            
            # 궤적 생성 및 실행
            traj = self._create_trajectory(joint_map)
            self._execute_trajectory(traj, target_pose)
            
        except Exception as e:
            self.get_logger().error(f'오류 발생: {str(e)}')

    def _create_trajectory(self, joint_map):
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4']
        positions = [joint_map[j] for j in arm_joints]
        
        traj = JointTrajectory()
        traj.joint_names = arm_joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2)
        traj.points.append(point)
        return traj

    def _execute_trajectory(self, traj, target_pose):
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        self.traj_cli.wait_for_server()
        
        send_goal_future = self.traj_cli.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda fut: self._handle_goal_response(fut, target_pose)
        )

    def _handle_goal_response(self, future, target_pose):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('궤적 목표 거부됨')
            return
        
        self.get_logger().info('궤적 실행 중...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut: self._handle_final_result(fut, target_pose)
        )

    def _handle_final_result(self, future, target_pose):
        result = future.result().result
        if result.error_code != 0:
            self.get_logger().error(f'실행 오류 코드: {result.error_code}')
        else:
            self.get_logger().info(
                f'이동 완료: x={target_pose.position.x:.3f}, '
                f'y={target_pose.position.y:.3f}, '
                f'z={target_pose.position.z:.3f}'
            )

def main():
    rclpy.init()
    node = GoToPose()
    
    # 5. 멀티스레드 Executor 설정
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
