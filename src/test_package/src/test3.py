#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from test_package.msg import RobotPose

class GoToPoseWithGripper(Node):
    def __init__(self):
        super().__init__('go_to_pose_with_gripper')
        
        # 실행 여부 플래그
        self.executed = False
        
        # 그리퍼 상수 정의
        self.OPEN_POS = 0.040
        self.PICK_POS = 0.006
        self.CLOSE_POS = -0.01
        self.MAX_EFFORT = 0.5
        
        # 1. 콜백 그룹 생성
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.gripper_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 2. 서비스 클라이언트 초기화 (별도 콜백 그룹 할당)
        self.ik_cli = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.service_cb_group
        )
        while not self.ik_cli.wait_for_service(1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        
        # 3. 액션 클라이언트 초기화 (팔 제어)
        self.traj_cli = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 4. 그리퍼 액션 클라이언트 초기화
        self.gripper_cli = ActionClient(
            self, GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.gripper_cb_group
        )
        
        # 5. 구독자 생성 (별도 콜백 그룹 할당)
        self.subscription = self.create_subscription(
            RobotPose, 'object_position_with_tf', 
            self.pose_callback, 10,
            callback_group=self.sub_cb_group
        )
        
        # 그리퍼 서버 대기
        self.gripper_cli.wait_for_server()
        self.get_logger().info('노드 초기화 완료 (그리퍼 포함) - 첫 번째 메시지 대기 중...')

    def pose_callback(self, msg):
        # 이미 실행했으면 무시
        if self.executed:
            self.get_logger().info('이미 실행 완료됨 - 메시지 무시')
            return
        
        # 실행 플래그 설정
        self.executed = True
        
        self.get_logger().info(
            f'첫 번째 메시지 수신 - 동작 시작: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, '
            f'gripper_open={msg.gripper_open}'
        )
        
        target_pose = Pose()
        target_pose.position.x = msg.x
        target_pose.position.y = msg.y
        target_pose.position.z = msg.z
        target_pose.orientation.w = 1.0
        
        # 토픽에서 받은 gripper_open 값에 따라 그리퍼 제어
        gripper_position = self.OPEN_POS if msg.gripper_open else self.CLOSE_POS
        gripper_reaction = self.PICK_POS if msg.gripper_open else self.OPEN_POS
        gripper_action = "open" if msg.gripper_open else "close"
        gripper_reaction_action = "pick" if msg.gripper_open else "open"
        
        # 1단계: 첫 번째 그리퍼 동작
        self._control_gripper(
            gripper_position,
            gripper_action, 
            lambda: self._start_arm_movement(target_pose, gripper_reaction, gripper_reaction_action)
        )

    def _start_arm_movement(self, target_pose, final_gripper_pos, final_action):
        """그리퍼 제어 완료 후 팔 이동 시작"""
        # 비동기 서비스 호출
        future = self.ik_cli.call_async(
            self._create_ik_request(target_pose)
        )
        future.add_done_callback(
            lambda fut: self._handle_ik_response(fut, target_pose, final_gripper_pos, final_action)
        )

    def _create_ik_request(self, pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout = Duration(sec=2)
        return req

    def _handle_ik_response(self, future, target_pose, final_gripper_pos, final_action):
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
            self._execute_trajectory(traj, target_pose, final_gripper_pos, final_action)
            
        except Exception as e:
            self.get_logger().error(f'오류 발생: {str(e)}')
            self._shutdown_node()

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

    def _execute_trajectory(self, traj, target_pose, final_gripper_pos, final_action):
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        self.traj_cli.wait_for_server()
        
        send_goal_future = self.traj_cli.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda fut: self._handle_goal_response(fut, target_pose, final_gripper_pos, final_action)
        )

    def _handle_goal_response(self, future, target_pose, final_gripper_pos, final_action):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('궤적 목표 거부됨')
            self._shutdown_node()
            return
        
        self.get_logger().info('궤적 실행 중...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut: self._handle_final_result(fut, target_pose, final_gripper_pos, final_action)
        )

    def _handle_final_result(self, future, target_pose, final_gripper_pos, final_action):
        result = future.result().result
        if result.error_code != 0:
            self.get_logger().error(f'실행 오류 코드: {result.error_code}')
            self._shutdown_node()
        else:
            self.get_logger().info(
                f'이동 완료: x={target_pose.position.x:.3f}, '
                f'y={target_pose.position.y:.3f}, '
                f'z={target_pose.position.z:.3f}'
            )
            
            # 3단계: 마지막 그리퍼 동작 (완료 후 노드 종료)
            self._control_gripper(
                final_gripper_pos, 
                final_action,
                self._shutdown_node
            )

    def _control_gripper(self, position1: float, tag: str, callback=None):
        """그리퍼 제어 함수"""
        goal = GripperCommand.Goal()
        goal.command.position = position1
        goal.command.max_effort = self.MAX_EFFORT

        self.get_logger().info(f'{tag.capitalize()} … (pos={position1})')

        # 1) goal 전송
        send_future = self.gripper_cli.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut: self._handle_gripper_goal(fut, tag, callback)
        )

    def _handle_gripper_goal(self, future, tag: str, callback=None):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{tag.capitalize()} goal rejected')
            if callback:
                callback()
            return

        # 2) 결과 대기
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(
            lambda fut: self._handle_gripper_result(fut, tag, callback)
        )

    def _handle_gripper_result(self, future, tag: str, callback=None):
        wrapped = future.result()
        status = wrapped.status
        reached = wrapped.result.reached_goal
        stalled = wrapped.result.stalled

        if status == GoalStatus.STATUS_SUCCEEDED and reached:
            self.get_logger().info(f'{tag.capitalize()} done')
            # 그리퍼 동작 완료 후 콜백 실행
            if callback:
                callback()
        else:
            self.get_logger().error(
                f'{tag.capitalize()} failed (status={status}, stalled={stalled})')
            if callback:
                callback()

    def _shutdown_node(self):
        """노드 종료"""
        self.get_logger().info('모든 동작 완료 - 노드 종료 중...')
        # 잠시 대기 후 종료
        self.create_timer(1.0, lambda: rclpy.shutdown())

def main():
    rclpy.init()
    node = GoToPoseWithGripper()
    
    # 멀티스레드 Executor 설정 (그리퍼 제어를 위해 스레드 수 증가)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의한 노드 종료...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
