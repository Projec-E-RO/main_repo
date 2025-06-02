#!/usr/bin/env python3
import time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand

OPEN_POS   = 0.03
CLOSE_POS  = 0.006
MAX_EFFORT = 0.5
PAUSE_SEC  = 2.0

class GripperAuto(Node):
    def __init__(self):
        super().__init__('gripper_auto_open_close')
        self.cli = ActionClient(
            self, GripperCommand,
            '/gripper_controller/gripper_cmd')
        self.cli.wait_for_server()

        self.send_goal(OPEN_POS,  'open')
        time.sleep(PAUSE_SEC)
        self.send_goal(CLOSE_POS, 'close')
        rclpy.shutdown()

    # ------------------------------------------------------------------
    def send_goal(self, position: float, tag: str):
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = MAX_EFFORT

        self.get_logger().info(f'{tag.capitalize()} … (pos={position})')

        # 1) goal 전송
        send_future = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{tag.capitalize()} goal rejected')
            return

        # 2) 결과 대기
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        wrapped = res_future.result()           # WrappedResult
        status  = wrapped.status
        reached = wrapped.result.reached_goal
        stalled = wrapped.result.stalled

        if status == GoalStatus.STATUS_SUCCEEDED and reached:
            self.get_logger().info(f'{tag.capitalize()} done')
        else:
            self.get_logger().error(
                f'{tag.capitalize()} failed (status={status}, stalled={stalled})')

def main():
    rclpy.init()
    GripperAuto()

if __name__ == '__main__':
    main()
