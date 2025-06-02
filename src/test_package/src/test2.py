#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from test_package.msg import RobotPose

class TfPoseMerger(Node):
    def __init__(self):
        super().__init__('tf_pose_merger')
        self.get_logger().info("TF + Pose 통합 노드 시작")

        # TF2 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO로부터 좌표 구독
        self.subscription = self.create_subscription(
            RobotPose,
            'object_position',
            self.pose_callback,
            10)

        # 최종 좌표 발행
        self.publisher_ = self.create_publisher(
            RobotPose,
            'object_position_with_tf',
            10)

    def pose_callback(self, msg: RobotPose):
        from_frame = 'link1'
        to_frame = 'end_effector_link'

        try:
            # 현재 시간 기준 TF 가져오기
            now = rclpy.time.Time()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame, to_frame, now)

            tf_x = transform.transform.translation.x
            tf_y = transform.transform.translation.y
            tf_z = transform.transform.translation.z

            # 구독한 좌표 + TF 오프셋
            merged_msg = RobotPose()
            merged_msg.x = msg.x + tf_x
            merged_msg.y = msg.y + tf_y
            merged_msg.z = msg.z + tf_z
            merged_msg.label = msg.label
            merged_msg.gripper_open = msg.gripper_open

            self.publisher_.publish(merged_msg)

            self.get_logger().info(
                f"[INPUT]  x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f} | label={msg.label}\n"
                f"[TF]     dx={tf_x:.3f}, dy={tf_y:.3f}, dz={tf_z:.3f}\n"
                f"[OUTPUT] x={merged_msg.x:.3f}, y={merged_msg.y:.3f}, z={merged_msg.z:.3f}"
            )

        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {from_frame} -> {to_frame} : {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TfPoseMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
