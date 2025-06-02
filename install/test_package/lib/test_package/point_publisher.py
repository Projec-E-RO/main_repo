#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from test_package.msg import RobotPose
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
import time

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('point_publisher')
        self.get_logger().info("물체감지 시작 및 좌표 발행")
        self.get_logger().info("YOLOv11s OBB + Realsense 객체 인식 시작")

        # YOLO OBB 모델 로드
        self.model = YOLO('yolo11s.pt')  # OBB 모델로 변경
        self.get_logger().info(f"모델 태스크: {self.model.task}")  # OBB 확인

        # Realsense 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # ROS 퍼블리셔
        self.publisher_ = self.create_publisher(RobotPose, 'object_position', 10)

        # 연속 감지 횟수 카운터
        self.one_counter = 0
        self.last_1_pose = None

        # FPS 계산을 위한 변수
        self.prev_time = time.time()
        self.fps = 0

        # 캘리브레이션 결과 (실제 캘리브레이션 결과로 대체)
        self.fx = 600.39586052
        self.fy = 601.20766753
        self.cx_img = 326.55439474   # 이미지 좌표계 중심 (cx)
        self.cy_img = 247.77241513   # 이미지 좌표계 중심 (cy)

        # 카메라 좌표계 오프셋 (카메라 기준 위치 조정)
        self.x_offset = -0.06
        self.y_offset = 0.03
        self.z_offset = 0.052

        # 층 정보
        self.current_floor = 5      # 초기값 5
        self.goal_floor = 0         # 초기값 0
        self.goal_buttons = None    # 목표 버튼 정보

    def update_fps(self):
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.prev_time)
        self.prev_time = current_time
        return self.fps

    def run(self):
        try:
            while rclpy.ok():
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # YOLO OBB로 객체 탐지
                results = self.model(color_image, conf=0.5)

                one_detected = False
                
                # OBB 결과 처리
                if hasattr(results[0], 'obb') and results[0].obb is not None:
                    for box in results[0].obb:
                        try:
                            # OBB 좌표 추출 (8개 점: x1,y1,x2,y2,x3,y3,x4,y4)
                            xyxyxyxy = box.xyxyxyxy[0].cpu().numpy()
                            conf = float(box.conf[0])
                            cls = int(box.cls[0])
                            label = self.model.names[cls]

                            # 라벨을 button에서 goal_button으로 변경
                            if label == 'milk':  # 원하는 라벨로 변경 가능
                            # if label == 'goal_button':  # 원하는 라벨로 변경 가능
                                one_detected = True
                                
                                # OBB 중심점 계산 (4개 점의 평균)
                                points = xyxyxyxy.reshape(4, 2)
                                cx_center = int(np.mean(points[:, 0]))  # x 좌표들의 평균
                                cy_center = int(np.mean(points[:, 1]))  # y 좌표들의 평균

                                # 깊이 값 확인
                                if (0 <= cx_center < depth_image.shape[1] and 
                                    0 <= cy_center < depth_image.shape[0]):
                                    z = depth_image[cy_center, cx_center] * 0.001
                                    if z == 0 or z > 2.0:
                                        continue

                                    # 캘리브레이션 결과로 3D 좌표 계산
                                    x = (cx_center - self.cx_img) / self.fx * z
                                    y = (cy_center - self.cy_img) / self.fy * z

                                    # 좌표축 변환 (카메라 위치 기준)
                                    x_real = z
                                    y_real = -x
                                    z_real = -y

                                    # 카메라 오프셋 적용
                                    x_end = x_real + self.x_offset
                                    y_end = y_real + self.y_offset
                                    z_end = z_real + self.z_offset

                                    self.last_1_pose = (x_end, y_end, z_end, label)

                                    # OBB 시각화 (회전된 사각형 그리기)
                                    text = f'{label} {conf:.2f}, X:{x_end:.3f} Y:{y_end:.3f} Z:{z_end:.3f}'
                                    points_int = points.astype(int)
                                    cv2.polylines(color_image, [points_int], True, (0, 255, 0), 2)
                                    cv2.putText(color_image, text, (cx_center, cy_center - 10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                                    
                                    # 중심점 표시
                                    cv2.circle(color_image, (cx_center, cy_center), 5, (0, 0, 255), -1)
                                    
                        except Exception as e:
                            self.get_logger().warn(f"OBB 처리 중 오류: {str(e)}")
                            continue
                            
                else:
                    # OBB 결과가 없을 경우 일반 박스로 처리
                    self.get_logger().warn("OBB 결과가 없습니다. 일반 박스로 처리합니다.")
                    if hasattr(results[0], 'boxes') and results[0].boxes is not None:
                        for box in results[0].boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            cls = int(box.cls[0])
                            label = self.model.names[cls]

                            if label == 'button1':
                                one_detected = True
                                cx_center = int((x1 + x2) / 2)
                                cy_center = int((y1 + y2) / 2)

                                z = depth_image[cy_center, cx_center] * 0.001
                                if z == 0 or z > 2.0:
                                    continue

                                # 캘리브레이션 결과로 3D 좌표 계산
                                x = (cx_center - self.cx_img) / self.fx * z
                                y = (cy_center - self.cy_img) / self.fy * z

                                # 좌표축 변환 (카메라 위치 기준)
                                x_real = z
                                y_real = -x
                                z_real = -y

                                # 카메라 오프셋 적용
                                x_end = x_real + self.x_offset
                                y_end = y_real + self.y_offset
                                z_end = z_real + self.z_offset

                                self.last_1_pose = (x_end, y_end, z_end, label)

                                # 일반 바운딩 박스 시각화
                                text = f'{label} {conf:.2f}, X:{x_end:.3f} Y:{y_end:.3f} Z:{z_end:.3f}'
                                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.putText(color_image, text, (x1, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # 키보드 입력으로 goal_floor 값 받기 (1~6)
                key = cv2.waitKey(1)
                if key >= ord('1') and key <= ord('6'):
                    self.goal_floor = int(chr(key))
                    # goal_buttons 정보 생성
                    if self.goal_floor > self.current_floor:
                        self.goal_buttons = ('button_up', self.goal_floor)
                    elif self.goal_floor < self.current_floor:
                        self.goal_buttons = ('button_down', self.goal_floor)
                    else:
                        self.goal_buttons = None
                    # 콘솔 출력
                    if self.goal_buttons:
                        self.get_logger().info(f"Goal buttons: {self.goal_buttons}")
                    else:
                        self.get_logger().info("Same floor, no button to press")

                if one_detected:
                    self.one_counter += 1

                if self.one_counter >= 30 and self.last_1_pose:
                    x_end, y_end, z_end, label = self.last_1_pose

                    msg = RobotPose()
                    msg.x = x_end
                    msg.y = y_end
                    msg.z = z_end
                    msg.label = label
                    
                    if label == 'milk':
                        msg.gripper_open = True
                    else:
                        msg.gripper_open = False
                        
                    self.publisher_.publish(msg)

                    self.get_logger().info(
                        f"goal_button detected 30 times in a row! Publishing pose: {x_end:.3f}, {y_end:.3f}, {z_end:.3f}")
                    
                    self.one_counter = 0
                    self.last_1_pose = None

                # FPS 계산 및 화면에 표시
                fps = self.update_fps()
                cv2.putText(color_image, f"FPS: {fps:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                # 현재 층 및 목표 층 정보 표시
                cv2.putText(color_image, f"Current: {self.current_floor}, Goal: {self.goal_floor}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if self.goal_buttons:
                    cv2.putText(color_image, f"Press: {self.goal_buttons[0]} for floor {self.goal_buttons[1]}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 모델 타입 표시
                cv2.putText(color_image, f"Model: OBB", (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                cv2.imshow('YOLOv11s OBB Detection with Depth', color_image)
                if key == 27:  # ESC 종료
                    break

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = ObjectDetector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
