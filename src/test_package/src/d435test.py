#!/usr/bin/env python3

import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self):
        print("물체감지 시작")
        print("YOLOv11s + Realsense 객체 인식 시작")

        # YOLO 모델 로드
        self.model = YOLO('yolo11n.pt')

        # Realsense 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # 캘리브레이션 결과 (실제 캘리브레이션 결과로 대체)
        self.fx = 600.39586052
        self.fy = 601.20766753
        self.cx_img = 326.55439474   # 이미지 좌표계 중심 (cx)
        self.cy_img = 247.77241513   # 이미지 좌표계 중심 (cy)

        # 카메라 좌표계 오프셋 (카메라 기준 위치 조정)
        self.x_offset = -0.06
        self.y_offset = 0.03
        self.z_offset = 0.06

    def run(self):
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # YOLO로 객체 탐지
                results = self.model(color_image, conf=0.5, verbose=False)


                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = self.model.names[cls]

                    if label == 'button1':  # 원하는 라벨로 변경 가능
                        cx_center = int((x1 + x2) / 2)
                        cy_center = int((y1 + y2) / 2)

                        z = depth1 = depth_image[cy_center, cx_center] * 0.001  # 미터 단위
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

                        # 버튼 중앙 depth 크기 print 출력
                        print(f"button1 detected! Center Depth: {z:.3f}m")

                        # 시각화
                        text = f'{label} {conf:.2f}, X:{x_end:.3f} Y:{y_end:.3f} Z:{z_end:.3f}\nDepth: {z:.3f}m'
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_image, text, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                cv2.imshow('YOLOv11s Detection with Depth', color_image)
                key = cv2.waitKey(1)
                if key == 27:  # ESC 종료
                    break

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = ObjectDetector()
    detector.run()
