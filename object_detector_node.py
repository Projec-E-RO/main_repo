import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info("YOLOv11s + Realsense 객체 인식 시작")

        # 최신 Ultralytics API로 YOLOv11s 모델 로드
        self.model = YOLO('best.pt')  # yolov11s.pt 파일이 프로젝트 디렉터리에 있어야 함

        # Realsense 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

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

                # 최신 API: conf=0.5로 confidence threshold 지정
                results = self.model(color_image, conf=0.5)
                # results[0].boxes.xyxy, results[0].boxes.conf, results[0].boxes.cls 등 사용
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                    # Depth 추출
                    z = depth_image[cy, cx] * 0.001  # mm → meter
                    if z == 0 or z > 2.0:
                        continue

                    # 3D 좌표 계산 (RealSense 기본 좌표계 → 원하는 좌표계)
                    intr = color_frame.profile.as_video_stream_profile().intrinsics
                    x = (cx - intr.ppx) / intr.fx * z
                    y = (cy - intr.ppy) / intr.fy * z

                    x_real = z
                    y_real = -x
                    z_real = -y

                    x_end = x_real - 0.09
                    y_end = y_real
                    z_end = z_real + 0.06

                    label = self.model.names[cls]
                    text = f'{label} {conf:.2f}, X:{x_end:.3f}m Y:{y_end:.3f}m Z:{z_end:.3f}m'
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

def main():
    rclpy.init()
    node = ObjectDetector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
