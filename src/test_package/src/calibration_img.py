import cv2
import datetime
import pyrealsense2 as rs
import numpy as np

# Realsense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # 프레임을 numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())

        # 프레임 표시
        cv2.imshow("Realsense Video", color_image)

        # 키 입력 대기
        key = cv2.waitKey(1) & 0xFF

        # 'a' 키: 이미지 저장
        if key == ord('a'):
            filename = datetime.datetime.now().strftime("./checkerboards/capture_%Y%m%d_%H%M%S.png")
            cv2.imwrite(filename, color_image)
            print(f"{filename} 이미지가 저장되었습니다.")

        # 'q' 키: 종료
        elif key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
