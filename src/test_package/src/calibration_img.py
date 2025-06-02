import cv2
import datetime
import os

# checkerboards_webcam 폴더가 없으면 생성
if not os.path.exists('./checkerboards_webcam'):
    os.makedirs('./checkerboards_webcam')

# 웹캠 설정 (기본 카메라는 0번)
cap = cv2.VideoCapture(0)

# 해상도 설정 (선택사항)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

print("웹캠이 시작되었습니다.")
print("'a' 키를 눌러 이미지를 저장하고, 'q' 키를 눌러 종료하세요.")

try:
    while True:
        # 프레임 읽기
        ret, frame = cap.read()
        
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # 프레임 표시
        cv2.imshow("Webcam - Checkerboard Capture", frame)

        # 키 입력 대기
        key = cv2.waitKey(1) & 0xFF

        # 'a' 키: 이미지 저장
        if key == ord('a'):
            filename = datetime.datetime.now().strftime("./checkerboards_webcam/capture_%Y%m%d_%H%M%S.png")
            cv2.imwrite(filename, frame)
            print(f"{filename} 이미지가 저장되었습니다.")

        # 'q' 키: 종료
        elif key == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("웹캠이 종료되었습니다.")
