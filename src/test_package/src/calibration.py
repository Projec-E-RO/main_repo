import cv2
import numpy as np
import glob
import pickle

def calibrate_camera():
    # 체커보드의 차원 정의 (내부 코너 수)
    CHECKERBOARD = (7, 10)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 3D 점 벡터 & 2D 점 벡터
    objpoints = []  # 3D 점 (체커보드 좌표계)
    imgpoints = []  # 2D 점 (이미지 좌표계)

    # 3D 점 생성 (체커보드 좌표계)
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    # 체커보드 이미지들 불러오기
    images = glob.glob('./checkerboards_webcam/*.png')
    
    if len(images) == 0:
        print("Error: ./checkerboards_webcam/ 폴더에 이미지가 없습니다!")
        return None

    print(f"{len(images)}개의 이미지를 찾았습니다.")

    # 코너가 한 번도 발견되지 않았을 경우를 위한 플래그
    found_corners = False
    successful_images = 0

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Warning: {fname} 이미지를 읽을 수 없습니다.")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret == True:
            found_corners = True
            successful_images += 1
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # 코너 그리기 및 표시
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow(f'Detected corners - {fname}', img)
            print(f"✓ {fname}: 체커보드 코너를 찾았습니다.")
            cv2.waitKey(500)  # 0.5초 대기
        else:
            print(f"✗ {fname}: 체커보드 코너를 찾을 수 없습니다.")

    cv2.destroyAllWindows()

    if not found_corners:
        print("Error: 어떤 이미지에서도 체커보드 코너를 찾을 수 없습니다!")
        print("체커보드가 명확하게 보이는 이미지를 다시 촬영해주세요.")
        return None

    print(f"\n총 {successful_images}개의 이미지에서 체커보드를 성공적으로 감지했습니다.")

    # 카메라 캘리브레이션 수행
    print("카메라 캘리브레이션을 수행중입니다...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("\n=== 캘리브레이션 결과 ===")
    print("Camera matrix (카메라 내부 파라미터):")
    print(mtx)
    print("\nDistortion coefficients (왜곡 계수):")
    print(dist)
    print(f"\nReprojection error: {ret}")

    # 캘리브레이션 결과 저장
    calibration_data_webcam = {
        'camera_matrix': mtx,
        'dist_coeffs': dist,
        'rvecs': rvecs,
        'tvecs': tvecs,
        'reprojection_error': ret
    }
    
    with open('camera_calibration_webcam.pkl', 'wb') as f:
        pickle.dump(calibration_data_webcam, f)

    return calibration_data_webcam

if __name__ == "__main__":
    calibration_data_webcam = calibrate_camera()
    if calibration_data_webcam is not None:
        print("\n✓ 캘리브레이션이 완료되어 'camera_calibration.pkl' 파일에 저장되었습니다.")
    else:
        print("\n✗ 캘리브레이션에 실패했습니다. 이미지를 확인해주세요.")
