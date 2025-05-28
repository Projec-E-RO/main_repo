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
    images = glob.glob('./checkerboards/*.png')

    # 코너가 한 번도 발견되지 않았을 경우를 위한 플래그
    found_corners = False

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret == True:
            found_corners = True
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # 코너 그리기 및 표시
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(0)

    cv2.destroyAllWindows()

    if not found_corners:
        print("Error: No chessboard corners found in any image!")
        return None

    # 카메라 캘리브레이션 수행
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("Camera matrix : \n")
    print(mtx)
    print("\ndist : \n")
    print(dist)
    print("\nrvecs : \n")
    print(rvecs)
    print("\ntvecs : \n")
    print(tvecs)

    # 캘리브레이션 결과 저장
    calibration_data = {
        'camera_matrix': mtx,
        'dist_coeffs': dist,
        'rvecs': rvecs,
        'tvecs': tvecs
    }
    with open('camera_calibration.pkl', 'wb') as f:
        pickle.dump(calibration_data, f)

    return calibration_data

if __name__ == "__main__":
    calibration_data = calibrate_camera()
    if calibration_data is not None:
        print("Calibration completed and saved to 'camera_calibration.pkl'")
    else:
        print("Calibration failed. Please check your images.")
