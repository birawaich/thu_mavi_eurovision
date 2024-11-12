import numpy as np
import cv2
import yaml #added to import offline parameters

#Missing: Get calibration matrices and save them in YAML file


# Load camera calibration parameters from YAML file
def load_calibration(file_path):
    with open(file_path, 'r') as file:
        calibration_data = yaml.safe_load(file)
    K = np.array(calibration_data["K"]) #camera matrix
    d = np.array(calibration_data["D"]) #distortion parameters
    R = np.array(calibration_data["R"]) #rotation matrix
    T = np.array(calibration_data["T"]) #translation matrix
    return K, d, R, T


def undistort_image(left_image, right_image, image_size,
                    calibrate_left:yaml = (np.eye(3, dtype=float),
                                           np.ones(5, dtype=float),
                                           np.eye(3, dtype=float),
                                           np.array([[0.1], [0.0], [0.0]], dtype=np.float32)),
                    calibrate_right:yaml= (np.eye(3, dtype=float),
                                           np.ones(5, dtype=float),
                                           np.eye(3, dtype=float),
                                           np.array([[0.1], [0.0], [0.0]], dtype=np.float32))):
    K1, d1, R1, T1 = load_calibration(calibrate_left)
    K2, d2, R2, T2 = load_calibration(calibrate_right)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, d1, K2, d2, image_size, R1, T1)

    # Undistortion and rectification map
    map1_left, map2_left = cv2.initUndistortRectifyMap(K1, d1, R1, P1, image_size, cv2.CV_32FC1)
    map1_right, map2_right = cv2.initUndistortRectifyMap(K2, d2, R2, P2, image_size, cv2.CV_32FC1)

    rectified_left = cv2.remap(left_image, map1_left, map2_left, cv2.INTER_LINEAR)
    rectified_right = cv2.remap(right_image, map1_right, map2_right, cv2.INTER_LINEAR)

    return rectified_left,rectified_right, Q






