import cv2
import numpy as np
import yaml
from ultralytics import YOLO
from collections import deque
import time

# Load camera calibration parameters from YAML file
def load_calibration(file_path):
    with open(file_path, 'r') as file:
        calibration_data = yaml.safe_load(file)
    K = np.array(calibration_data["K"])
    D = np.array(calibration_data["D"])
    R = np.array(calibration_data["R"])
    T = np.array(calibration_data["T"])
    return K, D, R, T

# Load calibration files for left and right cameras
K1, D1, R1, T1= load_calibration('camera_1_extrinsics.yaml')
K2, D2, R2, T2= load_calibration('camera_2_extrinsics.yaml')

# Initialize the YOLO model
model = YOLO('yolov8n.pt')  # Load YOLOv8 model

# Open the two USB cameras, change if necessary
cap_left = cv2.VideoCapture(2)
phone_camera_url = "http://192.168.31.214:4747/video"
cap_right = cv2.VideoCapture(3)

# Check if both cameras are accessible
if not cap_left.isOpened() or not cap_right.isOpened():
    print("Error: Could not open both cameras.")
    exit()

# Queue to store frames
frame_queue = deque(maxlen=10)
last_capture_time = time.time()

# Stereo rectification
image_size = (640, 600)  # Adjust based on your camera setup
R, T = R2, T2  # Use extrinsic parameters for stereo
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, T)

# Undistortion and rectification map
map1_left, map2_left = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
map1_right, map2_right = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)

# StereoBM matcher (for computing the depth map)
stereo_left = cv2.StereoBM_create(numDisparities=16, blockSize=15)
stereo_right = cv2.ximgproc.createRightMatcher(stereo_left)

try:
    while True:
        # Capture frames from both cameras
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        if not ret_left or not ret_right:
            print("Error: Could not read frames from both cameras.")
            break

        # Rectify and undistort frames
        rectified_left = cv2.remap(frame_left, map1_left, map2_left, cv2.INTER_LINEAR)
        rectified_right = cv2.remap(frame_right, map1_right, map2_right, cv2.INTER_LINEAR)

        # Convert frames to grayscale for depth map calculation
        gray_left = cv2.cvtColor(rectified_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rectified_right, cv2.COLOR_BGR2GRAY)

        # Compute disparity map
        disparity_left = stereo_left.compute(gray_left, gray_right).astype(np.float32) / 16.0
        disparity_left[disparity_left == 0] = 0.1  # Avoid division by zero
        disparity_right = stereo_right.compute(gray_right, gray_left).astype(np.float32) / 16.0
        disparity_right[disparity_right == 0] = 0.1  # Avoid division by zero

        # Apply WLS filter to improve disparity
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(stereo_left)
        wls_filter.setLambda(1000)
        wls_filter.setSigmaColor(1.5)
        filtered_disp = wls_filter.filter(disparity_left, gray_left, disparity_map_right=disparity_right)
        filtered_disp[filtered_disp == 0] = 0.1
        filtered_disp_normalized = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Compute depth map using Q matrix
        #depth_map = cv2.reprojectImageTo3D(filtered_disp, Q)q
        #depth_map_normalized = cv2.normalize(depth_map[:, :, 2], None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_map2= 10*50/filtered_disp
        #depth_map_normalized2 = cv2.normalize(depth_map2, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # Display depth map as color map
        #depth_colormap = cv2.applyColorMap(depth_map_normalized, cv2.COLORMAP_JET)
        cv2.imshow('Depth Map', filtered_disp_normalized)

        # Process YOLO detection on left frame every 5 seconds
        current_time = time.time()
        if current_time - last_capture_time >= 5:
            results = model(rectified_left)
            tags = [model.names[int(cls)] for cls in results[0].boxes.cls]
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = model.names[int(box.cls[0])]
                conf = box.conf[0]
                # Only proceed if the detected label is "bottle"
                if label == "bottle":
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    distance = depth_map2[center_x, center_y]  # Depth at the center
                    cv2.rectangle(rectified_left, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text = f"{label} {conf:.2f} {distance:.2f} cm"
                    cv2.putText(rectified_left, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('Left Camera with YOLO Detections', rectified_left)
            last_capture_time = current_time

        # Display rectified left frame
        cv2.imshow('Rectified Left Camera', rectified_left)

        # Press 'q' to quit the program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted. Exiting...")

# Release cameras and close OpenCV windows
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()