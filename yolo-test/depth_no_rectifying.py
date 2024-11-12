import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque
import time

# Camera parameter
K1 = np.array([[638.13621485, 0, 307.87103104], [0, 950, 240], [0, 0, 1]], dtype=np.float32)  # Intrinsic matrix (left)
K2 = K1.copy()  # Assuming both cameras are identical
D1 = np.zeros((1, 5))  # Distortion coefficients (adjust based on calibration)
D2 = np.zeros((1, 5))

# Initialize the YOLOv8 model
model = YOLO('yolov10n.pt')  # Load YOLOv8 model
# Initialize a queue to store frames
frame_queue = deque(maxlen=10)

# Open the two USB cameras
cap_left = cv2.VideoCapture(0)  # First USB camera
phone_camera_url = "http://192.168.31.123:4747 /video"  # Make sure the IP is correct
# Open the stream from the phone camera
cap_right = cv2.VideoCapture(phone_camera_url)
#cap_right = cv2.VideoCapture(2)  # Second USB camera

# Check if both cameras are accessible
if not cap_left.isOpened() or not cap_right.isOpened():
    print("Error: Could not open both cameras.")
    exit()

# Queue to store detected tags from the left camera
tag_list = []
last_capture_time = time.time()

# Camera parameters (tune based on your setup)
focal_length = 950  # Focal length in pixels (adjust to your camera)
baseline = 10  # Baseline in cm (distance between cameras)

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

        # Convert frames to grayscale for depth map calculation
        gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)



        # Compute the depth map
        disparity_left = stereo_left.compute(gray_left, gray_right).astype(np.float32) / 16.0
        disparity_left[disparity_left == 0] = 0.1  # Avoid division by zero
        disparity_right = stereo_right.compute(gray_right, gray_left).astype(np.float32) / 16.0

        wls_filter = cv2.ximgproc.createDisparityWLSFilter(stereo_left)
        wls_filter.setLambda(1000)
        wls_filter.setSigmaColor(1.5)
        filtered_disp = wls_filter.filter(disparity_left, gray_left, disparity_map_right=disparity_right)
        filtered_disp[filtered_disp == 0] = 0.1

        #depth map
        depth_map= (baseline*focal_length)/filtered_disp

        # Normalize the depth map for visualization
        depth_disparity_norm = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_map_norm=  cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # Process every 5 seconds
        current_time = time.time()
        if current_time - last_capture_time >= 5:
            # Perform YOLO detection on the left frame
            frame_queue.append(frame_left)
            # YOLO
            results = model(frame_left)

            # Extract detected labels and store them in the tag list
            tags = [model.names[int(cls)] for cls in results[0].boxes.cls]
            tag_list.append(tags)
            print(f"Detected tags: {tags}")

            # Draw bounding boxes and labels on the left frame
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                label = model.names[int(box.cls[0])]  # Class label
                conf = box.conf[0]  # Confidence score

                # Draw a green rectangle and label
                cv2.rectangle(frame_left, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} {conf:.2f}"

                # Calculate depth at the center of the bounding box
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                distance = 10*50/disparity_left[center_y, center_x]  # Depth value at center
                text += f" {distance:.2f} cm"

                # Put the text on the frame
                cv2.putText(frame_left, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            cv2.imshow('Left Camera with YOLO Detections', frame_left)
            last_capture_time = current_time  # Update last capture time

        # Display the left frame with YOLO detections
        cv2.imshow('Live Camera', frame_left)

        # Display the depth map
        #depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        #overlay = cv2.addWeighted(frame_left, 0.6, depth_map_normalized, 0.4, 0)
        #depth_colormap = cv2.applyColorMap(depth_disparity_norm, cv2.COLORMAP_JET)
        Q = np.array([
            [1, 0, 0, -100],
            [0, 1, 0, -100],
            [0, 0, 0, 100],
            [0, 0, -1 / baseline, 0]
        ])
        points3D= cv2.reprojectImageTo3D(filtered_disp, Q)
        depth_map = points3D[:, :, 2]
        depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow('Depth Map', depth_disparity_norm )

        # Press 'q' to quit the program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted. Exiting...")

# Release both cameras and close all OpenCV windows
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()

# Convert the tag list to a NumPy array and save it
tag_array = np.array(tag_list, dtype=object)
print(f"Tag array:\n{tag_array}")

# Optionally save the tags to a file
np.save('detected_tags.npy', tag_array)

print("Process completed.")
