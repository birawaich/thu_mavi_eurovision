import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque
import time

# Initialize the YOLOv8 model
model = YOLO('yolov8n.pt')  # Load YOLOv8 model
# Initialize a queue to store frames
frame_queue = deque(maxlen=10)

# Open the two USB cameras
cap_left = cv2.VideoCapture(1)  # First USB camera
cap_right = cv2.VideoCapture(2)  # Second USB camera

# Check if both cameras are accessible
if not cap_left.isOpened() or not cap_right.isOpened():
    print("Error: Could not open both cameras.")
    exit()

# Queue to store detected tags from the left camera
tag_list = []
last_capture_time = time.time()

# StereoBM matcher (for computing the depth map)
stereo = cv2.StereoBM_create(numDisparities=16*6, blockSize=15)

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
        disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0
        disparity[disparity == 0] = 0.1  # Avoid division by zero
        # Normalize the depth map for visualization
        depth_map_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

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
                distance = disparity[center_y, center_x]  # Depth value at center
                text += f" {distance:.2f} cm"

                # Put the text on the frame
                cv2.putText(frame_left, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            cv2.imshow('Left Camera with YOLO Detections', frame_left)
            last_capture_time = current_time  # Update last capture time

        # Display the left frame with YOLO detections
        cv2.imshow('Live Camera', frame_left)

        # Display the depth map
        depth_colormap = cv2.applyColorMap(depth_map_norm, cv2.COLORMAP_JET)
        cv2.imshow('Depth Map', depth_map_norm)

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
