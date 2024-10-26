import cv2
import numpy as np
from pygrabber.dshow_graph import FilterGraph

def get_available_cameras() :

    devices = FilterGraph().get_input_devices()

    available_cameras = {}

    for device_index, device_name in enumerate(devices):
        available_cameras[device_index] = device_name

    return available_cameras

# Set camera ports (change these according to your setup)
camera_port_1 = 1  # First camera port
camera_port_2 = 2  # Second camera port

# Initialize the video captures
cap1 = cv2.VideoCapture(camera_port_1)
cap2 = cv2.VideoCapture(camera_port_2)

# Check if both cameras opened successfully
if not cap1.isOpened() or not cap2.isOpened():
    print("Error: Could not open one of the cameras.")
    exit()

while True:
    # Capture frame-by-frame from both cameras
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    # Check if frames are returned correctly
    if not ret1 or not ret2:
        print("Error: Could not read from one of the cameras.")
        break

    # Resize frames if necessary (optional)
    frame1 = cv2.resize(frame1, (640, 480))  # Resize frame1
    frame2 = cv2.resize(frame2, (640, 480))  # Resize frame2

    # Combine the frames side by side
    combined_frame = np.hstack((frame1, frame2))
    cv2.imshow('Combined Video', combined_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video captures and close the window
cap1.release()
cap2.release()
cv2.destroyAllWindows()