import time
import cv2
from queue import Queue
import numpy as np
from ultralytics import YOLO
import threading
import os

from .camera_capture import get_frame_from_queue
from .frame_container import FrameContainer
from .frame_container import DetectedObject

def evaluate_captured_frames(queue_captured: Queue, #src queue
                             queue_distance: Queue, #destination queue
                             event_stop: threading.Event):
    """Evaluate captured frames and putting the good ones into a queue
    
    run in a thread"""

    # Initialize the YOLOv8 model
    print("Loding YOLOv8 Model...")
    model = YOLO('yolov8n.pt')  # Load YOLOv8 model
    print("\rDone.")

    # Initialize a StereoBM matcher
    print("Creating a StereoBM matcher...")
    stereo_matcher = cv2.StereoBM_create(numDisparities=16*6, blockSize=15)
    print("\rDone.")
    

    while not event_stop.is_set():
        # load latest from queue
        frame_container = get_frame_from_queue(queue_frame=queue_captured)
        if frame_container is None:
            continue

        # do object detection
        frame_container = _detect_objects(frame_container, model)

        # debug print
        cv2.imshow("Debug: Object Detection",frame_container.get_frame_with_objects_detected())
        cv2.waitKey(1)

        # give matching rating
        frame_container.rate_matching()

        # if above certain threshold, do evaluate distance and put into closer choice queue
        if len(frame_container.matchings) != 0:
            # calculate distance
            frame_container = _estimate_distance(frame_container, stereo_matcher)

            # put these frames into a queueq
            if queue_distance.full():
                queue_distance.get()
            queue_distance.put(frame_container)

            cv2.imshow("Debug: Matchings",frame_container.get_frame_with_matching_distances())
            cv2.waitKey(1)

            cv2.imshow("Debug: Depth Map", frame_container.depthmap)
            cv2.waitKey(1)
    return

def _detect_objects(container: FrameContainer, model: YOLO) -> FrameContainer:
    """Detects Objects in the frame container.
    
    Directly modifies the Frame Container (returns same container)"""
    time_start = time.time()

    frame_left = container.frame_left

    results = model(frame_left, verbose=False)

    # extract detected labels etc and store into FrameContainer
    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
        label_id = int(box.cls[0])
        label = model.names[label_id]  # Class label
        conf = float(box.conf[0])  # Confidence score
        container.detected_objects.append(
            DetectedObject(label_name=label,
                           label_id=label_id,
                           confidence=conf,
                           x1=x1,
                           x2=x2,
                           y1=y1,
                           y2=y2)
        )

    time_end = time.time()
    print(f"[Object Detection] Execution time:\t{time_end - time_start:.6f} s")
    return container


def _estimate_distance_old(container: FrameContainer, stereo_matcher: cv2.StereoBM) -> FrameContainer:
    """Estimates the distance to the best matched object in the frame container
    
    Directly modifies the Frame Container (returns same container)"""
    time_start = time.time()

    # Convert frames to grayscale for depth map calculation
    gray_left = cv2.cvtColor(container.frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(container.frame_right, cv2.COLOR_BGR2GRAY)

    # Compute the depth map
    disparity = stereo_matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
    disparity[disparity == 0] = 0.1  # Avoid division by zero

    # add depth map to frame
    container.depthmap = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # go through matchings and assign a distance
    for matching in container.matchings:
        # calculate center of box
        center_x, center_y = (matching.x1 + matching.x2) // 2,\
            (matching.y1 + matching.y2) // 2
        # take distance as distance to center #TODO do this better
        matching.distance_cm = disparity[center_y, center_x]  # Depth value at center

    time_end = time.time()
    print(f"[Distance Estimation] Execution time:\t{time_end - time_start:.6f} s")
    return container

class CameraCalibration:
    def __init__(self, parameter_folder):
        # Paths to the .dat files in the `camera_parameters` folder
        camera0_intrinsic_file = os.path.join(parameter_folder, 'camera0_intrinsics.dat')
        camera0_rot_trans_file = os.path.join(parameter_folder, 'camera0_rot_trans.dat')
        camera1_intrinsic_file = os.path.join(parameter_folder, 'camera1_intrinsics.dat')
        camera1_rot_trans_file = os.path.join(parameter_folder, 'camera1_rot_trans.dat')
        
        # Camera 0 parameters
        self.camera0_intrinsic_matrix = None
        self.camera0_distortion_coeffs = None
        self.camera0_rotation_matrix = None
        self.camera0_translation_vector = None
        
        # Camera 1 parameters
        self.camera1_intrinsic_matrix = None
        self.camera1_distortion_coeffs = None
        self.camera1_rotation_matrix = None
        self.camera1_translation_vector = None
        
        # Load calibration data for both cameras
        self.load_intrinsic(camera0_intrinsic_file, camera_num=0)
        self.load_rot_trans(camera0_rot_trans_file, camera_num=0)
        self.load_intrinsic(camera1_intrinsic_file, camera_num=1)
        self.load_rot_trans(camera1_rot_trans_file, camera_num=1)

    def load_intrinsic(self, filename, camera_num):
        with open(filename, 'r') as f:
            lines = f.readlines()
            intrinsic_data = lines[1:4]
            distortion_data = lines[5]
            
            # Intrinsic Matrix (3x3)
            intrinsic_matrix = np.array([
                [float(val) for val in intrinsic_data[0].split()],
                [float(val) for val in intrinsic_data[1].split()],
                [float(val) for val in intrinsic_data[2].split()]
            ])
            
            # Distortion coefficients
            distortion_coeffs = np.array([float(val) for val in distortion_data.split()])
            
            if camera_num == 0:
                self.camera0_intrinsic_matrix = intrinsic_matrix
                self.camera0_distortion_coeffs = distortion_coeffs
            elif camera_num == 1:
                self.camera1_intrinsic_matrix = intrinsic_matrix
                self.camera1_distortion_coeffs = distortion_coeffs

    def load_rot_trans(self, filename, camera_num):
        with open(filename, 'r') as f:
            lines = f.readlines()
            rotation_data = lines[1:4]
            translation_data = lines[5:8]
            
            # Rotation matrix (3x3)
            rotation_matrix = np.array([
                [float(val) for val in rotation_data[0].split()],
                [float(val) for val in rotation_data[1].split()],
                [float(val) for val in rotation_data[2].split()]
            ])
            
            # Translation vector (3x1)
            translation_vector = np.array([float(val) for val in translation_data])
            
            if camera_num == 0:
                self.camera0_rotation_matrix = rotation_matrix
                self.camera0_translation_vector = translation_vector
            elif camera_num == 1:
                self.camera1_rotation_matrix = rotation_matrix
                self.camera1_translation_vector = translation_vector

    def get_camera0_intrinsic(self):
        return self.camera0_intrinsic_matrix

    def get_camera0_distortion(self):
        return self.camera0_distortion_coeffs

    def get_camera0_rotation(self):
        return self.camera0_rotation_matrix

    def get_camera0_translation(self):
        return self.camera0_translation_vector

    # Methods to retrieve parameters for Camera 1
    def get_camera1_intrinsic(self):
        return self.camera1_intrinsic_matrix

    def get_camera1_distortion(self):
        return self.camera1_distortion_coeffs

    def get_camera1_rotation(self):
        return self.camera1_rotation_matrix

    def get_camera1_translation(self):
        return self.camera1_translation_vector
    
def _estimate_distance(container: FrameContainer, stereo_matcher: cv2.StereoBM, calibration: CameraCalibration) -> FrameContainer:
    """Estimates the distance to the best matched object in the frame container
    
    Directly modifies the Frame Container (returns same container)"""
    time_start = time.time()

    # Get the focal length from Camera 0 intrinsic matrix
    focal_length = calibration.get_camera0_intrinsic()[0, 0]  # f_x from camera 0

    # Get the baseline (translation in x direction from camera 0 to camera 1)
    baseline = abs(calibration.get_camera1_translation()[0])  # assuming translation is in cm

    # Convert frames to grayscale for depth map calculation
    gray_left = cv2.cvtColor(container.frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(container.frame_right, cv2.COLOR_BGR2GRAY)

    # Compute the depth map
    disparity = stereo_matcher.compute(gray_left, gray_right).astype(np.float32) / 16.0
    disparity[disparity <= 0] = 0.1  # Avoid division by zero or negative values

    # Add depth map to frame
    container.depthmap = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Go through matchings and assign a distance using camera calibration
    for matching in container.matchings:
        # Calculate center of bounding box
        center_x, center_y = (matching.x1 + matching.x2) // 2, (matching.y1 + matching.y2) // 2
        
        # Retrieve disparity value at center point
        disparity_value = disparity[center_y, center_x]
        
        # Calculate distance using calibration parameters (focal length and baseline)
        distance_cm = (focal_length * baseline) / disparity_value  # Distance in cm
        matching.distance_cm = distance_cm

    time_end = time.time()
    print(f"[Distance Estimation] Execution time:\t{time_end - time_start:.6f} s")
    return container