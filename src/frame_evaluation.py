import time
import cv2
from queue import Queue
import numpy as np
from ultralytics import YOLO
import threading

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


def _estimate_distance(container: FrameContainer, stereo_matcher: cv2.StereoBM) -> FrameContainer:
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