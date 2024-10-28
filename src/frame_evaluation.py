import time
import cv2
from queue import Queue
import warnings
from ultralytics import YOLO
import threading

from .camera_capture import get_frame_from_queue
from .frame_container import FrameContainer
from .frame_container import DetectedObject

def evaluate_captured_frames(queue_captured: Queue, #src queue
                             queue_distance: Queue, #destination queue
                             event_stop: threading.Event,
                             keyword: str):
    """Evaluate captured frames and putting the good ones into a queue
    
    run in a thread"""

    # Initialize the YOLOv8 model
    print("Loding YOLOv8 Model...")
    model = YOLO('yolov8n.pt')  # Load YOLOv8 model
    print("\rDone.")
    

    while not event_stop.is_set():
        # load latest from queue
        frame_container = get_frame_from_queue(queue_frame=queue_captured)
        if frame_container is None:
            continue

        # do object detection
        frame_container = _detect_objects(frame_container, model)

        # debug print
        # cv2.imshow("Debug: Object Detection",frame_container.get_frame_with_objects_detected())
        # cv2.waitKey(1000)

        # give matching rating
        frame_container.rate_matching(keyword)

        # if above certain threshold, do evaluate distance and put into closer choice queue
        if frame_container.matchings is not None \
            and frame_container.is_matching_significant():
            # calculate distance
            frame_container = _estimate_distance(frame_container)

            # put these frames into a queueq
            if queue_distance.full():
                queue_distance.get()
            queue_distance.put(frame_container)
    return

def _detect_objects(container: FrameContainer, model) -> FrameContainer:
    """Detects Objects in the frame container.
    
    Directly modifies the Frame Container (returns same container)"""
    time_start = time.time()

    frame_left = container.frame_left

    results = model(frame_left)

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


def _estimate_distance(container: FrameContainer) -> FrameContainer:
    """Estimates the distance to the best matched object in the frame container
    
    Directly modifies the Frame Container (returns same container)"""
    time_start = time.time()

    warnings.warn("TODO implement this function. Right now this is just a dummy function.")

    time_end = time.time()
    print(f"[Distance Estimation] Execution time:\t{time_end - time_start:.6f} s")
    return container