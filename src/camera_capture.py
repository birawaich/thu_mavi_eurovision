import cv2
import threading
from queue import Queue
import datetime
import warnings

from src.frame_container import FrameContainer

class CameraSetup:
    """Class to hold the camera setup"""

    def __init__(self,
                 port_left: int,
                 port_right: int):
        self.port_left = port_left
        self.port_right = port_right
        return

def capture_frames(camera_setup: CameraSetup,
                   queue_frame: Queue=None,
                   event_stop: threading.Event=None):
    """Capture Frames and put them into a queue"""

    # cap_left = cv2.VideoCapture(camera_setup.port_left)
    # cap_right = cv2.VideoCapture(camera_setup.port_right)

    cap_left = cv2.VideoCapture("/dev/video2")
    cap_right = cv2.VideoCapture("/dev/video6")

    docap_left = True
    docap_right = True
    
    # check if are capturing
    if not cap_left.isOpened(): 
        warnings.warn(f"Camera on port {camera_setup.port_left} is not opened! Will not capture.")
        docap_left = False
    if not cap_right.isOpened():
        warnings.warn(f"Camera on port {camera_setup.port_right} is not opened! Will not capture.")
        docap_right = False
    if not docap_right and not docap_left:
        return
    
    while not event_stop.is_set():

        # capture
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        # generate frame image
        frame = FrameContainer(frame_left= frame_left if ret_left else None,
            frame_right=frame_right if ret_right else None,
            timestamp=datetime.datetime.now()
        )

        # queue control
        if queue_frame.full():
            queue_frame.get()
        queue_frame.put(frame)

    cap_right.release()
    cap_left.release()
    return

def get_frame_from_queue(queue_frame: Queue) -> FrameContainer:
    """tries to get a frame from the queue, return None if not possible"""

    try:
        fc = queue_frame.get(timeout=1)
        assert type(fc) == FrameContainer, "Frame Queue did not return a frame container!"
        return fc
    except Exception as e:
        print(f"Caught Exception {e} when trying to get something from the Queue...")
        return None