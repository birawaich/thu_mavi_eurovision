import cv2
import threading
from queue import Queue
import datetime
import warnings
import numpy as np

class FrameContainer:
    """Class to hold captured frames and their corresponding infos"""

    def __init__(self, frame_left, frame_right, timestamp):
        self.frame_left = frame_left
        self.frame_right = frame_right
        self.timestamp: datetime.datetime = timestamp
        return

    def get_info_frame(self) -> cv2.Mat:
        """returns a combined frame of the two frames with the meta data in it.
        
        for informative and debug purposes"""

        window_width = 480 * 2  # 960
        window_height = 640
        dim_single_frame = (window_height, window_width // 2)  # (h, w)
        blank_frame = np.zeros((window_width // 2, window_height, 3), dtype=np.uint8)  # Shape: (w, h, , 3) - note H & W is swapped!

        # Get frames and combine
        if self.frame_left is not None:
            frame_left = cv2.resize(self.frame_left, dim_single_frame, interpolation=cv2.INTER_AREA)
        else:
            frame_left = blank_frame
        if self.frame_right is not None:
            frame_right = cv2.resize(self.frame_right, dim_single_frame, interpolation=cv2.INTER_AREA)
        else:
            frame_right = blank_frame
        assert frame_left.shape[0] == frame_right.shape[0],\
              "Frames must have the same height for concatenation."
        frame_combined = cv2.hconcat([frame_left,frame_right])

        # get timestamp and superimpose
        timestamp_txt = self.timestamp.strftime("%H:%M:%S.%f")[:-3]
        # Define the position for the text
        text_position = (10, 30)

        # Draw the black outline by putting text multiple times slightly offset
        outline_color = (0, 0, 0)  # Black color
        text_color = (255, 255, 255)  # White color
        font = cv2.FONT_HERSHEY_SIMPLEX  # Use a simple font; OpenCV does not have a specific mono font
        font_scale = 1
        thickness = 2
        # Draw the outline
        for offset in [-1, 0, 1]:  # Offset to create an outline
            cv2.putText(frame_combined, f"{timestamp_txt}", (text_position[0] + offset, text_position[1] - offset),
                        font, font_scale, outline_color, thickness, cv2.LINE_AA)
        # Draw the actual text on top
        cv2.putText(frame_combined, f"{timestamp_txt}", text_position, font, font_scale, text_color, thickness, cv2.LINE_AA)
        
        return frame_combined

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

    cap_left = cv2.VideoCapture(camera_setup.port_left)
    cap_right = cv2.VideoCapture(camera_setup.port_right)

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