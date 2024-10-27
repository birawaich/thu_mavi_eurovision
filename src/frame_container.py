import datetime
import cv2
import numpy as np
import warnings

class FrameContainer:
    """Class to hold captured frames and their corresponding infos"""

    def __init__(self, frame_left, frame_right, timestamp):
        self.frame_left = frame_left #capture frame with left camera
        self.frame_right = frame_right #captured frame with right camera
        self.timestamp: datetime.datetime = timestamp #timestamp of frame

        self.matchings = None #matchings
        return

    def get_raw_info_frame(self) -> cv2.Mat:
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
    
    def is_matching_significant(self) -> bool:
        """Returns whether a container has a significant matching with
        the target keyword according to its matchings"""

        warnings.warn("TODO implement this function. Right now this is just a dummy function.")

        return True
    

    def rate_matching(self, keyword: str):
        """Rates a frame container (if it has object detection) to a matching
        
        Directly modifies the frame container (returns same container)"""

        warnings.warn("TODO implement this function. Right now this is just a dummy function.")

        self.matchings = True #NOTE Dummy output, just to have something different than None

        return
