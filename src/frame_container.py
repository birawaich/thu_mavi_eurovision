import datetime
import cv2
import numpy as np
import warnings
from typing import List

class FrameContainer:
    """Class to hold captured frames and their corresponding infos"""

    THRESHOLD_CONFIDENCE = 0.42
    """Confidence threshold s.t. a matching is regarded as significant"""

    USER_KEYWORD = ''
    """User Keyword, i.e. what is looked for"""

    def __init__(self, frame_left, frame_right, timestamp):
        self.frame_left = frame_left #capture frame with left camera
        self.frame_right = frame_right #captured frame with right camera
        self.timestamp: datetime.datetime = timestamp #timestamp of frame

        self.detected_objects: List[DetectedObject] = [] #list of the detected objects
        self.matchings: List[DetectedObject] = [] #list of the detected objects which are matchings

        self.depthmap: cv2.Mat = None #depth map, 2D
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
    
    def get_frame_with_objects_detected(self) -> cv2.Mat:
        """Add detected objects over a raw frame (if any) and return that"""
        base = self.get_raw_info_frame()

        for detected_object  in self.detected_objects:
            cv2.rectangle(base,
                          (detected_object.x1, detected_object.y1),
                          (detected_object.x2, detected_object.y2),
                          (169, 42, 0), 1)
            text = f"{detected_object.label_name} {detected_object.confidence:.2f}"
            cv2.putText(base,
                        text, 
                        (detected_object.x1, detected_object.y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (169, 42, 0), 2)
            
        return base

    def get_frame_with_matching_distances(self) -> cv2.Mat:
        """Add the detected matchings and the distance to them"""
        base = self.get_frame_with_objects_detected()

        for matching in self.matchings:
            cv2.rectangle(base,
                          (matching.x1, matching.y1),
                          (matching.x2, matching.y2),
                          (0, 242, 0), 2)
            text = f"{matching.distance_cm:.2f}cm"
            cv2.putText(base,
                        text, 
                        (matching.x1, matching.y1 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 242, 0), 2)

        return base


    def rate_matching(self):
        """Rates a frame container (if it has object detection) to a matching
        q
        Directly modifies the frame container (returns same container)"""

        if len(self.detected_objects) == 0:
            return

        # check if right label is present
        lables = [detected_object.label_name for detected_object in self.detected_objects]
        if self.USER_KEYWORD not in lables:
            return
        
        # check if these are above confidence interval
        for detected_object in self.detected_objects:
            if detected_object.label_name != self.USER_KEYWORD:
                continue
            if detected_object.confidence < self.THRESHOLD_CONFIDENCE:
                continue
            self.matchings.append(detected_object)

        return
    
    def get_normalized_depth_map(self):
        """
        Returns the depth map s.t. it can be plotted with cv2.imshow
        """
        # Normalize the depth map to the range 0-255
        depth_map_normalized = cv2.normalize(self.depthmap
                                             , None, 0, 255, cv2.NORM_MINMAX)
        
        # Convert to an unsigned 8-bit integer format
        return depth_map_normalized.astype(np.uint8)

class DetectedObject:
    """Class Representing a detected object"""

    width_frame: int = 480
    """Width of the frame of the matching"""

    def __init__(self,
                 label_name: str,
                 label_id: int,
                 confidence: float,
                 x1: int,
                 x2: int,
                 y1: int,
                 y2: int):
        self.label_name = label_name
        self.label_id = label_id
        self.confidence = confidence
        self.x1 = x1 #bounding box data
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.distance_cm = None #distance to object in cm
        return
    
    def get_rotation(self) -> float:
        """return roation (around vertical axis) from camera to object in deg"""
        if self.distance_cm  is None:
            print("Warning: Cannot calucalte rotation if distance is unknown. Returning 0.")
            return 0.0

        offset_to_midpoint_pxl = self.width_frame/2 - (self.x1+self.x2)/2 #pixel
        
        # calculate this distance in cm
        offset_to_midpoint = offset_to_midpoint_pxl * self.distance_cm * 1/1337
        """note: this is roughly linear, with the farhter away, the more cm per pixel

        this formula was guessed late at night by somebody on vacation that does not have
        access to the cameras.
        """

        return np.tan(offset_to_midpoint/self.distance_cm) *360.0/(2*np.pi)