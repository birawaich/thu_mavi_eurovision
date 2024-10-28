import threading
from queue import Queue

from src.camera_capture import CameraSetup
from src.camera_capture import capture_frames
from src.frame_evaluation import evaluate_captured_frames
from src.frame_container import FrameContainer

def main():
    ### PARAMETERS ###

    keyword = "bottle"
    """Keyword inputed by user = what robot needs to find
    
    IMPORTANT needs to be a YOLO label"""

    ### SCRIPT ###

    # set up search target
    print(f"Searching for '{keyword}'...")
    FrameContainer.USER_KEYWORD = keyword

    # set up camera capturing
    window_name = "Raw Capturing"
    camera_setup = CameraSetup(6,2) #capturing camera port 0 and camera port 1
    queue_frame_caputure = Queue(maxsize=2) #capturing to a queue of length 1 --> always most recent image (sometimes errors if it is empty)
    # set up matching
    queue_frame_distance = Queue(maxsize=5) #queue for frames that have distance information

    event_stop_capture = threading.Event()
    thread_capture = threading.Thread( #setting up thread
        target=capture_frames,
        args=(camera_setup, queue_frame_caputure, event_stop_capture)
    )

    event_stop_evaluate = threading.Event()
    thread_evaluate = threading.Thread(
        target=evaluate_captured_frames,
        args=(queue_frame_caputure,queue_frame_distance,event_stop_evaluate)
    )

    # start threads
    thread_capture.start()
    print("Started camera capturing.")
    thread_evaluate.start()
    print("Started evaluation thread...")

    # now can do stuff with the frames in the distance queue
    # (1) display
    # (2) if have had high matching object with similar distance for a few frames --> call it the final candidat

if __name__ == "__main__":
    main()