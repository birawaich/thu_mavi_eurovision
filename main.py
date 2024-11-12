import threading
from queue import Queue
import time
from pynput import keyboard

from src.camera_capture import CameraSetup
from src.camera_capture import capture_frames
from src.frame_evaluation import evaluate_captured_frames
from src.frame_container import FrameContainer
from src.navigation import navigate

### FLAGS
exit_program = False
"""control whether or not the main loop should be exited"""

def main():
    ### PARAMETERS ###

    keyword = "bottle"
    """Keyword inputed by user = what robot needs to find
    
    IMPORTANT needs to be a YOLO label"""

    ### SCRIPT ###

    #s set up exiting of programm
    global exit_program
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # set up search target
    print(f"Searching for '{keyword}'...")
    FrameContainer.USER_KEYWORD = keyword

    # set up camera capturingq
    camera_setup = CameraSetup(0,2) #capturing camera port 0 and camera port 1
    queue_frame_caputure = Queue(maxsize=2) #capturing to a queue of length 1 --> always most recent image (sometimes errors if it is empty)
    # set up matching
    queue_frame_distance = Queue(maxsize=2) #queue for frames that have distance information

    # set up threads
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

    event_stop_navigation = threading.Event()
    thread_navigation = threading.Thread(
        target = navigate,
        args=(queue_frame_distance,queue_frame_caputure, event_stop_navigation)
    )

    # start threads
    thread_capture.start()
    print("Started camera capturing thread.")
    thread_evaluate.start()
    print("Started evaluation thread.")
    thread_navigation.start()
    print("Started navigation thread.")


    # stop if key is pressed
    while not exit_program:
        # sleep a bit to be a bit more performant
        time.sleep(0.069)

    # stop programm
    print("Stopping Programm...")
    event_stop_navigation.set()
    event_stop_evaluate.set()
    event_stop_capture.set()
    listener.join()
    time.sleep(0.42) #allow threads to propperly finish
    print("Stopped programm. Thanks for choosing EuroVision to command your cleaning robot today~")
    return

def on_press(key):
    """function to handle keypresses (cross platform)"""
    global exit_program
    try:
        if key.char == 'q':  # Check if 'q' is pressed
            print("Exit button pressed, quitting...")
            exit_program = True  # Set the flag to True to exit the main function
            return False  # Stop the listener
    except AttributeError:
        pass  # Ignore other keys


if __name__ == "__main__":
    main()