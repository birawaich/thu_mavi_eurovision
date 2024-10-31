
from queue import Queue
import threading
import warnings
import time

from src.frame_container import FrameContainer

def navigate(queue_distance: Queue,#destination queue
             queue_captured: Queue,
             event_stop: threading.Event):
    """Function to navigate a robot based on the queues
    = control loop of movement"""

    ### PARAMETERS

    time_sleep_noinput = 0.5 #time in s to sleep if there is no input [s]

    distance_env_too_close = 0.3 #distance when enviornment is too close and should rotate [m]
    distance_env_way_too_close = 0.2 #distance when enviornment is way too close and should back up [m]

    angle_stuck = 15 #angle in degrees that the robot rotates if stuck
    
    assert distance_env_way_too_close < distance_env_too_close,\
        "Bro, read the comments before setting parameters!"

    ### CONTROL LOOP

    # control loop based on queues
    while not event_stop.is_set():
        
        if not queue_distance.empty():
            # if have matching in quueue, navigate according to that

            # TODO extract distance of matching
            distance_object = 4.2
            # TODO extract heading from bounding box
            rotation = 6.9
            _move(distane_m=_revise_distance(distance_object),
                 rotation_deg=_revise_rotation(rotation))

            continue

        # if also no images captured, do nothing
        if queue_captured.empty():
            print("Warning! No captured images, will not drive the robot!"+
                   " Will continue doing siesta. Check if the cameras are connected.")
            time.sleep(time_sleep_noinput)
            continue

        # go exploring
        container: FrameContainer = queue_captured.get()

        # TODO extract distance of sourinding objects
        distance_env = 0.42

        if distance_env < distance_env_way_too_close:
            #if way too close: back up
            _move(distane_m= distance_env - distance_env_too_close #move s.t. be in theory back at "too close"
                  , rotation_deg=0)
            continue
        elif distance_env < distance_env_too_close:
            # if too close: rotate
            _move(distane_m=0,
            rotation_deg=angle_stuck)
            continue
        else: #just move forward
            _move(distane_m=_revise_distance(distance_env),
            rotation_deg=0)
            continue   

    return

def _revise_distance(distance_raw: float) -> float:
    """function to revice the distance according to some parameters"""

    ### PARAMETER

    distance_max_move = 1.0 #maximal distance the robot is allowed to move in one step [m]
    distance_safteyfactor = 0.69 #factor \in [0,1] how much to trust the distance to object

    ### CODE
    trusted_distance = distance_raw*distance_safteyfactor
    result = min(distance_max_move,trusted_distance)

    return result

def _revise_rotation(rotation_raw: float) -> float:
    """function to revice the rotation according to some parameters"""

    ### PARAMETER

    rotation_max_move = 30 #maximal rotation the robot is allowed to move in one step [deg]

    ### CODE
    result = min(rotation_max_move,rotation_raw)

    return result


def _move(distane_m: float,
         rotation_deg: float):
    """Function to move the robot according to distance and rotation"""

    print(f">> MOVE el turtoichrot by {distane_m:3}m and {rotation_deg:1}deg")

    time.sleep(0.69)
    """
    Why sleep here?

    The control loop assumes that the move command finishes before being called again.
    If a new command comes before the old one finished and they "stack",
    this will lead to undefined behavior.

    Having a sleep command like this is the simplest way in case there is no feedback.
    """

    warnings.warn("Move function is just printing, not moving any robot!")


    return