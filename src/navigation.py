
from queue import Queue
import threading
import warnings

def navigate(queue_distance: Queue,#destination queue
             queue_captured: Queue,
             event_stop: threading.Event):
    """Function to navigate a robot based on the queues
    = control loop of movement"""

    # control loop based on queues
    while not event_stop.is_set():
        
        if not queue_distance.empty():
            # if have matching in quueue, navigate according to that

            continue

        

    return


def move(distane_m: float,
         rotation_deg: float):
    """Function to move the robot according to distance and rotation"""

    print(f">> MOVE el turtoichrot by {distane_m}m and {rotation_deg}deg")
    
    warnings.warn("Move function is just printing, not moving any robot!")


    return