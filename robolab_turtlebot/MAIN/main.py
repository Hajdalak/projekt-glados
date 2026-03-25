# Role of this module:
# - Wire high-level robot behavior.
# - Register safety callbacks.
# - Coordinate detection and movement modules.

from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around

killSwitch = 0
turtle = Turtlebot(rgb=True, pc=True)
buttonPressed = False

def is_stop_requested():
    """Return True when the global safety stop (killSwitch) is active."""
    return killSwitch != 0


def bumper_callback(msg):
    """Emergency bumper callback: latch killSwitch and stop robot motion."""
    global killSwitch

    killSwitch = msg.state
    turtle.cmd_velocity(linear=0, angular=0)
    print('bumper {}. Do neceho jsem narazil.'.format(killSwitch))

def start_drive():
    turtle.register_bumper_event_cb(bumper_callback)

    ball_center = detection.find_ball(turtle, stop_requested=is_stop_requested)
    if ball_center is None:
        print("Byl aktivovan killswitch, koncim.")
        return

    centered = movement.recenter_to_ball(turtle)
    if centered is None:
        print("Micek nelze vystredit, koncim.")
        return

    cx, cy = centered
    objects = [(cx, cy)]

    movement.drive_to_ball(turtle, objects, 0.3, stop_requested=is_stop_requested)


def gateJed():
    gate_center = movement.recenter_between_two_objects(turtle, stop_requested=is_stop_requested)
    if gate_center is None:
        print("Pocatecni centrovani selhalo: brána neni videt. Konec.")
        return

    movement.drive_to_ball(turtle, [], target_distance=0.3, target_type='gate', stop_requested=is_stop_requested)

buttonPressed = False
def registerCallback(fun):
    global buttonPressed
    if fun.state == 1:
        buttonPressed = True

def cekejNaTlacirko():
    turtle.register_button_event_cb(registerCallback)
    wait_rate = Rate(10)
    while(not buttonPressed):
        wait_rate.sleep()

def main():
    """Robot entrypoint: safety setup, ball find, center and approach."""
    #cekejNaTlacirko()
    
    start_drive()
    drive_around.drive_around(turtle)
    gateJed()
   

if __name__ == '__main__':
    main()