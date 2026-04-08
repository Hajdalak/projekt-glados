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
    """Register safety callback, find ball, center on it, and approach it."""
    turtle.register_bumper_event_cb(bumper_callback)

    # Find the ball before starting the approach sequence.
    ball_center = detection.find_ball(turtle, stop_requested=is_stop_requested)
    if ball_center is None:
        print("Byl aktivovan killswitch, koncim.")
        return

    # Center the robot on the visible ball.
    centered = movement.recenter_to_ball(turtle)
    if centered is None:
        print("Micek nelze vystredit, koncim.")
        return

    # Build object tuple for the drive-to-ball helper.
    cx, cy = centered
    objects = [(cx, cy)]

    # Drive toward the ball and stop at the target distance.
    movement.drive_to_ball(turtle, objects, 0.3, stop_requested=is_stop_requested)


def gate_go():
    """Center between gate objects and drive toward the gate."""
    # Center the robot between the two visible gate objects.
    gate_center = movement.recenter_between_two_objects(turtle, stop_requested=is_stop_requested)
    if gate_center is None:
        print("Pocatecni centrovani selhalo: brána neni videt. Konec.")
        return

    # Drive toward the gate and stop at the target distance.
    movement.drive_to_ball(turtle, [], target_distance=0.3, target_type='gate', stop_requested=is_stop_requested)


buttonPressed = False


def registerCallback(fun):
    """Set buttonPressed flag when the start button is pressed."""
    global buttonPressed

    if fun.state == 1:
        buttonPressed = True


def wait_for_button():
    """Wait until the user presses the robot button."""
    turtle.register_button_event_cb(registerCallback)
    wait_rate = Rate(10)

    # Keep waiting until the button press is detected.
    while not buttonPressed:
        wait_rate.sleep()


def main():
    """Robot entrypoint: wait for button, approach ball, drive around it, and go to gate."""
    # Wait for the user to start the robot.
    wait_for_button()

    # Find, center, and approach the ball.
    start_drive()

    # Execute the drive-around maneuver.
    drive_around.drive_around(turtle)

    # Center on the gate and approach it.
    gate_go()


if __name__ == '__main__':
    main()
    