from __future__ import print_function
import sys
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around
import safety

turtle = Turtlebot(rgb=True, pc=True)
buttonPressed = False


def abort_if_needed():
    if safety.is_stop_requested():
        turtle.cmd_velocity(0.0, 0.0)
        print("Program ukoncen kvuli nouzovemu stopu.")
        raise SystemExit(1)


def bumper_callback(msg):
    """Emergency bumper callback: latch stop forever and stop robot immediately."""
    if msg.state == 1:
        safety.request_stop("bumper")
        turtle.cmd_velocity(0.0, 0.0)
        print("bumper 1. Do neceho jsem narazil.")


def start_drive():
    """Find ball, center on it, and approach it."""
    abort_if_needed()

    ball_center = detection.find_ball(turtle, stop_requested=safety.is_stop_requested)
    if ball_center is None:
        abort_if_needed()
        print("Byl aktivovan killswitch, koncim.")
        return False

    centered = movement.recenter_to_ball(turtle, stop_requested=safety.is_stop_requested)
    if centered is None:
        abort_if_needed()
        print("Micek nelze vystredit, koncim.")
        return False

    cx, cy = centered
    objects = [(cx, cy)]

    movement.drive_to_ball(
        turtle,
        objects,
        0.3,
        stop_requested=safety.is_stop_requested
    )

    abort_if_needed()
    return True


def gate_go():
    """Center between gate objects and drive toward the gate."""
    abort_if_needed()

    gate_center = movement.recenter_between_two_objects(
        turtle,
        stop_requested=safety.is_stop_requested
    )
    if gate_center is None:
        abort_if_needed()
        print("Pocatecni centrovani selhalo: brána neni videt. Konec.")
        return False

    movement.drive_to_ball(
        turtle,
        [],
        target_distance=0.3,
        target_type='gate',
        stop_requested=safety.is_stop_requested
    )

    abort_if_needed()
    return True


def registerCallback(fun):
    global buttonPressed
    if fun.state == 1:
        buttonPressed = True


def wait_for_button():
    """Wait until the user presses the robot button."""
    turtle.register_button_event_cb(registerCallback)
    wait_rate = Rate(10)

    while not buttonPressed and not safety.is_stop_requested():
        wait_rate.sleep()

    abort_if_needed()


def main():
    """Robot entrypoint."""
    turtle.register_bumper_event_cb(bumper_callback)

    try:
        wait_for_button()

        ok = start_drive()
        if not ok:
            return

        abort_if_needed()
        drive_around.drive_around(turtle, stop_requested=safety.is_stop_requested)

        abort_if_needed()
        ok = gate_go()
        if not ok:
            return

    finally:
        turtle.cmd_velocity(0.0, 0.0)


if __name__ == '__main__':
    main()
    