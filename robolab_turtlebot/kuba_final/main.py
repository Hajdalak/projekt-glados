from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around
import safety

# Create the main robot instance used by the whole program flow.
turtle = Turtlebot(rgb=True, depth=True, pc=True)
buttonPressed = False


def abort_if_needed():
    """Stop the robot and terminate the program when emergency stop is active."""
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


def leave_garage_start():
    """Find the garage opening using depth and drive a short distance out."""
    abort_if_needed()

    ok = movement.leave_garage(
        turtle,
        exit_distance=0.30,
        opening_depth_threshold=0.4,
        stop_requested=safety.is_stop_requested,
    )
    if not ok:
        abort_if_needed()
        print("Vyjezd z garaze selhal, koncim.")
        return False

    abort_if_needed()
    return True


def start_drive():
    """Find ball, center on it, and approach it."""
    abort_if_needed()

    ball_center = detection.find_ball(turtle, stop_requested=safety.is_stop_requested)
    if ball_center is None:
        abort_if_needed()
        print("Micek jsem nenasel, koncim.")
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
        target_distance=0.3,
        stop_requested=safety.is_stop_requested
    )

    abort_if_needed()
    return True


def registerCallback(fun):
    """Set the button flag when the robot start button is pressed."""
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
    """Robot entrypoint for: garage exit -> ball -> drive around -> stop."""
    turtle.register_bumper_event_cb(bumper_callback)

    try:
        wait_for_button()

        ok = leave_garage_start()
        if not ok:
            return

        ok = start_drive()
        if not ok:
            return

        abort_if_needed()
        drive_around.drive_around(turtle, stop_requested=safety.is_stop_requested)
        abort_if_needed()

        print("Program uspesne dokoncen.")

    finally:
        turtle.cmd_velocity(0.0, 0.0)


if __name__ == '__main__':
    main()

