from __future__ import print_function
import sys
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around as drive_around
import safety

# Create the main robot instance used by the whole program flow.
turtle = Turtlebot(rgb=True, pc=True)
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


def start_drive():
    """Find ball, center on it, and approach it."""
    # Stop early if emergency stop was already requested.
    abort_if_needed()

    # Search for the ball in the scene.
    ball_center = detection.find_ball(turtle, stop_requested=safety.is_stop_requested)
    if ball_center is None:
        abort_if_needed()
        print("Byl aktivovan killswitch, koncim.")
        return False

    # Fine-center the robot on the visible ball.
    centered = movement.recenter_to_ball(turtle, stop_requested=safety.is_stop_requested)
    if centered is None:
        abort_if_needed()
        print("Micek nelze vystredit, koncim.")
        return False

    # Build the object list expected by the drive helper.
    cx, cy = centered
    objects = [(cx, cy)]

    # Approach the ball in multiple controlled steps.
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
    # Stop early if emergency stop was already requested.
    abort_if_needed()

    # Center between the two detected gate poles.
    gate_center = movement.recenter_between_two_objects(
        turtle,
        stop_requested=safety.is_stop_requested
    )
    if gate_center is None:
        abort_if_needed()
        print("Pocatecni centrovani selhalo: brána neni videt. Konec.")
        return False

    # Approach the gate using the wall-centered mode.
    movement.drive_to_ball(
        turtle,
        [],
        target_distance=0.2,
        target_type='gate',
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

    # Idle until the start button is pressed or emergency stop is active.
    while not buttonPressed and not safety.is_stop_requested():
        wait_rate.sleep()

    abort_if_needed()


def main():
    """Robot entrypoint."""
    # Register the global emergency bumper handler once at startup.
    turtle.register_bumper_event_cb(bumper_callback)

    try:
        found_garage = movement.find_garage_by_turning_left(
            turtle,
            stop_requested=safety.is_stop_requested,
        )

        if not found_garage:
            print("Garaz se nepodarilo najit.")
            return

        lost_garage = movement.lose_garage_by_turning_left(
            turtle,
            stop_requested=safety.is_stop_requested,
        )

        if not lost_garage:
            print("Garaz se nepodarilo nechat zmizet ze zaberu.")
            return

        repositioned = movement.rotate_left_10deg_and_drive_30cm(
            turtle,
            stop_requested=safety.is_stop_requested,
        )
        if not repositioned:
            print("Nepodarilo se provest kratky manevr pred startem.")
            return

        # Wait for the manual start signal.
        wait_for_button()

        # Find and approach the ball.
        ok = start_drive()
        if not ok:
            return

        # Perform the drive-around maneuver.
        abort_if_needed()
        drive_around.drive_around(turtle, stop_requested=safety.is_stop_requested)

        # Re-center and drive to the gate.
        abort_if_needed()
        ok = gate_go()
        if not ok:
            return

    finally:
        # Always send a final zero command before exiting.
        turtle.cmd_velocity(0.0, 0.0)


if __name__ == '__main__':
    main()

