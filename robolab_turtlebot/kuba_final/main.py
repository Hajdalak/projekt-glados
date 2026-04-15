from __future__ import print_function
import sys
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around
import safety
import math

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

def start_from_garage():
    """Initial maneuver: find garage wall, rotate away from it, turn 20 deg right and move 30 cm."""
    abort_if_needed()
    print("=== STARTOVNI MANEVR Z GARAZE: ZACATEK ===")

    # Step 1: search until the yellow garage wall is visible.
    found = detection.find_garage_wall(
        turtle,
        stop_requested=safety.is_stop_requested,
        search_angular_speed=0.3
    )
    if not found:
        abort_if_needed()
        print("Startovni manevr selhal: zluta stena garaze nebyla nalezena.")
        return False

    abort_if_needed()

    # Step 2: rotate right until the garage wall disappears from view.
    cleared = detection.rotate_right_until_garage_lost(
        turtle,
        stop_requested=safety.is_stop_requested,
        search_angular_speed=0.3
    )
    if not cleared:
        abort_if_needed()
        print("Startovni manevr selhal: nepodarilo se odtocit od garaze.")
        return False

    abort_if_needed()

    # Step 3: perform an extra 20 degree right turn.
    print("START GARAGE: otacim se jeste o 20 stupnu doprava...")
    rate = Rate(10)
    ok = drive_around.rotate_by(
        turtle,
        rate,
        delta_rad=-math.radians(20),
        w=0.4,
        stop_requested=safety.is_stop_requested
    )
    if not ok:
        abort_if_needed()
        print("Startovni manevr selhal: 20st otoceni bylo preruseno.")
        return False

    abort_if_needed()

    # Step 4: move forward by 30 cm.
    print("START GARAGE: popojizdim vpred o 0.30 m...")
    ok = drive_around.drive_straight(
        turtle,
        rate,
        dist_m=0.30,
        v=0.15,
        stop_requested=safety.is_stop_requested
    )
    if not ok:
        abort_if_needed()
        print("Startovni manevr selhal: popojeti o 30 cm bylo preruseno.")
        return False

    abort_if_needed()
    print("=== STARTOVNI MANEVR Z GARAZE: KONEC ===")
    return True   


def main():
    """Robot entrypoint."""
    turtle.register_bumper_event_cb(bumper_callback)

    try:
        wait_for_button()

        # Initial maneuver to leave the garage area.
        ok = start_from_garage()
        if not ok:
            return

        # Continue with the original ball task.
        ok = start_drive()
        if not ok:
            return

        abort_if_needed()
        drive_around.drive_around(turtle, stop_requested=safety.is_stop_requested)

        abort_if_needed()
        print("Drive around dokonceno. Program konci.")
        return

    finally:
        turtle.cmd_velocity(0.0, 0.0)

        
if __name__ == '__main__':
    main()

