from __future__ import print_function
import sys
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around as drive_around
import safety
import vision
import math

# Create the main robot instance used by the whole program flow.
turtle = Turtlebot(rgb=True, pc=True)
buttonPressed = False

def escape_to_garage(turtle, escape_dist_m=1.5, stop_requested=None):
    """
    Rotate 360 degrees smoothly, look for the 'garage' using HSV vision,
    find the center of the detection sector, and drive towards it.
    """
    print("Hledam zlutou garaz...")
    rate = Rate(10)
    
    garage_angles = []

    # OCHRANA PROTI PÁDU: Počkáme na inicializaci odometrie
    print("Cekam na inicializaci odometrie...")
    while not safety.is_stop_requested():
        if turtle.get_odometry() is not None:
            break
        rate.sleep()

    # Reset odometrie před plynulým otáčením
    turtle.reset_odometry()
    while not safety.is_stop_requested() and not (stop_requested and stop_requested()):
        x, y, a = turtle.get_odometry()
        if x == 0 and y == 0 and a == 0:
            break
        rate.sleep()

    # Roztočíme robota plynule
    turtle.cmd_velocity(linear=0.0, angular=0.5)
    
    total_rotated = 0.0
    last_angle = 0.0

    # Sweep 360 degrees smoothly
    while total_rotated < 2.0 * math.pi:
        if safety.is_stop_requested() or (stop_requested and stop_requested()):
            print("Hledani garaze preruseno.")
            turtle.cmd_velocity(0.0, 0.0)
            return False

        # Zkontrolujeme, jestli kamera prave ted vidi garaz
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')
        _, _, current_angle = turtle.get_odometry()

        # Hlídáme, kolik jsme se už celkově otočili
        delta = current_angle - last_angle
        while delta > math.pi: 
            delta -= 2.0 * math.pi
        while delta < -math.pi: 
            delta += 2.0 * math.pi
        total_rotated += abs(delta)
        last_angle = current_angle

        # Pokud kamera vidí alespoň jeden objekt splňující parametry garáže
        if len(objects) > 0:
            print("Vidim garaz! Uhel: {:.2f} rad".format(current_angle))
            garage_angles.append(current_angle)

    # Jakmile dokončíme celou otáčku, zastavíme robota
    turtle.cmd_velocity(0.0, 0.0)

    if len(garage_angles) == 0:
        print("Garaz nenalezena (nikde kolem neni zluta barva).")
        return False

    print("Pocitam stred garaze (jednoduchy stred pole)...")
    # Find the middle angle simply by taking the middle element of the array
    mid_index = len(garage_angles) // 2
    target_angle = garage_angles[mid_index]

    print("Stred garaze nalezen na {:.2f} rad. Natacim se...".format(target_angle))

    # Turn robot toward the calculated target angle
    _, _, current_angle = turtle.get_odometry()
    diff = target_angle - current_angle

    # Normalize shortest rotation difference to <-pi, pi>
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi

    if not rotate_by(turtle, rate, diff, w=0.5, stop_requested=stop_requested):
        return False

    print("Smer na garaz nastaven, vyjizdim...")
    # Drive straight out to the garage
    if not drive_straight(turtle, rate, escape_dist_m, v=0.2, stop_requested=stop_requested):
        return False

    print("Uspesne jsem dorazil ke garazi.")
    return True
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

