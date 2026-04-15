from __future__ import print_function
import sys
from robolab_turtlebot import Turtlebot, Rate

import detection
import movement
import drive_around
import safety
import math
import vision

turtle = Turtlebot(rgb=True, pc=True)
buttonPressed = False
from drive_around import rotate_by, drive_straight
def get_center_depth(turtle):
    """
    Cista a bezpecna funkce pro ziskani vzdalenosti primo pred robotem 
    z hloubkove kamery (pomoci 2D rezu - laserscanu).
    Vraci vzdalenost v metrech. Pokud je volno/chyba, vraci nekonecno.
    """
    turtle.wait_for_depth_image()
    scan = turtle.get_depth_image()
    
    # Ochrana proti prazdnym datum ze senzoru
    if scan is None or len(scan) == 0:
        return float('inf')
        
    # Vezmeme stredove mereni (robot kouka primo vpred)
    mid_idx = len(scan) // 2
    dist = scan[mid_idx]
    
    # Pokud kamera nevrati cislo (NaN) nebo je vzdalenost nulova, znamena to prazdny prostor
    if math.isnan(dist) or dist <= 0.0:
        return float('inf')
        
    return float(dist)

def escape_from_garage(turtle, garage_radius_m=1.0, escape_dist_m=1.5, stop_requested=None):
    """
    Otaci se plynule o 360 stupnu.
    Pomoci ciste funkce meri hloubku a hleda, kde konci steny garaze (volny prostor).
    Matematicky jednoduse najde stred tohoto volneho prostoru a vyjede ven.
    """
    print("Skenuji okoli pro nalezeni vychodu z garaze...")
    
    open_angles = []

    # OCHRANA: Pockame, az najedou senzory odometrie
    while not safety.is_stop_requested():
        if turtle.get_odometry() is not None:
            break

    # Reset odometrie pred plynulym otacenim
    turtle.reset_odometry()
    while not safety.is_stop_requested() and not (stop_requested and stop_requested()):
        x, y, a = turtle.get_odometry()
        if x == 0 and y == 0 and a == 0:
            break

    # Roztocime robota plynule
    turtle.cmd_velocity(linear=0.0, angular=0.5)
    
    total_rotated = 0.0
    last_angle = 0.0

    # Plynula rotace 360 stupnu
    while total_rotated < 2.0 * math.pi:
        if safety.is_stop_requested() or (stop_requested and stop_requested()):
            print("Manevr prerusen.")
            turtle.cmd_velocity(0.0, 0.0)
            return False

        # Zjistime vzdalenost z hloubkove kamery cistou funkci
        dist = get_center_depth(turtle)
        _, _, current_angle = turtle.get_odometry()

        # Hlidame celkove natoceni
        delta = current_angle - last_angle
        while delta > math.pi: delta -= 2.0 * math.pi
        while delta < -math.pi: delta += 2.0 * math.pi
        total_rotated += abs(delta)
        last_angle = current_angle

        # Pokud je vzdalenost vetsi nez steny garaze (např. > 1 metr), jsme celem k vychodu
        if dist > garage_radius_m:
            open_angles.append(current_angle)

    # Dokoncili jsme otacku, zastavime
    turtle.cmd_velocity(0.0, 0.0)

    if len(open_angles) == 0:
        print("Vychod nenalezen (vse kolem je blizko jako zed).")
        return False

    print("Pocitam stred vychodu (prostredkem pole)...")
    
    # MATEMATICKY JEDNODUSI RESENI: Vezmeme uhel presne uprostred z tech, kde bylo volno
    mid_index = len(open_angles) // 2
    target_angle = open_angles[mid_index]

    print("Stred vychodu nalezen na {:.2f} rad. Natacim se...".format(target_angle))

    # Dotocime se na tento uhel
    _, _, current_angle = turtle.get_odometry()
    diff = target_angle - current_angle

    # Normalizace uhlu na interval <-pi, pi>
    while diff > math.pi: diff -= 2.0 * math.pi
    while diff < -math.pi: diff += 2.0 * math.pi

    if not rotate_by(turtle, 10, diff, w=0.5, stop_requested=stop_requested):
        return False

    print("Otoceno k vychodu. Vyjizdim z garaze...")
    # Vyjedeme ven (tvoje funkce z drive_around.py)
    if not drive_straight(turtle, 10, escape_dist_m, v=0.2, stop_requested=stop_requested):
        return False

    print("Uspesne jsem vyjel z garaze.")
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
        escape_from_garage(turtle)

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

