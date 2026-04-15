from __future__ import print_function
import math
from robolab_turtlebot import Rate, get_time
import vision
import safety


def should_stop(stop_requested=None):
    """Return True when motion should stop."""
    if stop_requested is not None:
        return bool(stop_requested())
    return safety.is_stop_requested()


def drive_forward_for(turtle, speed, duration, stop_requested=None):
    """Drive forward for a fixed time unless stop is requested."""
    rate = Rate(10)
    t = get_time()

    # Keep sending forward velocity until the requested time elapses.
    while (get_time() - t < duration) and not should_stop(stop_requested):
        turtle.cmd_velocity(linear=speed, angular=0.0)
        rate.sleep()

    # Explicitly stop after the timed segment.
    turtle.cmd_velocity(0.0, 0.0)

    if should_stop(stop_requested):
        print("Jizda prerusena: byl pozadovan stop.")


def recenter_between_two_objects(turtle, image_width=640, tolerance=20, kp=0.005, stop_requested=None):
    """Center robot between two visible objects and return (mid_cx, mid_cy), or None if objects are lost."""
    if should_stop(stop_requested):
        print("Centrovani preskoceno: byl pozadovan stop.")
        return None

    # Detect the two gate poles used for midpoint centering.
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='gate')
    if len(objects) < 2:
        print("Centrovani selhalo: Nevidim obe vezicky v obrazu.")
        return None

    # Compute the initial midpoint between both detected objects.
    cx1, cy1 = float(objects[0][0]), float(objects[0][1])
    cx2, cy2 = float(objects[1][0]), float(objects[1][1])

    mid_cx = (cx1 + cx2) / 2.0
    mid_cy = (cy1 + cy2) / 2.0

    rate = Rate(10)
    while not should_stop(stop_requested):
        center_x = image_width / 2.0
        error = center_x - mid_cx

        # Stop rotating once the midpoint is close enough to image center.
        if abs(error) <= tolerance:
            break

        angular_vel = abs(error) * kp
        max_vel = 0.5
        if angular_vel > max_vel:
            angular_vel = max_vel
        direction = 1 if error > 0 else -1

        # Rotate proportionally toward the midpoint.
        turtle.cmd_velocity(linear=0.0, angular=direction * angular_vel)

        # Refresh detections continuously during centering.
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='gate')
        if len(objects) < 2:
            print("Centrovani preruseno: jeden nebo oba objekty zmizely ze zaberu.")
            turtle.cmd_velocity(0.0, 0.0)
            return None

        # Recompute the midpoint from the newest detections.
        cx1, cy1 = float(objects[0][0]), float(objects[0][1])
        cx2, cy2 = float(objects[1][0]), float(objects[1][1])
        mid_cx = (cx1 + cx2) / 2.0
        mid_cy = (cy1 + cy2) / 2.0

        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)

    if should_stop(stop_requested):
        print("Centrovani preruseno: byl pozadovan stop.")
        return None

    return mid_cx, mid_cy


def recenter_to_ball(turtle, image_width=640, tolerance=20, kp=0.005, stop_requested=None):
    """Center robot on visible ball and return (cx, cy), or None if ball is lost."""
    if should_stop(stop_requested):
        print("Centrovani micku preskoceno: byl pozadovan stop.")
        return None

    # Detect the current ball position in the image.
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
    if len(objects) == 0:
        print("Micek behem jizdy zmizel z obrazu.")
        return None

    cx, cy = float(objects[0][0]), float(objects[0][1])

    rate = Rate(10)
    while not should_stop(stop_requested):
        center_x = image_width / 2.0
        error = center_x - cx

        # Stop rotating once the ball is centered well enough.
        if abs(error) <= tolerance:
            break

        angular_vel = abs(error) * kp
        max_vel = 0.5
        if angular_vel > max_vel:
            angular_vel = max_vel
        direction = 1 if error > 0 else -1

        # Rotate proportionally toward the ball center.
        turtle.cmd_velocity(linear=0.0, angular=direction * angular_vel)

        # Refresh the ball position during centering.
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
        if len(objects) == 0:
            print("Micek behem centrovani zmizel z obrazu.")
            turtle.cmd_velocity(0.0, 0.0)
            return None

        cx, cy = float(objects[0][0]), float(objects[0][1])
        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)

    if should_stop(stop_requested):
        print("Centrovani micku preruseno: byl pozadovan stop.")
        return None

    return cx, cy



def wrap_pi(a):
    """Wrap angle to the interval <-pi, pi>."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rotate_by(turtle, rate, delta_rad, w=0.25, tol=0.05, stop_requested=None):
    """Rotate robot by the requested relative angle using odometry feedback."""
    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)

    while not turtle.is_shutting_down() and not should_stop(stop_requested):
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)

        if abs(err) < tol:
            break

        ang = 0.8 * err
        if ang > w:
            ang = w
        elif ang < -w:
            ang = -w

        turtle.cmd_velocity(0.0, ang)
        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)
    return not should_stop(stop_requested)


def drive_straight(turtle, rate, dist_m, v=0.10, tol=0.02, stop_requested=None):
    """Drive straight for the requested distance using odometry."""
    x0, y0, _ = turtle.get_odometry()

    while not turtle.is_shutting_down() and not should_stop(stop_requested):
        x, y, _ = turtle.get_odometry()
        d = math.hypot(x - x0, y - y0)

        if d >= dist_m - tol:
            break

        turtle.cmd_velocity(v, 0.0)
        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)
    return not should_stop(stop_requested)

def get_depth_z(turtle, cx=320.0, cy=240.0, window_size=7):
    z = vision.get_average_depth(turtle, cx, cy, window_size=window_size)
    if z is None or math.isnan(z):
        return None
    return z

def find_opening_by_depth_scan(
    turtle,
    scan_step_deg=15.0,
    opening_depth_threshold=0.35,
    max_scan_deg=360.0,
    stop_requested=None,
):
    """Scan around the robot and return the center angle of the widest open sector."""
    if should_stop(stop_requested):
        print("Skenovani garaze preskoceno: byl pozadovan stop.")
        return None

    rate = Rate(10)
    step_rad = math.radians(scan_step_deg)
    steps = int(round(max_scan_deg / scan_step_deg))
    open_flags = []
    depth_samples = []

    print("Zacinam hloubkove skenovani vyjezdu z garaze.")

    for _ in range(steps):
        if should_stop(stop_requested):
            turtle.cmd_velocity(0.0, 0.0)
            return None

        z = get_depth_z(turtle, 320.0, 240.0, window_size=7)
        depth_samples.append(z)
        is_open = (z is not None) and (z > opening_depth_threshold)
        open_flags.append(is_open)

        if not rotate_by(
            turtle,
            rate,
            delta_rad=step_rad,
            w=0.50,
            tol=0.07,
            stop_requested=stop_requested,
        ):
            return None

    if not any(open_flags):
        print("Pri skenovani jsem nenasel zadny otevreny smer.")
        return None

    doubled = open_flags + open_flags
    best_start = None
    best_len = 0

    current_start = None
    current_len = 0
    for i, flag in enumerate(doubled):
        if flag and current_start is None:
            current_start = i
            current_len = 1
        elif flag:
            current_len += 1
            if current_len > steps:
                current_start += 1
                current_len = steps
        else:
            if current_start is not None and current_len > best_len:
                best_start = current_start
                best_len = current_len
            current_start = None
            current_len = 0

    if current_start is not None and current_len > best_len:
        best_start = current_start
        best_len = current_len

    mid_idx = (best_start + (best_len - 1) / 2.0) % steps
    delta_rad = wrap_pi(mid_idx * step_rad)

    print(
        "Otevreny sektor nalezen: {} vzorku, stred otoceni {:.1f} stupnu.".format(
            best_len, math.degrees(delta_rad)
        )
    )

    return delta_rad


def center_opening_with_side_depth(
    turtle,
    left_cx=220.0,
    right_cx=420.0,
    cy=240.0,
    diff_tolerance=0.04,
    max_ang=0.20,
    stop_requested=None,
):
    """Fine-center the robot in the opening by balancing left/right depth."""
    if should_stop(stop_requested):
        print("Jemne centrovani otvoru preskoceno: byl pozadovan stop.")
        return False

    rate = Rate(10)

    for _ in range(30):
        if should_stop(stop_requested):
            turtle.cmd_velocity(0.0, 0.0)
            return False

        z_left = get_depth_z(turtle, left_cx, cy, window_size=5)
        z_right = get_depth_z(turtle, right_cx, cy, window_size=5)

        if z_left is None or z_right is None:
            print("Jemne centrovani otvoru selhalo: chybi hloubka vlevo nebo vpravo.")
            turtle.cmd_velocity(0.0, 0.0)
            return False

        err = z_left - z_right
        if abs(err) <= diff_tolerance:
            turtle.cmd_velocity(0.0, 0.0)
            print("Otvor garaze je jemne vycentrovan.")
            return True

        ang = 1.2 * err
        if ang > max_ang:
            ang = max_ang
        elif ang < -max_ang:
            ang = -max_ang

        turtle.cmd_velocity(0.0, ang)
        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)
    print("Jemne centrovani otvoru skoncilo po limitu iteraci.")
    return True


def leave_garage(
    turtle,
    exit_distance=0.30,
    opening_depth_threshold=0.40,
    stop_requested=None,
):
    """Find the garage opening using depth, center in it, and drive out."""
    if should_stop(stop_requested):
        print("Vyjezd z garaze zrusen: byl pozadovan stop.")
        return False

    delta_rad = find_opening_by_depth_scan(
        turtle,
        scan_step_deg=15.0,
        opening_depth_threshold=opening_depth_threshold,
        max_scan_deg=360.0,
        stop_requested=stop_requested,
    )
    if delta_rad is None:
        return False

    rate = Rate(10)
    if not rotate_by(
        turtle,
        rate,
        delta_rad=delta_rad,
        w=0.25,
        tol=0.04,
        stop_requested=stop_requested,
    ):
        return False

    center_opening_with_side_depth(
        turtle,
        left_cx=220.0,
        right_cx=420.0,
        cy=240.0,
        diff_tolerance=0.04,
        max_ang=0.20,
        stop_requested=stop_requested,
    )

    front_z = get_depth_z(turtle, 320.0, 240.0, window_size=7)
    if front_z is not None and front_z < 0.18:
        print("Vyjezd z garaze zrusen: pred robotem je stale mala mezera {:.2f} m.".format(front_z))
        turtle.cmd_velocity(0.0, 0.0)
        return False

    print("Vyjizdim z garaze o {:.2f} m bez dotyku sten.".format(exit_distance))
    ok = drive_straight(
        turtle,
        rate,
        dist_m=exit_distance,
        v=0.10,
        tol=0.02,
        stop_requested=stop_requested,
    )

    turtle.cmd_velocity(0.0, 0.0)

    if not ok:
        print("Vyjezd z garaze byl prerusen.")
        return False

    print("Vyjezd z garaze dokoncen.")
    return True


def approach_and_center(turtle, target_boundary, speed, target_type='ball', stop_requested=None):
    """
    Helper function: measure distance, drive to a target boundary, and re-center.
    Distinguishes between 'ball' (single object) and 'gate' (camera center and two poles).
    """
    avg_point = None

    if target_type == 'ball':
        # Measure depth directly on the currently detected ball centroid.
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
        if len(objects) > 0 and not should_stop(stop_requested):
            cx, cy = float(objects[0][0]), float(objects[0][1])
            print("Ziskavam vzdalenost od micku pro hranici {} m.".format(target_boundary))
            avg_point = vision.get_average_3d_point(turtle, cx, cy)
        elif len(objects) == 0:
            print("Krok priblizeni preskocen: cilovy micek neni videt.")
        else:
            print("Krok priblizeni preskocen: stop pred merenim vzdalenosti.")
    else:  # target_type == 'gate'
        # For the gate, sample depth in the camera center toward the wall.
        center_cx, center_cy = 320.0, 240.0
        if not should_stop(stop_requested):
            print("Ziskavam vzdalenost od zdi pro hranici {} m.".format(target_boundary))
            avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
        else:
            print("Krok priblizeni preskocen: stop pred merenim vzdalenosti.")

    if avg_point is not None:
        current_z = float(avg_point[2])
        distance_to_travel = current_z - target_boundary

        # Move only if the robot is still farther than the requested boundary.
        if distance_to_travel > 0 and not should_stop(stop_requested):
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m k hranici {}m.".format(
                current_z, distance_to_travel, target_boundary
            ))
            duration = distance_to_travel / speed
            drive_forward_for(turtle, speed, duration, stop_requested=stop_requested)

            print("Provadim centrovani na {} m...".format(target_boundary))
            turtle.cmd_velocity(linear=0.0, angular=0.0)

            # Re-center after each approach segment.
            if target_type == 'ball':
                recenter_to_ball(turtle, stop_requested=stop_requested)
            else:
                recenter_between_two_objects(turtle, stop_requested=stop_requested)
    else:
        print("Krok priblizeni preskocen: neni k dispozici platny hloubkovy bod.")


def drive_to_ball(turtle, objects, target_distance=0.1, target_type='ball', stop_requested=None):
    """
    Measure distance to target, compute drive time for drive_forward_for,
    drive forward, and then perform verification and fine correction.
    With target_type, the same flow can also approach a wall ('gate').
    """
    if target_type == 'ball' and len(objects) == 0:
        print("Jizda zrusena: pro pocatecni mereni neni detekovan micek.")
        return

    if should_stop(stop_requested):
        print("Jizda zrusena: stop pred zacatkem sekvence.")
        return

    # Use a single fixed forward speed for all approach phases.
    speed = 0.15
    center_cx, center_cy = 320.0, 240.0

    # First coarse stop around 1 meter from the target.
    approach_and_center(turtle, 1.0, speed, target_type, stop_requested=stop_requested)

    if should_stop(stop_requested):
        turtle.cmd_velocity(0.0, 0.0)
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
        return

    # Second coarse stop around 0.5 meter from the target.
    approach_and_center(turtle, 0.5, speed, target_type, stop_requested=stop_requested)

    if should_stop(stop_requested):
        turtle.cmd_velocity(0.0, 0.0)
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
        return

    # Final depth measurement for the last approach segment.
    avg_point = None
    if target_type == 'ball':
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
        if len(objects) > 0 and not should_stop(stop_requested):
            cx, cy = float(objects[0][0]), float(objects[0][1])
            print("Ziskavam vzdalenost od micku pro finalni dojezd.")
            avg_point = vision.get_average_3d_point(turtle, cx, cy)
        elif len(objects) == 0:
            print("Finalni dojezd preskocen: micek neni videt pro hloubkovou kontrolu.")
        else:
            print("Finalni dojezd preskocen: stop pred hloubkovou kontrolou.")
    else:
        if not should_stop(stop_requested):
            print("Ziskavam vzdalenost od zdi pro finalni dojezd.")
            avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
        else:
            print("Finalni dojezd preskocen: stop pred hloubkovou kontrolou.")

    if avg_point is not None:
        current_z = float(avg_point[2])
        distance_to_travel = current_z - target_distance

        # Execute the main final forward segment.
        if distance_to_travel > 0 and not should_stop(stop_requested):
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))

            duration = distance_to_travel / speed
            drive_forward_for(turtle, speed, duration, stop_requested=stop_requested)

            print("Provadim kontrolu a doladeni...")
            turtle.cmd_velocity(linear=0.0, angular=0.0)

            # Re-sample depth after stopping to measure the remaining error.
            if target_type == 'ball':
                objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
                if len(objects) > 0:
                    cx, cy = float(objects[0][0]), float(objects[0][1])
                    avg_point = vision.get_average_3d_point(turtle, cx, cy)
                else:
                    avg_point = None
            else:
                avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)

            if avg_point is not None:
                current_z = float(avg_point[2])
                error_dist = current_z - target_distance

                # Apply one extra short correction if the robot is still too far.
                if error_dist > 0.02 and not should_stop(stop_requested):
                    print("Doladuji o {:.2f} m.".format(error_dist))
                    duration_fine = error_dist / speed
                    drive_forward_for(turtle, speed, duration_fine, stop_requested=stop_requested)
            else:
                print("Jemne doladeni preskoceno: po kontrolnim zastaveni neni platny hloubkovy bod.")
    else:
        print("Finalni dojezd preskocen: neni k dispozici platny hloubkovy bod.")

    if should_stop(stop_requested):
        turtle.cmd_velocity(0.0, 0.0)
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
        return

    # Re-center once more before the last residual distance check.
    if target_type == 'ball':
        centered = recenter_to_ball(turtle, stop_requested=stop_requested)
    else:
        centered = recenter_between_two_objects(turtle, stop_requested=stop_requested)

    if centered is not None and not should_stop(stop_requested):
        if target_type == 'ball':
            cx, cy = centered
            avg_point = vision.get_average_3d_point(turtle, cx, cy)
        else:
            avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)

        if avg_point is not None:
            current_z = float(avg_point[2])
            final_error_dist = current_z - target_distance
            if final_error_dist > 0.02:
                print("Po vycentrovani dojizdim o {:.2f} m.".format(final_error_dist))
                duration_last = final_error_dist / speed
                drive_forward_for(turtle, speed, duration_last, stop_requested=stop_requested)
        else:
            print("Kontrola po centrovani preskocena: neni platny hloubkovy bod.")
    elif centered is None:
        print("Kontrola po centrovani preskocena: centrovani nevratilo cilovy bod.")
    else:
        print("Kontrola po centrovani preskocena: byl pozadovan stop.")

    # End the full approach sequence with an explicit zero command.
    turtle.cmd_velocity(0.0, 0.0)
    if should_stop(stop_requested):
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
    else:
        print("Sekvence jizdy dokoncena.")


