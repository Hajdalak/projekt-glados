from __future__ import print_function
import math
from robolab_turtlebot import Rate, get_time
import drive_around
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


def find_garage_by_turning_left(turtle, search_angular_speed=0.3, stop_requested=None):
    """Rotate left until the garage is visible and return whether it was found."""
    if should_stop(stop_requested):
        print("Hledani garaze preskoceno: byl pozadovan stop.")
        return False

    print("Hledam garaz otacenim doleva...")
    rate = Rate(10)
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')

    while len(objects) == 0 and not should_stop(stop_requested):
        # Positive angular velocity rotates the robot left.
        turtle.cmd_velocity(linear=0.0, angular=abs(search_angular_speed))
        rate.sleep()
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')

    turtle.cmd_velocity(0.0, 0.0)

    if should_stop(stop_requested):
        print("Hledani garaze preruseno: byl pozadovan stop.")
        return False

    if len(objects) == 0:
        print("Hledani garaze skoncilo bez detekce.")
        return False

    print("Garaz nalezena.")
    return True


def lose_garage_by_turning_left(
    turtle,
    search_angular_speed=0.3,
    consecutive_missing_frames=3,
    stop_requested=None,
):
    """Keep turning left until the garage disappears and return whether it was lost."""
    if should_stop(stop_requested):
        print("Otaceni za garaz preskoceno: byl pozadovan stop.")
        return False

    print("Otacim se doleva, dokud garaz nezmizí ze zaberu...")
    rate = Rate(10)
    # Require several missed detections in a row to avoid stopping on one noisy frame.
    missing_frames = 0
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')

    if len(objects) == 0:
        print("Garaz uz neni v zaberu.")
        return True

    while missing_frames < consecutive_missing_frames and not should_stop(stop_requested):
        turtle.cmd_velocity(linear=0.0, angular=abs(search_angular_speed))
        rate.sleep()
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')

        if len(objects) == 0:
            missing_frames += 1
        else:
            missing_frames = 0

    turtle.cmd_velocity(0.0, 0.0)

    if should_stop(stop_requested):
        print("Otaceni za garaz preruseno: byl pozadovan stop.")
        return False

    if missing_frames < consecutive_missing_frames:
        print("Garaz nezmizela ze zaberu.")
        return False

    print("Garaz zmizela ze zaberu.")
    return True


def rotate_left_10deg_and_drive_30cm(turtle, stop_requested=None):
    """Rotate left by 10 degrees and then drive forward 30 cm."""
    if should_stop(stop_requested):
        print("Kratky manevr preskocen: byl pozadovan stop.")
        return False

    rate = Rate(10)
    print("Otacim robota o 10 stupnu doleva.")
    ok = drive_around.rotate_by(
        turtle,
        rate,
        delta_rad=math.radians(12.0),
        w=0.2,
        stop_requested=stop_requested,
    )
    if not ok:
        return False

    if should_stop(stop_requested):
        turtle.cmd_velocity(0.0, 0.0)
        print("Kratky manevr prerusen po otoceni.")
        return False

    print("Jedu vpred 25 cm.")
    ok = drive_around.drive_straight(
        turtle,
        rate,
        dist_m=0.28,
        v=0.15,
        stop_requested=stop_requested,
    )
    if not ok:
        return False

    turtle.cmd_velocity(0.0, 0.0)
    print("Kratky manevr dokoncen.")
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

