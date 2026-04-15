from __future__ import print_function
import math
from robolab_turtlebot import Rate, get_time
import vision
import safety


def should_stop(stop_requested=None):
    if stop_requested is not None:
        return bool(stop_requested())
    return safety.is_stop_requested()


def drive_forward_for(turtle, speed, duration, stop_requested=None):
    rate = Rate(10)
    t = get_time()
    while (get_time() - t < duration) and not should_stop(stop_requested):
        turtle.cmd_velocity(linear=speed, angular=0.0)
        rate.sleep()
    turtle.cmd_velocity(0.0, 0.0)
    if should_stop(stop_requested):
        print("Jizda prerusena: byl pozadovan stop.")


def recenter_between_two_objects(turtle, image_width=640, tolerance=20, kp=0.005, stop_requested=None):
    if should_stop(stop_requested):
        print("Centrovani preskoceno: byl pozadovan stop.")
        return None

    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='gate')
    if len(objects) < 2:
        print("Centrovani selhalo: Nevidim obe vezicky v obrazu.")
        return None

    cx1, cy1 = float(objects[0][0]), float(objects[0][1])
    cx2, cy2 = float(objects[1][0]), float(objects[1][1])
    mid_cx = (cx1 + cx2) / 2.0
    mid_cy = (cy1 + cy2) / 2.0

    rate = Rate(10)
    while not should_stop(stop_requested):
        center_x = image_width / 2.0
        error = center_x - mid_cx
        if abs(error) <= tolerance:
            break

        angular_vel = abs(error) * kp
        max_vel = 0.5
        if angular_vel > max_vel:
            angular_vel = max_vel
        direction = 1 if error > 0 else -1
        turtle.cmd_velocity(linear=0.0, angular=direction * angular_vel)

        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='gate')
        if len(objects) < 2:
            print("Centrovani preruseno: jeden nebo oba objekty zmizely ze zaberu.")
            turtle.cmd_velocity(0.0, 0.0)
            return None

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
    if should_stop(stop_requested):
        print("Centrovani micku preskoceno: byl pozadovan stop.")
        return None

    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
    if len(objects) == 0:
        print("Micek behem jizdy zmizel z obrazu.")
        return None

    cx, cy = float(objects[0][0]), float(objects[0][1])
    rate = Rate(10)
    while not should_stop(stop_requested):
        center_x = image_width / 2.0
        error = center_x - cx
        if abs(error) <= tolerance:
            break

        angular_vel = abs(error) * kp
        max_vel = 0.5
        if angular_vel > max_vel:
            angular_vel = max_vel
        direction = 1 if error > 0 else -1
        turtle.cmd_velocity(linear=0.0, angular=direction * angular_vel)

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
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rotate_by(turtle, rate, delta_rad, w=0.25, tol=0.05, stop_requested=None):
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


def recenter_to_garage_opening(turtle, image_width=640, tolerance=20, kp=0.004, min_gap_width=80, stop_requested=None):
    """Center robot into the largest non-yellow gap in the image."""
    if should_stop(stop_requested):
        print("Centrovani otvoru preskoceno: byl pozadovan stop.")
        return None

    detection = vision.find_garage_opening_center(turtle, row_ratio=0.60, min_gap_width=min_gap_width)
    if detection is None:
        print("Otvor garaze zatim nevidim.")
        return None

    gap_cx, gap_width, _ = detection
    print("Nasel jsem otvor, sirka {} px, stred {:.1f}.".format(gap_width, gap_cx))

    rate = Rate(10)
    while not should_stop(stop_requested):
        center_x = image_width / 2.0
        error = center_x - gap_cx
        if abs(error) <= tolerance:
            turtle.cmd_velocity(0.0, 0.0)
            print("Otvor je vycentrovany.")
            return gap_cx

        angular_vel = abs(error) * kp
        if angular_vel > 0.35:
            angular_vel = 0.35
        direction = 1 if error > 0 else -1
        turtle.cmd_velocity(linear=0.0, angular=direction * angular_vel)

        detection = vision.find_garage_opening_center(turtle, row_ratio=0.60, min_gap_width=min_gap_width)
        if detection is None:
            turtle.cmd_velocity(0.0, 0.0)
            print("Pri centrovani jsem otvor ztratil.")
            return None

        gap_cx, gap_width, _ = detection
        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)
    return None


def search_and_center_garage_opening(turtle, min_gap_width=80, search_angular_speed=0.35, stop_requested=None):
    """Rotate until a non-yellow gap is visible, then center on it."""
    if should_stop(stop_requested):
        print("Hledani otvoru preskoceno: byl pozadovan stop.")
        return False

    rate = Rate(10)
    print("Hledam vychod z garaze podle zlute barvy.")

    while not turtle.is_shutting_down() and not should_stop(stop_requested):
        detection = vision.find_garage_opening_center(turtle, row_ratio=0.60, min_gap_width=min_gap_width)
        if detection is not None:
            turtle.cmd_velocity(0.0, 0.0)
            print("Otvor je videt, zacinam centrovat.")
            centered = recenter_to_garage_opening(
                turtle,
                image_width=640,
                tolerance=20,
                kp=0.004,
                min_gap_width=min_gap_width,
                stop_requested=stop_requested,
            )
            return centered is not None

        turtle.cmd_velocity(linear=0.0, angular=search_angular_speed)
        rate.sleep()

    turtle.cmd_velocity(0.0, 0.0)
    print("Hledani otvoru preruseno nebo nedokonceno.")
    return False


def leave_garage(turtle, exit_distance=0.30, stop_requested=None):
    """Find non-yellow opening, center into it, and drive out."""
    if should_stop(stop_requested):
        print("Vyjezd z garaze zrusen: byl pozadovan stop.")
        return False

    ok = search_and_center_garage_opening(
        turtle,
        min_gap_width=80,
        search_angular_speed=0.35,
        stop_requested=stop_requested,
    )
    if not ok:
        return False

    rate = Rate(10)
    print("Vyjizdim z garaze o {:.2f} m.".format(exit_distance))
    ok = drive_straight(
        turtle,
        rate,
        dist_m=exit_distance,
        v=0.12,
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
    avg_point = None

    if target_type == 'ball':
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
        if len(objects) > 0 and not should_stop(stop_requested):
            cx, cy = float(objects[0][0]), float(objects[0][1])
            print("Ziskavam vzdalenost od micku pro hranici {} m.".format(target_boundary))
            avg_point = vision.get_average_3d_point(turtle, cx, cy)
        elif len(objects) == 0:
            print("Krok priblizeni preskocen: cilovy micek neni videt.")
        else:
            print("Krok priblizeni preskocen: stop pred merenim vzdalenosti.")
    else:
        center_cx, center_cy = 320.0, 240.0
        if not should_stop(stop_requested):
            print("Ziskavam vzdalenost od zdi pro hranici {} m.".format(target_boundary))
            avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
        else:
            print("Krok priblizeni preskocen: stop pred merenim vzdalenosti.")

    if avg_point is not None:
        current_z = float(avg_point[2])
        distance_to_travel = current_z - target_boundary
        if distance_to_travel > 0 and not should_stop(stop_requested):
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m k hranici {}m.".format(
                current_z, distance_to_travel, target_boundary
            ))
            duration = distance_to_travel / speed
            drive_forward_for(turtle, speed, duration, stop_requested=stop_requested)

            print("Provadim centrovani na {} m...".format(target_boundary))
            turtle.cmd_velocity(linear=0.0, angular=0.0)
            if target_type == 'ball':
                recenter_to_ball(turtle, stop_requested=stop_requested)
            else:
                recenter_between_two_objects(turtle, stop_requested=stop_requested)
    else:
        print("Krok priblizeni preskocen: neni k dispozici platny hloubkovy bod.")


def drive_to_ball(turtle, objects, target_distance=0.1, target_type='ball', stop_requested=None):
    if target_type == 'ball' and len(objects) == 0:
        print("Jizda zrusena: pro pocatecni mereni neni detekovan micek.")
        return

    if should_stop(stop_requested):
        print("Jizda zrusena: stop pred zacatkem sekvence.")
        return

    speed = 0.15
    center_cx, center_cy = 320.0, 240.0

    approach_and_center(turtle, 1.0, speed, target_type, stop_requested=stop_requested)
    if should_stop(stop_requested):
        turtle.cmd_velocity(0.0, 0.0)
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
        return

    approach_and_center(turtle, 0.5, speed, target_type, stop_requested=stop_requested)
    if should_stop(stop_requested):
        turtle.cmd_velocity(0.0, 0.0)
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
        return

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
        if distance_to_travel > 0 and not should_stop(stop_requested):
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))
            duration = distance_to_travel / speed
            drive_forward_for(turtle, speed, duration, stop_requested=stop_requested)

            print("Provadim kontrolu a doladeni...")
            turtle.cmd_velocity(linear=0.0, angular=0.0)

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

    turtle.cmd_velocity(0.0, 0.0)
    if should_stop(stop_requested):
        print("Sekvence jizdy ukoncena kvuli stop pozadavku.")
    else:
        print("Sekvence jizdy dokoncena.")

