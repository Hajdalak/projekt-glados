from __future__ import print_function

# Role of this module:
# - Implement movement primitives and centering routines.
# - Handle distance-based approach behavior.
# - Respect emergency stop signals during motion.

from robolab_turtlebot import Turtlebot, Rate, get_time
import vision


killSwitch = 0
turtle = Turtlebot(rgb=True)


def should_stop(stop_requested=None):
    """Return True when motion should stop.

    If stop_requested callback is provided, use its return value.
    Otherwise, fall back to this module's local killSwitch flag.
    """
    if stop_requested is not None:
        return bool(stop_requested())
    return killSwitch != 0


def bumper_callback(msg):
    """Emergency bumper callback: latch killSwitch and stop robot motion."""

    global killSwitch
    killSwitch = msg.state

    turtle.cmd_velocity(linear=0)
    turtle.cmd_velocity(angular=0)
    print('bumper {}'.format(killSwitch))


def drive_forward_for(turtle, speed, duration, stop_requested=None):
    """Drive forward for a fixed time unless stop is requested."""
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < duration) and not should_stop(stop_requested):
        turtle.cmd_velocity(linear=speed)
        rate.sleep()


def recenter_between_two_objects(turtle, image_width=640, tolerance=20, kp=0.005, stop_requested=None):
    """Center robot between two visible objects and return (mid_cx, mid_cy), or None if objects are lost."""
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    if len(objects) < 2:
        print("Nevidim obe vezicky v obrazu.")
        return None

    # Get X and Y coordinates of the first two detected objects.
    cx1, cy1 = float(objects[0][0]), float(objects[0][1])
    cx2, cy2 = float(objects[1][0]), float(objects[1][1])
    
    # Compute the midpoint between the two objects.
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
        
        turtle.cmd_velocity(angular=direction * angular_vel)

        objects = vision.detect_objects_by_hsv_and_area(turtle)
        if len(objects) < 2:
            print("Behem centrovani zmizela jedna nebo obe vezicky z obrazu.")
            turtle.cmd_velocity(angular=0.0)
            return None
            
        cx1, cy1 = float(objects[0][0]), float(objects[0][1])
        cx2, cy2 = float(objects[1][0]), float(objects[1][1])
        mid_cx = (cx1 + cx2) / 2.0
        mid_cy = (cy1 + cy2) / 2.0
        
        rate.sleep()

    turtle.cmd_velocity(angular=0.0)
    return mid_cx, mid_cy


def approach_and_center(turtle, target_boundary, speed, target_type='ball', stop_requested=None):
    """
    Helper function: measure distance, drive to a target boundary, and re-center.
    Distinguishes between 'ball' (single object) and 'gate' (camera center and two poles).
    """
    avg_point = None
    
    if target_type == 'ball':
        objects = vision.detect_objects_by_hsv_and_area(turtle)
        if len(objects) > 0 and not should_stop(stop_requested):
            cx, cy = float(objects[0][0]), float(objects[0][1])
            print("Získávám vzdálenost odmíčku pro hranici {} m.".format(target_boundary))
            avg_point = vision.get_average_3d_point(turtle, cx, cy)
    else:  # target_type == 'gate'
        center_cx, center_cy = 320.0, 240.0
        if not should_stop(stop_requested):
            print("Získávám vzdálenost od zdi pro hranici {} m.".format(target_boundary))
            avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)

    if avg_point is not None:
        current_z = float(avg_point[2])
        distance_to_travel = current_z - target_boundary
        
        if distance_to_travel > 0 and not should_stop(stop_requested):
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m k hranici {}m.".format(current_z, distance_to_travel, target_boundary))
            duration = distance_to_travel / speed
            drive_forward_for(turtle, speed, duration, stop_requested=stop_requested)
            
            print("Provádím centrování na {} m...".format(target_boundary))
            turtle.cmd_velocity(linear=0.0, angular=0.0)
            
            if target_type == 'ball':
                recenter_to_ball(turtle)
            else:
                recenter_between_two_objects(turtle, stop_requested=stop_requested)

def drive_to_ball(turtle, objects, target_distance=0.1, target_type='ball', stop_requested=None):
    """
    Measure distance to target, compute drive time for drive_forward_for,
    drive forward, and then perform verification and fine correction.
    With target_type, the same flow can also approach a wall ('gate').
    """

    if target_type == 'ball' and len(objects) == 0:
        print("Nevidim micek pro mereni vzdalenosti.")
        return
        
    speed = 0.15  # Speed in m/s.
    center_cx, center_cy = 320.0, 240.0

    # === Stop 1: 1 m before target ===
    approach_and_center(turtle, 1.0, speed, target_type, stop_requested=stop_requested)

    # === Stop 2: 0.5 m before target ===
    approach_and_center(turtle, 0.5, speed, target_type, stop_requested=stop_requested)

    # === Final approach to target_distance ===
    avg_point = None
    if target_type == 'ball':
        objects = vision.detect_objects_by_hsv_and_area(turtle)
        if len(objects) > 0 and not should_stop(stop_requested):
            cx, cy = float(objects[0][0]), float(objects[0][1])
            print("Získávám vzdálenost odmíčku pro finální dojezd.")
            avg_point = vision.get_average_3d_point(turtle, cx, cy)
    else:
        if not should_stop(stop_requested):
            print("Získávám vzdálenost od zdi pro finální dojezd.")
            avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
            
    if avg_point is not None:
        current_z = float(avg_point[2])
        distance_to_travel = current_z - target_distance
        
        if distance_to_travel > 0 and not should_stop(stop_requested):
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))

            duration = distance_to_travel / speed
            drive_forward_for(turtle, speed, duration, stop_requested=stop_requested)

            # --- Fine-tuning phase ---
            print("Provadim kontrolu a doladeni...")
            turtle.cmd_velocity(linear=0.0, angular=0.0)

            if target_type == 'ball':
                objects = vision.detect_objects_by_hsv_and_area(turtle)
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

                # If still farther than 2 cm from the goal, move forward a bit.
                if error_dist > 0.02 and not should_stop(stop_requested):
                    print("Doladuji o {:.2f} m.".format(error_dist))
                    duration_fine = error_dist / speed
                    drive_forward_for(turtle, speed, duration_fine, stop_requested=stop_requested)

    # End of function: re-center and, if still far, finish the approach.
    if target_type == 'ball':
        centered = recenter_to_ball(turtle)
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
                        
    turtle.cmd_velocity(linear=0.0)
    turtle.cmd_velocity(angular=0.0)
    print("Dojeto do cile!")
