from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import vision


killSwitch = 0
turtle = Turtlebot(rgb=True)

RECENTER_STEP_M = 1


def should_stop(stop_requested=None):
    """Return True when motion should stop.

    If stop_requested callback is provided, use its return value.
    Otherwise, fall back to this module's local killSwitch flag.
    """
    if stop_requested is not None:
        return bool(stop_requested())
    return killSwitch != 0

# ========== SAFETY ==============
# =>
def bumper_callback(msg):

    global killSwitch
    killSwitch = msg.state

    turtle.cmd_velocity(linear=0)
    turtle.cmd_velocity(angular=0)
    print('bumper {}'.format(killSwitch))
# <=
# ========== SAFETY ==============

def drive_forward_for(turtle, speed, duration, stop_requested=None):
    
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < duration) and not should_stop(stop_requested):
        turtle.cmd_velocity(linear=speed)
        rate.sleep()


# Hodnoty pro detekci tyček (věžiček) - nutno naladit
POLE_MIN_H = 100
POLE_MAX_H = 140
POLE_MIN_S = 100
POLE_MAX_S = 255
POLE_MIN_V = 100
POLE_MAX_V = 255

def recenter_between_two_objects(turtle, image_width=640, tolerance=20, kp=0.005, stop_requested=None):
    """Center robot between two visible objects and return (mid_cx, mid_cy), or None if objects are lost."""
    
    # Volání detekce s HSV parametry pro tyčky
    objects = vision.detect_objects_by_hsv_and_area(
        turtle,
        min_h=POLE_MIN_H, max_h=POLE_MAX_H,
        min_s=POLE_MIN_S, max_s=POLE_MAX_S,
        min_v=POLE_MIN_V, max_v=POLE_MAX_V
    )
    
    if len(objects) < 2:
        print("Nevidim obe vezicky v obrazu.")
        return None

    # Získáme X a Y souřadnice prvních dvou detekovaných objektů
    cx1, cy1 = float(objects[0][0]), float(objects[0][1])
    cx2, cy2 = float(objects[1][0]), float(objects[1][1])
    
    # Vypočítáme střed mezi dvěma objekty
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

        # Kontrolní detekce uvnitř cyklu opět s HSV parametry tyček
        objects = vision.detect_objects_by_hsv_and_area(
            turtle,
            min_h=POLE_MIN_H, max_h=POLE_MAX_H,
            min_s=POLE_MIN_S, max_s=POLE_MAX_S,
            min_v=POLE_MIN_V, max_v=POLE_MAX_V
        )
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

def recenter_between_two_objects(turtle, image_width=640, tolerance=20, kp=0.005, stop_requested=None):
    """Center robot between two visible objects and return (mid_cx, mid_cy), or None if objects are lost."""
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    if len(objects) < 2:
        print("Nevidim obe vezicky v obrazu.")
        return None

    # Získáme X a Y souřadnice prvních dvou detekovaných objektů
    cx1, cy1 = float(objects[0][0]), float(objects[0][1])
    cx2, cy2 = float(objects[1][0]), float(objects[1][1])
    
    # Vypočítáme střed mezi dvěma objekty
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
    Pomocna funkce: Zmeri vzdalenost, dojede na zadanou hranici a vycentruje.
    Rozlisuje mezi 'ball' (jeden objekt) a 'gate' (stred kamery a dve vezicky).
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
    Zmeri vzdalenost k micku, dopocita dobu jizdy pro funkci drive_forward_for,
    pojede, a pote provede kontrolni mereni a doladeni pozice.
    Díky parametru target_type umí jet i ke zdi ('gate').
    """

    if target_type == 'ball' and len(objects) == 0:
        print("Nevidim micek pro mereni vzdalenosti.")
        return
        
    speed = 0.15  # Rychlost v m/s
    center_cx, center_cy = 320.0, 240.0

    # === 1. Zastávka: 1 m před objektem ===
    approach_and_center(turtle, 1.0, speed, target_type, stop_requested=stop_requested)

    # === 2. Zastávka: 0.5 m před objektem ===
    approach_and_center(turtle, 0.5, speed, target_type, stop_requested=stop_requested)

    # === Finální dojezd na target_distance ===
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

            # --- Faze doladeni ---
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

                # Pokud je porad vic jak 2 cm daleko od cile, popojede
                if error_dist > 0.02 and not should_stop(stop_requested):
                    print("Doladuji o {:.2f} m.".format(error_dist))
                    duration_fine = error_dist / speed
                    drive_forward_for(turtle, speed, duration_fine, stop_requested=stop_requested)

    # Konec funkce: znovu vycentruj a kdyz je jeste daleko, dojed ke micku.
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
"""
nechat rollback kdyby byl problem s debugem

def approach_and_center(turtle, target_boundary, speed):
  
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    if len(objects) > 0 and killSwitch == 0:
        cx, cy = float(objects[0][0]), float(objects[0][1])
        print("Získávám vzdálenost odmíčku pro hranici {} m.".format(target_boundary))
        avg_point = vision.get_average_3d_point(turtle, cx, cy)
        
        if avg_point is not None:
            current_z = float(avg_point[2])
            distance_to_travel = current_z - target_boundary
            
            if distance_to_travel > 0 and killSwitch == 0:
                print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m k hranici {}m.".format(current_z, distance_to_travel, target_boundary))
                duration = distance_to_travel / speed
                drive_forward_for(turtle, speed, duration)
                
                print("Provádím centrování na {} m...".format(target_boundary))
                turtle.cmd_velocity(linear=0.0, angular=0.0)
                recenter_to_ball(turtle)

def drive_to_ball(turtle, objects, target_distance=0.1):
    
    if len(objects) == 0:
        print("Nevidim micek pro mereni vzdalenosti.")
        return
        
    speed = 0.15  # Rychlost v m/s

    # === 1. Zastávka: 1 m před objektem ===
    approach_and_center(turtle, 1.0, speed)

    # === 2. Zastávka: 0.5 m před objektem ===
    approach_and_center(turtle, 0.5, speed)

    # === Finální dojezd na target_distance ===
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    if len(objects) > 0 and killSwitch == 0:
        cx, cy = float(objects[0][0]), float(objects[0][1])
        print("Získávám vzdálenost odmíčku pro finální dojezd.")
        avg_point = vision.get_average_3d_point(turtle, cx, cy)
        
        if avg_point is not None:
            current_z = float(avg_point[2])
            distance_to_travel = current_z - target_distance
            
            if distance_to_travel > 0 and killSwitch == 0:
                print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))

                duration = distance_to_travel / speed
                drive_forward_for(turtle, speed, duration)

                # --- Faze doladeni ---
                print("Provadim kontrolu a doladeni...")
                turtle.cmd_velocity(linear=0.0, angular=0.0)

                objects = vision.detect_objects_by_hsv_and_area(turtle)
                if len(objects) > 0:
                    cx, cy = float(objects[0][0]), float(objects[0][1])
                    avg_point = vision.get_average_3d_point(turtle, cx, cy)

                    if avg_point is not None:
                        current_z = float(avg_point[2])
                        error_dist = current_z - target_distance

                        # Pokud je porad vic jak 2 cm daleko od cile, popojede
                        if error_dist > 0.02 and killSwitch == 0:
                            print("Doladuji o {:.2f} m.".format(error_dist))
                            duration_fine = error_dist / speed
                            drive_forward_for(turtle, speed, duration_fine)

    # Konec funkce: znovu vycentruj a kdyz je jeste daleko, dojed ke micku.
    centered = recenter_to_ball(turtle)
    if centered is not None and killSwitch == 0:
        cx, cy = centered
        avg_point = vision.get_average_3d_point(turtle, cx, cy)
        if avg_point is not None:
            current_z = float(avg_point[2])
            final_error_dist = current_z - target_distance
            if final_error_dist > 0.02:
                print("Po vycentrovani dojizdim o {:.2f} m.".format(final_error_dist))
                duration_last = final_error_dist / speed
                drive_forward_for(turtle, speed, duration_last)
                        
    turtle.cmd_velocity(linear=0.0)
    turtle.cmd_velocity(angular=0.0)
    print("Dojeto do cile!")

def approach_and_center_between(turtle, target_boundary, speed):
   
    # Středový bod kamery (pro rozlišení 640x480)
    center_cx, center_cy = 320.0, 240.0
    
    if killSwitch == 0:
        print("Získávám vzdálenost od zdi pro hranici {} m.".format(target_boundary))
        avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
        
        if avg_point is not None:
            current_z = float(avg_point[2])
            distance_to_travel = current_z - target_boundary
            
            if distance_to_travel > 0 and killSwitch == 0:
                print("Zmerena hloubka zdi: {:.2f} m. Jedu vpred {:.2f} m k hranici {}m.".format(current_z, distance_to_travel, target_boundary))
                duration = distance_to_travel / speed
                drive_forward_for(turtle, speed, duration)
                
                print("Provádím centrování mezi objekty na {} m...".format(target_boundary))
                turtle.cmd_velocity(linear=0.0, angular=0.0)
                recenter_between_two_objects(turtle)

def drive_between_objects(turtle, target_distance=0.1):
   
        
    speed = 0.15  # Rychlost v m/s

    # === 1. Zastávka: 1 m před zdí ===
    approach_and_center_between(turtle, 1.0, speed)

    # === 2. Zastávka: 0.5 m před zdí ===
    approach_and_center_between(turtle, 0.5, speed)

    # === Finální dojezd na target_distance ke zdi ===
    center_cx, center_cy = 320.0, 240.0
    
    if killSwitch == 0:
        print("Získávám vzdálenost od zdi pro finální dojezd.")
        avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
        
        if avg_point is not None:
            current_z = float(avg_point[2])
            distance_to_travel = current_z - target_distance
            
            if distance_to_travel > 0 and killSwitch == 0:
                print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))

                duration = distance_to_travel / speed
                drive_forward_for(turtle, speed, duration)

                # --- Faze doladeni ---
                print("Provadim kontrolu a doladeni...")
                turtle.cmd_velocity(linear=0.0, angular=0.0)

                avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)

                if avg_point is not None:
                    current_z = float(avg_point[2])
                    error_dist = current_z - target_distance

                    # Pokud je porad vic jak 2 cm daleko od cile, popojede
                    if error_dist > 0.02 and killSwitch == 0:
                        print("Doladuji o {:.2f} m.".format(error_dist))
                        duration_fine = error_dist / speed
                        drive_forward_for(turtle, speed, duration_fine)

    # Konec funkce: znovu vycentruj a kdyz je jeste daleko, dojed ke zdi.
    centered = recenter_between_two_objects(turtle)
    if centered is not None and killSwitch == 0:
        avg_point = vision.get_average_3d_point(turtle, center_cx, center_cy)
        if avg_point is not None:
            current_z = float(avg_point[2])
            final_error_dist = current_z - target_distance
            if final_error_dist > 0.02:
                print("Po vycentrovani dojizdim o {:.2f} m.".format(final_error_dist))
                duration_last = final_error_dist / speed
                drive_forward_for(turtle, speed, duration_last)
                        
    turtle.cmd_velocity(linear=0.0)
    turtle.cmd_velocity(angular=0.0)
    print("Dojeto do cile!")
"""