from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import vision


killSwitch = 0
turtle = Turtlebot(rgb=True)
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

def drive_forward_for(speed, duration):
    
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < duration) and killSwitch == 0:
        turtle.cmd_velocity(linear=speed)
        rate.sleep()

def drive_to_ball(turtle, objects, target_distance=0.1):
    """
    Zmeri vzdalenost k micku, dopocita dobu jizdy pro funkci drive_forward_for,
    pojede, a pote provede kontrolni mereni a doladeni pozice.
    """

    # delete later if not needed
    #objects = vision.detect_objects_by_hsv_and_area(turtle)

    if len(objects) == 0:
        print("Nevidim micek pro mereni vzdalenosti.")
        return
        
    cx, cy = objects[0]
    print("Získávám vzdálenost odmíčku.")
    avg_point = vision.get_average_3d_point(turtle, cx, cy)
    
    if avg_point is not None:
        current_z = avg_point[2]
        distance_to_travel = current_z - target_distance
        
        if distance_to_travel > 0:
            speed = 0.15  # Rychlost v m/s
            duration = distance_to_travel // speed
            
            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))
            # Pouziti tvoji existujici funkce
            drive_forward_for(speed, duration)
            
            # --- Faze doladeni ---
            print("Provadim kontrolu a doladeni...")
            turtle.cmd_velocity(linear=0.0, angular=0.0)
            
            # Pro jistotu se znova vycentruje (jizdou se mohl micek vychylit)
            #find_and_center_ball(turtle)  idk prijde mi zbytecny
            
            objects = vision.detect_objects_by_hsv_and_area(turtle)
            if len(objects) > 0:
                cx, cy = objects[0]
                avg_point = vision.get_average_3d_point(turtle, cx, cy)
                
                if avg_point is not None:
                    current_z = avg_point[2]
                    error_dist = current_z - target_distance
                    
                    # Pokud je porad vic jak 2 cm daleko od cile, popojede
                    if error_dist > 0.02:
                        print("Doladuji o {:.2f} m.".format(error_dist))
                        duration_fine = error_dist / speed
                        drive_forward_for(speed, duration_fine)
                        
    turtle.cmd_velocity(linear=0.0)
    print("Dojeto do cile!")