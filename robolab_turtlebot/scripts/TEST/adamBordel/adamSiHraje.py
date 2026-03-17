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

# ========== MOVEMENT ==============
# =>
def drive_forward_for(speed, duration):
    
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < duration) and killSwitch == 0:
        turtle.cmd_velocity(linear=speed)
        rate.sleep()

def find_ball(turtle):
    """
    Otaci se na miste, dokud nenajde zeleny micek. 
    Jakmile ho detekuje, zastavi a vycentruje se na nej.
    """
    print("Hledam micek...")
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    
    # Faze 1: Otaceni dokud nic nevidime
    while len(objects) == 0 and killSwitch == 0:
        turtle.cmd_velocity(angular=0.3)
        objects = vision.detect_objects_by_hsv_and_area(turtle)
        

def center_on_object(turtle, cx, image_width=640, tolerance=20, kp=0.005):
    """
    Natoci robota tak, aby byl zadany bod (cx) uprostred obrazu.
    Pouziva proporcionalni rizeni (P-regulator) pro plynule zastaveni.
    Vrati True, pokud je robot vystreden, jinak False.
    """
    center_x = image_width / 2.0
    error = center_x - cx

    if abs(error) > tolerance:
        angular_vel = abs(error) * kp
        
        #maximalni rychlost
        max_vel = 0.5 
        if angular_vel > max_vel:
            angular_vel = max_vel
            
        #smer
        direction = 1 if error > 0 else -1
        

        turtle.cmd_velocity(angular=direction * angular_vel)
        return False
    else:
        turtle.cmd_velocity(angular=0.0)
        return True
    
def drive_to_ball(turtle, target_distance=0.1):
    """
    Zmeri vzdalenost k micku, dopocita dobu jizdy pro funkci drive_forward_for,
    pojede, a pote provede kontrolni mereni a doladeni pozice.
    """
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    if len(objects) == 0:
        print("Nevidim micek pro mereni vzdalenosti.")
        return
        
    cx, cy = objects[0]
    avg_point = vision.get_average_3d_point(turtle, cx, cy)
    
    if avg_point is not None:
        current_z = avg_point[2]
        distance_to_travel = current_z - target_distance
        
        if distance_to_travel > 0:
            speed = 0.15  # Rychlost v m/s
            duration = distance_to_travel / speed
            
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
# <= 
# ========== MOVEMENT ==============

# ========== DETECT OBJECTS ==============
# =>
def count_objects(turtle):
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    print('{} objects detected.'.format(len(objects)))

# <= 
# ========== DETECT OBJECTS ==============
def main():

    turtle.register_bumper_event_cb(bumper_callback)
    
    count_objects(turtle)

    

if __name__ == '__main__':
    main()