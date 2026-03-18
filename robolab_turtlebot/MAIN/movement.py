from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import vision


killSwitch = 0
turtle = Turtlebot(rgb=True)

RECENTER_STEP_M = 1
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

def drive_forward_for(turtle, speed, duration):
    
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < duration) and killSwitch == 0:
        turtle.cmd_velocity(linear=speed)
        rate.sleep()


def recenter_to_ball(turtle, image_width=640, tolerance=20, kp=0.005):
    """Center robot on visible ball and return (cx, cy), or None if ball is lost."""
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    if len(objects) == 0:
        print("Micek behem jizdy zmizel z obrazu.")
        return None

    cx, cy = float(objects[0][0]), float(objects[0][1])

    rate = Rate(10)
    while killSwitch == 0:
        center_x = image_width / 2.0
        error = center_x - cx

        if abs(error) <= tolerance:
            break

        angular_vel = abs(error) * kp
        max_vel = 0.5
        if angular_vel > max_vel:
            angular_vel = max_vel
        direction = 1 if error > 0 else -1
        turtle.cmd_velocity(angular=direction * angular_vel)

        objects = vision.detect_objects_by_hsv_and_area(turtle)
        if len(objects) == 0:
            print("Micek behem centrovani zmizel z obrazu.")
            turtle.cmd_velocity(angular=0.0)
            return None
        cx, cy = float(objects[0][0]), float(objects[0][1])
        rate.sleep()

    turtle.cmd_velocity(angular=0.0)
    return cx, cy

def drive_to_ball(turtle, objects, target_distance=0.1):
    """
    Zmeri vzdalenost k micku, dopocita dobu jizdy pro funkci drive_forward_for,
    pojede, a pote provede kontrolni mereni a doladeni pozice.
    """


    if len(objects) == 0:
        print("Nevidim micek pro mereni vzdalenosti.")
        return
        
    cx, cy = float(objects[0][0]), float(objects[0][1])
    print("Získávám vzdálenost odmíčku.")
    avg_point = vision.get_average_3d_point(turtle, cx, cy)
    
    if avg_point is not None:
        current_z = avg_point[2]
        distance_to_travel = current_z - target_distance
        
        if distance_to_travel > 0:
            speed = 0.15  # Rychlost v m/s
            remaining = distance_to_travel

            print("Zmerena hloubka: {:.2f} m. Jedu vpred {:.2f} m.".format(current_z, distance_to_travel))

            while remaining > 0.02 and killSwitch == 0:
                step = min(RECENTER_STEP_M, remaining)
                duration = step / speed
                drive_forward_for(turtle, speed, duration)
                turtle.cmd_velocity(linear=0.0, angular=0.0)

                centered = recenter_to_ball(turtle)
                if centered is None:
                    break

                cx, cy = centered
                avg_point = vision.get_average_3d_point(turtle, cx, cy)
                if avg_point is None:
                    break

                current_z = float(avg_point[2])
                remaining = current_z - target_distance
                print("Aktualni vzdalenost po centrovani: {:.2f} m".format(current_z))
                        
    turtle.cmd_velocity(linear=0.0)
    turtle.cmd_velocity(angular=0.0)
    print("Dojeto do cile!")