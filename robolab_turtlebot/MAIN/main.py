from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import vision
import movement

killSwitch = 0
turtle = Turtlebot(rgb=True, pc = True)


def is_stop_requested():
    """Return True when the global safety stop (killSwitch) is active."""
    return killSwitch != 0

# ========== SAFETY ==============
# =>
def bumper_callback(msg):

    global killSwitch
    killSwitch = msg.state

    turtle.cmd_velocity(linear = 0, angular = 0)

    print('bumper {}. Do něčeho jsem narazil.'.format(killSwitch))
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
# <= 
# ========== MOVEMENT ==============

# ========== DETECT OBJECTS ==============
# =>
def count_objects(turtle):
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    print('{} objects detected.'.format(len(objects)))


def show_detected_objects(turtle):
    vision.show_detection_stream(turtle)

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
    
    #If smt stopped the robot
    if killSwitch != 0:
        turtle.cmd_velocity(linear=0.0, angular=0.0)
        return None
    
    print("Našel jsem míček.")

    # objects[0] je numpy pole [cx, cy] — rozbalime na pojmenovane hodnoty
    cx, cy = float(objects[0][0]), float(objects[0][1])
    return cx, cy

# <= 
# ========== DETECT OBJECTS ==============
def main():

    turtle.register_bumper_event_cb(bumper_callback)

    ball_center = find_ball(turtle)
    if ball_center is None:
        print("Byl aktivovan killSwitch, koncim.")
        return

    centered = movement.recenter_to_ball(turtle)
    if centered is None:
        print("Micek nelze vystredit, koncim.")
        return

    cx, cy = centered
    objects = [(cx, cy)]

    movement.drive_to_ball(turtle, objects, 0.3, stop_requested=is_stop_requested)



if __name__ == '__main__':
    main()