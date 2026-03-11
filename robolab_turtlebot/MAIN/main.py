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