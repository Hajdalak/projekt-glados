from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import vision

killSwitch = 0
turtle = Turtlebot(rgb=True)
# ==========SAFETY==============
# =>
def bumperProc(msg):

    global killSwitch
    killSwitch = msg.state

    turtle.cmd_velocity(linear=0)

    print('bumper {}'.format(killSwitch))
# <=
# ==========SAFETY==============

# ==========POHYB==============
# =>
def jizdaDopreduO(rychlost,cas):
    
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < cas) and killSwitch == 0:
        turtle.cmd_velocity(linear=rychlost)
        rate.sleep()
# <= 
# ==========POHYB==============

# ==========Detect objects==============
# =>
def how_many(turtle):
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    print('{} objects detected.'.format(len(objects)))

# <= 
# ==========Detect objects==============
def main():

    turtle.register_bumper_event_cb(bumperProc)
    
    how_many(turtle)

    

if __name__ == '__main__':
    main()