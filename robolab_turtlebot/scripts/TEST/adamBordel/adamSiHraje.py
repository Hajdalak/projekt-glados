from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import cameraHan
import plutoHledaMicek

killSwitch = 0
turtle = Turtlebot()
# ==========SAFETY==============
# =>
def bumperProc(msg):

    global killSwitch
    killSwitch = msg.state

    turtle.cmd_velocity(linear=0)

    print('bumper {}'.format(killSwitch))
# <=
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
# ==========SMYSL==============
# =>
# <=
# ==========SMYSL==============
def main():

    plutoHledaMicek.videt(Turtlebot(rgb=True))
    cameraHan.odometry(turtle)
    cameraHan.rgbImage(turtle)
    cameraHan.depthImage(turtle)
    cameraHan.pointCloud(turtle)

    turtle.register_bumper_event_cb(bumperProc)
    jizdaDopreduO(10,0.1)
    

if __name__ == '__main__':
    main()