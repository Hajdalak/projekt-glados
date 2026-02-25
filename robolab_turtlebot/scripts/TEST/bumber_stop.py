from __future__ import print_function

from robolab_turtlebot import Turtlebot, Rate, get_time


killSwitch = 0

def bumperProc(msg):
    global killSwitch
    killSwitch = msg.state

    print('bumper {}'.format(killSwitch))


def main():

    turtle = Turtlebot()

    turtle.register_bumper_event_cb(bumperProc)
    rate = Rate(10)
    t = get_time()

    while (get_time() - t < 20) and killSwitch == 0:
        turtle.cmd_velocity(linear=1)
        rate.sleep()

    turtle.cmd_velocity(linear=0)
    

if __name__ == '__main__':
    main()