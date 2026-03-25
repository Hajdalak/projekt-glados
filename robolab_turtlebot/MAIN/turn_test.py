from __future__ import print_function
import math
from robolab_turtlebot import Turtlebot, Rate

killSwitch = 0
turtle_ref = None


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def stop(turtle):
    turtle.cmd_velocity(0.0, 0.0)


def bumper_callback(msg):
    global killSwitch
    killSwitch = msg.state
    if turtle_ref is not None:
        turtle_ref.cmd_velocity(0.0, 0.0)
    print("bumper {}".format(killSwitch))


def rotate_by(turtle, rate, delta_rad, w=0.5, tol=0.05):
    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)

    print("START angle:", a0)
    print("TARGET angle:", target)
    print("DELTA rad:", delta_rad)
    print("DELTA deg:", math.degrees(delta_rad))

    while not turtle.is_shutting_down() and killSwitch == 0:
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)

        print("current:", a, " err:", err)

        if abs(err) < tol:
            break

        ang = max(-abs(w), min(abs(w), 1.5 * err))
        turtle.cmd_velocity(0.0, ang)
        rate.sleep()

    stop(turtle)
    print("rotation done")


def test_rotations(turtle):
    rate = Rate(10)

    turtle.reset_odometry()

    print("\nTEST 1: +30 deg")
    rotate_by(turtle, rate, math.radians(30), w=0.4)

    stop(turtle)
    rate.sleep()

    print("\nTEST 2: -30 deg")
    rotate_by(turtle, rate, math.radians(-30), w=0.4)

    stop(turtle)


def main():
    global turtle_ref
    turtle = Turtlebot(rgb=False, depth=False, pc=False)
    turtle_ref = turtle
    turtle.register_bumper_event_cb(bumper_callback)

    test_rotations(turtle)


if __name__ == "__main__":
    main()