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
    print('bumper {}'.format(killSwitch))

def rotate_by(turtle, rate, delta_rad, w=0.6, tol=0.06):

    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)
    while not turtle.is_shutting_down() and killSwitch == 0:
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)
        if abs(err) < tol:
            break
        ang = max(-abs(w), min(abs(w), 1.5 * err))
        turtle.cmd_velocity(0.0, ang)
        rate.sleep()
    stop(turtle)

def drive_straight(turtle, rate, dist_m, v=0.18, tol=0.03):
    x0, y0, _ = turtle.get_odometry()
    while not turtle.is_shutting_down() and killSwitch == 0:
        x, y, _ = turtle.get_odometry()
        d = math.hypot(x - x0, y - y0)
        if d >= dist_m - tol:
            break
        turtle.cmd_velocity(v, 0.0)
        rate.sleep()
    stop(turtle)

def maneuver_start_face_ball(turtle,
                             side_m=0.30,     # délka strany hexagonu (ladíš)
                             v=0.18,
                             w=0.6):
    """
    Start:
      - robot ~30 cm od míčku
      - robot míří čelem na míček

    Steps:
      1) rotate 60° RIGHT
      2) repeat 6x: drive side_m, rotate 60° LEFT
    """
    rate = Rate(10)
    
    # 1) 60° doprava (ENTRY)  -> ZÁPORNĚ
    rotate_by(turtle, rate, delta_rad=+math.radians(60), w=w)
    if killSwitch != 0:
        stop(turtle); return

    # 2) cyklus: rovně + 60° doleva -> KLADNĚ
    for i in range(6):                       # pokud chceš celý hexagon, dej 6
        if killSwitch != 0: break
        drive_straight(turtle, rate, dist_m=side_m, v=v)
        if killSwitch != 0: break
        rotate_by(turtle, rate, delta_rad=+math.radians(60), w=w)

    if killSwitch != 0:
        stop(turtle); return

    # 3) 90° doprava na konci -> ZÁPORNĚ
    rotate_by(turtle, rate, delta_rad=-math.radians(90), w=w)
    stop(turtle)

def drive_around(turtle):
    global turtle_ref
    turtle_ref = turtle
    turtle.reset_odometry()
    turtle.register_bumper_event_cb(bumper_callback)

    # Robota postav 30 cm před míček čelem k míčku, pak spusť:
    maneuver_start_face_ball(turtle, side_m=0.40, v=0.18, w=0.6)
