from __future__ import print_function
import math
from robolab_turtlebot import Turtlebot, Rate

def wrap_pi(a):
    while a > math.pi: 
        a -= 2.0 * math.pi
    while a < -math.pi: 
        a += 2.0 * math.pi
    return a

def stop(turtle):
    turtle.cmd_velocity(0.0, 0.0)

def rotate_by(turtle, rate, delta_rad, w=0.8, tol=0.06):

    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)
    while not turtle.is_shutting_down():
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
    while not turtle.is_shutting_down():
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
                             w=0.8):
    """
    Start:
      - robot ~30 cm od míčku
      - robot míří čelem na míček

    Steps:
      1) rotate 60° RIGHT
      2) repeat 6x: drive side_m, rotate 60° LEFT
    """
    rate = Rate(10)
    turtle.reset_odometry()
    
    # 1) 60° doprava (entry)
    rotate_by(turtle, rate, delta_rad=math.radians(-60), w=w)

    # 2) hexagon: 6x (rovně + 60° doleva)
    for i in range(5):
        drive_straight(turtle, rate, dist_m=side_m, v=v)
        rotate_by(turtle, rate, delta_rad=+math.radians(60), w=w)

    drive_straight(turtle, rate, dist_m=side_m/2, v=v)
    rotate_by(turtle, rate, delta_rad=math.radians(-90), w=w)
    stop(turtle)

def main():
    turtle = Turtlebot()
    turtle.reset_odometry()

    # Robota postav 30 cm před míček čelem k míčku, pak spusť:
    maneuver_start_face_ball(turtle, side_m=0.40, v=0.18, w=0.8)

if __name__ == "__main__":
    main()