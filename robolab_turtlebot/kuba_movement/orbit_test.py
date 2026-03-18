from __future__ import print_function
import math

from robolab_turtlebot import Turtlebot, Rate

# ------------------------------
# Helpers: odometry-based motion
# ------------------------------
def wrap_pi(a): # prevedeni uhlu do -pi do pi, protoze a je v rad
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def stop(turtle):
    turtle.cmd_velocity(0.0, 0.0)

def rotate_by(turtle, rate, delta_rad, w=1.0, tol=0.05):
    """Rotate in place by delta_rad (rad) using odometry."""
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

def drive_straight(turtle, rate, dist_m, v=0.5, tol=0.02):
    """Drive straight by dist_m using odometry."""
    x0, y0, _ = turtle.get_odometry() # startovni bod
    while not turtle.is_shutting_down():
        x, y, _ = turtle.get_odometry()
        d = math.hypot(x - x0, y - y0)
        if d >= dist_m - tol:
            break
        turtle.cmd_velocity(v, 0.0)
        rate.sleep()
    stop(turtle)

# ------------------------------
# Orbit maneuver (no vision)
# ------------------------------
def orbit_ball_polygon(turtle,
                       side_m=0.35,         # length of each straight segment
                       n_sides=6,           # 6 = hexagon
                       turn_rad=math.radians(60),  # per-corner turn
                       v=0.12,
                       w=0.5): # max rychlost otaceni
    """
    Assumption for test:
      - Robot is placed facing the ball (ball somewhere in front).
      - Ball should remain on the LEFT side during orbit.
    """

    rate = Rate(10)

    # Save initial heading (approx "towards ball")
    _, _, a_face_ball = turtle.get_odometry()

    # ENTRY: rotate RIGHT ~90° to make ball go to left side (tangent start)
    rotate_by(turtle, rate, delta_rad=-math.radians(90), w=w)

    # ORBIT: regular polygon, turning LEFT each corner
    for _ in range(n_sides):
        drive_straight(turtle, rate, dist_m=side_m, v=v)
        rotate_by(turtle, rate, delta_rad=+turn_rad, w=w)

    # EXIT: face back to garage direction (approx opposite of "face ball")
    target = wrap_pi(a_face_ball + math.pi)
    # Rotate to target absolute heading:
    while not turtle.is_shutting_down():
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)
        if abs(err) < math.radians(4):
            break
        turtle.cmd_velocity(0.0, max(-w, min(w, 1.5 * err)))
        rate.sleep()
    stop(turtle)

# ------------------------------
# Minimal runnable main
# ------------------------------
def main():
    turtle = Turtlebot()
    turtle.reset_odometry()

    # Place robot facing the ball, then run:
    orbit_ball_polygon(turtle, side_m=0.35, n_sides=6)

if __name__ == "__main__":
    main()