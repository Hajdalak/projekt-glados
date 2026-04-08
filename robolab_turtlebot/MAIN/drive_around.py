from __future__ import print_function
import math
from robolab_turtlebot import Turtlebot, Rate

# Role of this module:
# - Implement odometry-based turning and straight driving primitives.
# - Execute the maneuver for driving around the ball.
# - Respect emergency stop requests during motion.


killSwitch = 0
turtle_ref = None


def wrap_pi(a):
    """Wrap angle to the interval <-pi, pi>."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def stop(turtle):
    """Send zero linear and angular velocity."""
    turtle.cmd_velocity(0.0, 0.0)


def bumper_callback(msg):
    """Emergency bumper callback: latch killSwitch and stop robot motion."""
    global killSwitch
    killSwitch = msg.state
    if turtle_ref is not None:
        turtle_ref.cmd_velocity(0.0, 0.0)
    print('bumper {}'.format(killSwitch))


def rotate_by(turtle, rate, delta_rad, w=0.25, tol=0.08):
    """Rotate robot by the requested relative angle using odometry feedback."""
    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)

    while not turtle.is_shutting_down() and killSwitch == 0:
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)

        if abs(err) < tol:
            break

        ang = 0.8 * err

        if ang > w:
            ang = w
        elif ang < -w:
            ang = -w

        turtle.cmd_velocity(0.0, ang)
        rate.sleep()

    stop(turtle)


def drive_straight(turtle, rate, dist_m, v=0.18, tol=0.03):
    """Drive straight for the requested distance using odometry."""
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
                             side_m=0.30,
                             v=0.18,
                             w=0.6):
    """
    Start:
      - robot is about 30 cm from the ball
      - robot is facing the ball

    Sequence:
      1) initial turn
      2) hexagon-like motion around the ball
      3) final alignment
    """
    rate = Rate(10)

    # === Step 1: Initial turn ===
    # The robot first turns by 60 degrees.
    # This is where the rotation sign can be checked immediately.
    rotate_by(turtle, rate, delta_rad=-math.radians(60), w=w)

    if killSwitch != 0:
        stop(turtle)
        return

    # === Step 2: Motion along the "hexagon" sides ===
    # In each iteration:
    #   - drive straight for one side
    #   - then rotate by 60 degrees
    # If the error appears only here, the issue is more likely
    # in the loop than in the initial turn.
    for i in range(5):
        if killSwitch != 0:
            break

        drive_straight(turtle, rate, dist_m=side_m, v=v)

        if killSwitch != 0:
            break

        rotate_by(turtle, rate, delta_rad=+math.radians(60), w=w)

    if killSwitch != 0:
        stop(turtle)
        return

    # === Step 3: Final alignment ===
    # After finishing the drive-around maneuver, the robot performs
    # one final correction turn.
    # If everything before this works and the issue appears only here,
    # then the bug is likely in this last rotation.
    drive_straight(turtle, rate, dist_m=side_m / 2.5, v=v)
    rotate_by(turtle, rate, delta_rad=-math.radians(90), w=w)

    stop(turtle)


def drive_around(turtle):
    """Reset odometry, register safety callback, and execute the drive-around maneuver."""
    global turtle_ref
    turtle_ref = turtle

    print("Starting the ball drive-around maneuver.")

    # === Step A: Reset odometry before the maneuver ===
    turtle.reset_odometry()
    wait_rate = Rate(10)

    while True:
        x, y, a = turtle.get_odometry()
        if x == 0 and y == 0 and a == 0:
            break
        wait_rate.sleep()

    # === Step B: Register bumper callback for safety ===
    turtle.register_bumper_event_cb(bumper_callback)

    # === Step C: Start the full drive-around maneuver ===
    maneuver_start_face_ball(turtle, side_m=0.40, v=0.18, w=0.6)