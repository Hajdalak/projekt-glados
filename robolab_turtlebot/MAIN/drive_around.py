from __future__ import print_function
import math
from robolab_turtlebot import Rate
import safety


def should_stop(stop_requested=None):
    """Return True when the maneuver should stop."""
    if stop_requested is not None:
        return bool(stop_requested())
    return safety.is_stop_requested()


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


def rotate_by(turtle, rate, delta_rad, w=0.25, tol=0.08, stop_requested=None):
    """Rotate robot by the requested relative angle using odometry feedback."""
    # Read the starting heading and compute the target heading.
    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)

    while not turtle.is_shutting_down() and not should_stop(stop_requested):
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)

        # Stop rotating once the heading error is small enough.
        if abs(err) < tol:
            break

        # Use proportional control for angular speed.
        ang = 0.8 * err

        # Clamp angular speed to the configured limit.
        if ang > w:
            ang = w
        elif ang < -w:
            ang = -w

        turtle.cmd_velocity(0.0, ang)
        rate.sleep()

    stop(turtle)
    return not should_stop(stop_requested)


def drive_straight(turtle, rate, dist_m, v=0.18, tol=0.03, stop_requested=None):
    """Drive straight for the requested distance using odometry."""
    # Save the starting odometry position for distance tracking.
    x0, y0, _ = turtle.get_odometry()

    while not turtle.is_shutting_down() and not should_stop(stop_requested):
        x, y, _ = turtle.get_odometry()
        d = math.hypot(x - x0, y - y0)

        # Stop once the traveled distance is close enough to the goal.
        if d >= dist_m - tol:
            break

        turtle.cmd_velocity(v, 0.0)
        rate.sleep()

    stop(turtle)
    return not should_stop(stop_requested)


def maneuver_start_face_ball(turtle, side_m=0.30, v=0.18, w=0.6, stop_requested=None):
    """Execute the predefined drive-around sequence starting while facing the ball."""
    rate = Rate(10)

    # Initial turn that starts the orbit-like path.
    if not rotate_by(turtle, rate, delta_rad=-math.radians(60), w=w, stop_requested=stop_requested):
        return False

    # Repeated straight-and-turn segments around the ball.
    for i in range(5):
        if should_stop(stop_requested):
            stop(turtle)
            return False

        if not drive_straight(turtle, rate, dist_m=side_m, v=v, stop_requested=stop_requested):
            return False

        if not rotate_by(turtle, rate, delta_rad=+math.radians(60), w=w, stop_requested=stop_requested):
            return False

    # Final short segment and last heading correction toward the gate direction.
    if not drive_straight(turtle, rate, dist_m=side_m / 2.5, v=v, stop_requested=stop_requested):
        return False

    if not rotate_by(turtle, rate, delta_rad=-math.radians(90), w=w, stop_requested=stop_requested):
        return False

    stop(turtle)
    return True


def drive_around(turtle, stop_requested=None):
    """Reset odometry and execute the drive-around maneuver."""
    print("Starting the ball drive-around maneuver.")

    # Reset odometry so the maneuver starts from a clean reference frame.
    turtle.reset_odometry()
    wait_rate = Rate(10)

    # Wait until the odometry reset is reflected by the robot state.
    while not should_stop(stop_requested):
        x, y, a = turtle.get_odometry()
        if x == 0 and y == 0 and a == 0:
            break
        wait_rate.sleep()

    if should_stop(stop_requested):
        stop(turtle)
        return

    # Run the full predefined drive-around sequence.
    ok = maneuver_start_face_ball(
        turtle,
        side_m=0.40,
        v=0.18,
        w=0.6,
        stop_requested=stop_requested
    )

    stop(turtle)

    if not ok:
        print("Objeti micku preruseno kvuli stop pozadavku.")

