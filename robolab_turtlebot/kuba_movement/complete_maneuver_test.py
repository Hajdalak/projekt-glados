from __future__ import print_function

import math
import numpy as np
import cv2

from robolab_turtlebot import Turtlebot, Rate, get_time


# ============================================================
# CONFIG
# ============================================================
W = 640.0
CX_CENTER = W / 2.0
CX_LEFT = 0.25 * W
CX_LEFT_EXIT = 0.08 * W

Z_TARGET = 0.40     # meters, camera -> ball surface
Z_TOL = 0.03

# Speeds
V_APP = 0.10
V_ORB = 0.10
V_MAX = 0.20

W_MAX = 0.8

# Gains
K_X = 1.6 / W
K_Z = 0.8
K_ORB_X = 2.2 / W
K_ORB_Z = 0.7
ORBIT_BIAS_LEFT = 0.15  # makes orbit naturally curve left

# Safety
ORBIT_TIMEOUT = 40.0    # seconds

# HSV range for GREEN (OpenCV Hue: 0..179) - start values, tune if needed
H_MIN, H_MAX = 35, 85
S_MIN, S_MAX = 80, 255
V_MIN, V_MAX_HSV = 40, 255

# Morphology
OPEN_K = 5
CLOSE_K = 7

# Area filter (pixels) - tune depending on distance
MIN_AREA = 200
MAX_AREA = 20000

# Depth sampling window around centroid (odd)
DEPTH_WIN = 11


# ============================================================
# UTILS
# ============================================================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def stop(turtle):
    turtle.cmd_velocity(linear=0.0, angular=0.0)

def median_depth_around(depth, cx, cy, win=11):
    if depth is None:
        return None
    h, w = depth.shape[:2]
    r = win // 2
    x0 = int(clamp(cx - r, 0, w - 1))
    x1 = int(clamp(cx + r, 0, w - 1))
    y0 = int(clamp(cy - r, 0, h - 1))
    y1 = int(clamp(cy + r, 0, h - 1))

    patch = depth[y0:y1+1, x0:x1+1].astype(np.float32).reshape(-1)
    patch = patch[np.isfinite(patch)]
    patch = patch[(patch > 0.05) & (patch < 5.0)]  # drop invalids/outliers
    if patch.size < 10:
        return None
    return float(np.median(patch))

def detect_green_ball_rgb_depth(turtle):
    """
    Returns (found, cx, z) where:
      cx: centroid x in pixels
      z: median depth around centroid (meters, camera->surface)
    """
    rgb = turtle.get_rgb_image()
    depth = turtle.get_depth_image()

    if rgb is None or depth is None:
        return (False, None, None)

    # NOTE: In your repo you used cv2.COLOR_BGR2HSV; image source might be RGB.
    # If mask is bad, switch to cv2.COLOR_RGB2HSV.
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    lower = np.array([H_MIN, S_MIN, V_MIN], dtype=np.uint8)
    upper = np.array([H_MAX, S_MAX, V_MAX_HSV], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    # Morphology cleanup
    if OPEN_K > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_K, OPEN_K))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    if CLOSE_K > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # Find largest contour within area bounds
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return (False, None, None)

    best = None
    best_area = 0.0
    for c in contours:
        a = cv2.contourArea(c)
        if a < MIN_AREA or a > MAX_AREA:
            continue
        if a > best_area:
            best_area = a
            best = c

    if best is None:
        return (False, None, None)

    M = cv2.moments(best)
    if abs(M["m00"]) < 1e-6:
        return (False, None, None)

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]

    z = median_depth_around(depth, cx, cy, win=DEPTH_WIN)
    if z is None:
        return (False, None, None)

    return (True, float(cx), float(z))

def rotate_to_angle(turtle, target_a, rate, tol=0.07, w_max=0.6):
    while not turtle.is_shutting_down():
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target_a - a)
        if abs(err) < tol:
            break
        ang = clamp(err * 1.5, -w_max, w_max)
        turtle.cmd_velocity(linear=0.0, angular=ang)
        rate.sleep()
    stop(turtle)


# ============================================================
# STATE MACHINE
# ============================================================
APPROACH = "APPROACH"
ENTRY = "ENTRY"
ORBIT = "ORBIT"
EXIT = "EXIT"
DONE = "DONE"


killSwitch = 0
turtle = Turtlebot(rgb=True, depth=True)

def bumper_callback(msg):
    global killSwitch
    killSwitch = msg.state
    stop(turtle)
    print("BUMPER:", killSwitch)


def main():
    global killSwitch
    turtle.register_bumper_event_cb(bumper_callback)

    rate = Rate(10)
    turtle.reset_odometry()

    # Wait a bit for streams to start (optional but helps)
    t0 = get_time()
    while get_time() - t0 < 1.0 and not turtle.is_shutting_down():
        rate.sleep()

    state = APPROACH
    face_ball_heading = None
    orbit_start_t = None

    print("Maneuver test started...")

    while not turtle.is_shutting_down() and killSwitch == 0 and state != DONE:

        found, cx, z = detect_green_ball_rgb_depth(turtle)
        if not found:
            # simple: stop if ball not seen
            stop(turtle)
            rate.sleep()
            continue

        if state == APPROACH:
            ex = (cx - CX_CENTER)
            ez = (z - Z_TARGET)

            w_cmd = clamp(K_X * ex, -W_MAX, W_MAX)
            v_cmd = clamp(K_Z * ez, 0.0, V_MAX)
            if ez < 0:
                v_cmd = 0.0

            turtle.cmd_velocity(linear=v_cmd, angular=w_cmd)

            if abs(ez) < Z_TOL and abs(ex) < 0.08 * W:
                _, _, a = turtle.get_odometry()
                face_ball_heading = a
                stop(turtle)
                state = ENTRY
                print("-> ENTRY (face_ball_heading=%.3f, z=%.3f, cx=%.1f)" % (a, z, cx))

        elif state == ENTRY:
            ex = (cx - CX_LEFT)
            ez = (z - Z_TARGET)

            # rotate until ball goes to left quarter; keep distance gently
            w_cmd = clamp(K_ORB_X * ex, -W_MAX, W_MAX)
            v_cmd = clamp(K_ORB_Z * ez, 0.0, 0.10)
            if abs(ez) < Z_TOL:
                v_cmd = 0.0

            turtle.cmd_velocity(linear=v_cmd, angular=w_cmd)

            if abs(ex) < 0.07 * W and abs(ez) < Z_TOL:
                orbit_start_t = get_time()
                stop(turtle)
                state = ORBIT
                print("-> ORBIT")

        elif state == ORBIT:
            if orbit_start_t is None:
                orbit_start_t = get_time()

            if (get_time() - orbit_start_t) > ORBIT_TIMEOUT:
                stop(turtle)
                state = EXIT
                print("ORBIT TIMEOUT -> EXIT")

            ex = (cx - CX_LEFT)
            ez = (z - Z_TARGET)

            # forward + keep ball on left + keep distance
            w_cmd = clamp((K_ORB_X * ex) + ORBIT_BIAS_LEFT, -W_MAX, W_MAX)
            v_cmd = clamp(V_ORB - 0.20 * abs(ez), 0.05, 0.15)

            turtle.cmd_velocity(linear=v_cmd, angular=w_cmd)

            # exit when ball is far left (likely behind us)
            if cx < CX_LEFT_EXIT:
                stop(turtle)
                state = EXIT
                print("-> EXIT (cx=%.1f, z=%.3f)" % (cx, z))

        elif state == EXIT:
            stop(turtle)

            if face_ball_heading is None:
                _, _, a = turtle.get_odometry()
                face_ball_heading = a

            target = wrap_pi(face_ball_heading + math.pi)
            print("Rotating to face garage (target heading %.3f rad)" % target)
            rotate_to_angle(turtle, target, rate, tol=0.07, w_max=0.6)

            state = DONE
            print("DONE: facing garage (approx).")

        rate.sleep()

    stop(turtle)
    print("Stopped (shutdown or bumper).")


if __name__ == "__main__":
    main()