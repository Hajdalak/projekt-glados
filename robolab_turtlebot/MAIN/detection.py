from __future__ import print_function

# Role of this module:
# - Provide object-detection helpers used by the main flow.
# - Encapsulate search routines such as finding the ball.

import vision


def _should_stop(stop_requested=None):
    """Return True when a stop callback says motion should stop."""
    if stop_requested is None:
        return False
    return bool(stop_requested())


def count_objects(turtle):
    """Print how many HSV-detected objects are currently visible."""
    objects = vision.detect_objects_by_hsv_and_area(turtle)
    print("{} objects detected.".format(len(objects)))


def show_detected_objects(turtle):
    """Open live preview with object detections."""
    vision.show_detection_stream(turtle)


def find_ball(turtle, stop_requested=None, search_angular_speed=0.3):
    """Rotate in place until a ball is detected, then return its center (cx, cy)."""
    print("Hledam micek...")
    objects = vision.detect_objects_by_hsv_and_area(turtle)

    while len(objects) == 0 and not _should_stop(stop_requested):
        turtle.cmd_velocity(angular=search_angular_speed)
        objects = vision.detect_objects_by_hsv_and_area(turtle)

    if _should_stop(stop_requested):
        turtle.cmd_velocity(linear=0.0, angular=0.0)
        return None

    print("Nasel jsem micek.")
    cx, cy = float(objects[0][0]), float(objects[0][1])
    return cx, cy
