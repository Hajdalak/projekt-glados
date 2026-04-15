from __future__ import print_function
import vision


def _should_stop(stop_requested=None):
    """Return True when a provided stop callback requests termination."""
    if stop_requested is None:
        return False
    return bool(stop_requested())


def count_objects(turtle):
    """Print the number of currently detected ball-like objects."""
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
    print("Detekovano objektu: {}.".format(len(objects)))


def show_detected_objects(turtle):
    """Print a one-shot detection count for quick debugging."""
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')
    print("Detected objects: {}.".format(len(objects)))


def find_ball(turtle, stop_requested=None, search_angular_speed=0.3):
    """Rotate in place until a ball is detected, then return its center (cx, cy)."""
    print("Hledam micek...")
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')

    # Rotate in place until at least one ball candidate is detected.
    while len(objects) == 0 and not _should_stop(stop_requested):
        turtle.cmd_velocity(linear=0.0, angular=search_angular_speed)
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='ball')

    # Stop rotation after the search phase ends.
    turtle.cmd_velocity(0.0, 0.0)

    if _should_stop(stop_requested):
        print("Hledani micku preruseno: byl pozadovan stop.")
        return None

    if len(objects) == 0:
        print("Hledani micku skoncilo bez detekce.")
        return None

    # Return the centroid of the first detected ball candidate.
    print("Nasel jsem micek.")
    cx, cy = float(objects[0][0]), float(objects[0][1])
    return cx, cy

def center_garage(turtle, stop_requested=None, search_angular_speed=0.3)