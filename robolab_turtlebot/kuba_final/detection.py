from __future__ import print_function
import vision
from robolab_turtlebot import Rate


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

# def center_garage(turtle, stop_requested=None, search_angular_speed=0.3):
#     """Rotate in place until a center of garage is found"""
#     print("Hledam stred garaze...")

#     for ang in (0, 360):
#         turtle.cmd_velocity(linear=0.0, angular=search_angular_speed)


def find_garage_wall(turtle, stop_requested=None, search_angular_speed=0.3):
    """Rotate until the yellow garage wall is detected."""
    print("START GARAGE: zacinam hledat zloutou stenu garaze...")
    rate = Rate(10)

    # Initial one-shot detection before rotation starts.
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')
    print("START GARAGE: aktualne detekovano objektu garaze: {}.".format(len(objects)))

    # Rotate until at least one garage object is visible.
    while len(objects) == 0 and not _should_stop(stop_requested):
        # Debug: robot is still searching for the yellow wall.
        turtle.cmd_velocity(linear=0.0, angular=search_angular_speed)
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')
        rate.sleep()

    # Stop rotation once the search phase ends.
    turtle.cmd_velocity(0.0, 0.0)

    if _should_stop(stop_requested):
        print("START GARAGE: hledani zlute steny preruseno stopem.")
        return False

    if len(objects) == 0:
        print("START GARAGE: zluta stena nebyla nalezena.")
        return False

    print("START GARAGE: nasel jsem stenu garaze. Pocet objektu: {}.".format(len(objects)))
    return True


def rotate_right_until_garage_lost(turtle, stop_requested=None, search_angular_speed=0.3):
    """Rotate right until the yellow garage wall is no longer visible."""
    print("START GARAGE: otacim se doprava, dokud zluta stena nezmizi...")
    rate = Rate(10)

    # Check whether the garage is currently visible.
    objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')
    print("START GARAGE: na zacatku otaceni vidim objektu garaze: {}.".format(len(objects)))

    # If nothing is visible already, there is nothing to clear.
    if len(objects) == 0:
        print("START GARAGE: zluta stena uz neni videt, neni co odtocit.")
        turtle.cmd_velocity(0.0, 0.0)
        return True

    # Rotate right until the garage wall disappears from the image.
    while len(objects) > 0 and not _should_stop(stop_requested):
        # Debug: right turn = negative angular velocity in this project.
        turtle.cmd_velocity(linear=0.0, angular=-abs(search_angular_speed))
        objects = vision.detect_objects_by_hsv_and_area(turtle, target_type='garage')
        rate.sleep()

    # Stop rotation after the wall disappears.
    turtle.cmd_velocity(0.0, 0.0)

    if _should_stop(stop_requested):
        print("START GARAGE: pravotocive otaceni preruseno stopem.")
        return False

    print("START GARAGE: zluta stena uz neni videt. Zastavuji.")
    return True