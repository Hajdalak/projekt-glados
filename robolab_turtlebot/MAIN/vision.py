from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import cv2
import numpy as np

def odometry(turtle):
    turtle.wait_for_odometry()
    print('Odometry: {}'.format(turtle.get_odometry()))

def rgbImage(turtle):
    turtle = Turtlebot(rgb=True)
    turtle.wait_for_rgb_image()
    rgb = turtle.get_rgb_image()
    print('RGB shape: {}'.format(rgb.shape))
    print('RGB at 20,20: {}'.format(rgb[20, 20, :]))

def depthImage(turtle):
    turtle = Turtlebot(depth=True)
    turtle.wait_for_depth_image()
    depth = turtle.get_depth_image()
    print('Depth shape: {}'.format(depth.shape))
    print('Depth at 20,20: {}'.format(depth[20, 20]))

def pointCloud(turtle):
    turtle = Turtlebot(pc=True)
    turtle.wait_for_point_cloud()
    pc = turtle.get_point_cloud()
    print('PointCloud shape: {}'.format(pc.shape))
    print('Point at 20,20: {}'.format(pc[20, 20, :]))

def get_hsv(turtle):
    """Vrati HSV obraz z kamery Turtlebotu."""
    turtle.wait_for_rgb_image()  # pocka na prvni frame
    rgb_image = turtle.get_rgb_image()
    if rgb_image is None:
        return None
    return cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

def create_hsv_mask(hsv_image, min_h=18, max_h=55, min_s=35, max_s=121, min_v=0, max_v=255):
    """Create a binary mask for pixels inside the given HSV interval."""
    lower = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper = np.array([max_h, max_s, max_v], dtype=np.uint8)
    return cv2.inRange(hsv_image, lower, upper).astype(np.uint8)


def find_centroids(mask, max_area, min_area):
    """Return connected components filtered by area interval [min_area, max_area]."""

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

    chosen = []
    for label in range(1, num_labels):  # label 0 is background
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            chosen.append(centroids[label])

    return chosen


def detect_objects_by_hsv_and_area(turtle, max_area = 700, min_area = 42):
    """Return objects centeroid in cx, cy"""
    hsv = get_hsv(turtle)
    if hsv is None:
        return []

    mask = create_hsv_mask(hsv, min_h=18, max_h=55, min_s=35, max_s=121, min_v=0, max_v=255)

    objects_centeroid = find_centroids(mask, max_area, min_area)
    return objects_centeroid

