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
    """Return an HSV image from the Turtlebot RGB camera."""
    turtle.wait_for_rgb_image()  # Wait for the first frame.
    rgb_image = turtle.get_rgb_image()
    if rgb_image is None:
        return None
    return cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

def create_hsv_mask(hsv_image, min_h=20, max_h=53, min_s=38, max_s=118, min_v=0, max_v=255):
    """Create a binary mask for pixels inside the given HSV interval."""
    lower = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper = np.array([max_h, max_s, max_v], dtype=np.uint8)
    return cv2.inRange(hsv_image, lower, upper).astype(np.uint8)


def find_centroids(mask, max_area, min_area, axis_tolerance = 0.8):
    """Return centroids (cx, cy) of connected components with area in [min_area, max_area]."""

    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    chosen = []
    for label in range(1, num_labels):  # label 0 is background
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            width = stats[label, cv2.CC_STAT_WIDTH]
            height = stats[label, cv2.CC_STAT_HEIGHT]
            axis_ratio = min(width, height) / float(max(width, height))
            if axis_ratio < axis_tolerance:
                chosen.append(centroids[label])
    return chosen


def detect_objects_by_hsv_and_area(turtle, max_area=700, min_area=300):
    """Return a list of object centroids (cx, cy) detected in the configured HSV range."""
    hsv = get_hsv(turtle)
    if hsv is None:
        return []

    mask = create_hsv_mask(hsv, min_h=18, max_h=55, min_s=35, max_s=121, min_v=0, max_v=255)

    object_centroids = find_centroids(mask, max_area, min_area)
    return object_centroids

def get_average_3d_point(turtle, cx, cy, window_size=5):
    """
    Ziska prumernou 3D souradnici (X, Y, Z) z okoli zadaneho centroidu v Point Cloudu.
    """
    turtle.wait_for_point_cloud()
    pc = turtle.get_point_cloud()
    
    if pc is None:
        return None
        
    # Prevod centroidu na cele cislo pro indexovani pole
    # Numpy pole maji indexovani [radek, sloupec], pricemz radek = cy, sloupec = cx
    col = int(round(cx))
    row = int(round(cy))
    
    # Rozmery point cloudu (obvykle shodne s RGB obrazem)
    h, w, _ = pc.shape
    
    # Vypocet hranic okna (s osetrenim okraju obrazku, abychom neindexovali mimo pole)
    half_w = window_size // 2
    row_start = max(0, row - half_w)
    row_end = min(h, row + half_w + 1)
    col_start = max(0, col - half_w)
    col_end = min(w, col + half_w + 1)
    
    # Vyrez (Region of Interest) z point cloudu kolem centroidu
    window_pc = pc[row_start:row_end, col_start:col_end, :]
    
    # Prevedeme vyrez na 2D pole bodu (N, 3) a spocitame prumer pres sloupce (X, Y, Z)
    # np.nanmean automaticky preskoci NaN hodnoty
    valid_points = window_pc.reshape(-1, 3)
    
    # Kontrola, zda okno neobsahuje pouze NaN hodnoty
    if np.all(np.isnan(valid_points)):
        return None
        
    avg_point = np.nanmean(valid_points, axis=0)
    
    return avg_point
