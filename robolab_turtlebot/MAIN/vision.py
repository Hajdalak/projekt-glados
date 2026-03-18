from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time
import cv2
import numpy as np

turtle = Turtlebot(rgb=True)

# Tuned for the green ball in current lab lighting.
BALL_MIN_H = 37
BALL_MAX_H = 74
BALL_MIN_S = 98
BALL_MAX_S = 255
BALL_MIN_V = 25
BALL_MAX_V = 255

DEFAULT_MIN_AREA = 200
DEFAULT_MAX_AREA = 6000
DEFAULT_AXIS_RATIO_MIN = 0.75

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

def create_hsv_mask(
    hsv_image,
    min_h=BALL_MIN_H,
    max_h=BALL_MAX_H,
    min_s=BALL_MIN_S,
    max_s=BALL_MAX_S,
    min_v=BALL_MIN_V,
    max_v=BALL_MAX_V,
):
    """Create a binary mask for pixels inside the given HSV interval."""
    lower = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper = np.array([max_h, max_s, max_v], dtype=np.uint8)
    return cv2.inRange(hsv_image, lower, upper).astype(np.uint8)


def find_centroids(mask, max_area, min_area, axis_tolerance=DEFAULT_AXIS_RATIO_MIN):
    """Return centroids (cx, cy) of connected components with area in [min_area, max_area]."""

    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    chosen = []
    for label in range(1, num_labels):  # label 0 is background
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            width = stats[label, cv2.CC_STAT_WIDTH]
            height = stats[label, cv2.CC_STAT_HEIGHT]
            axis_ratio = min(width, height) / float(max(width, height))
            if axis_ratio >= axis_tolerance:
                chosen.append(centroids[label])
    return chosen


def detect_objects_by_hsv_and_area(
    turtle,
    max_area=DEFAULT_MAX_AREA,
    min_area=DEFAULT_MIN_AREA,
    axis_tolerance=DEFAULT_AXIS_RATIO_MIN,
):
    """Return a list of object centroids (cx, cy) detected in the configured HSV range."""
    hsv = get_hsv(turtle)
    if hsv is None:
        return []

    mask = create_hsv_mask(hsv)

    object_centroids = find_centroids(mask, max_area, min_area, axis_tolerance=axis_tolerance)
    return object_centroids


def detect_objects_with_debug_frame(
    turtle,
    max_area=DEFAULT_MAX_AREA,
    min_area=DEFAULT_MIN_AREA,
    axis_tolerance=DEFAULT_AXIS_RATIO_MIN,
):
    """Return centroids, annotated RGB frame and binary mask for debugging."""
    turtle.wait_for_rgb_image()
    rgb_image = turtle.get_rgb_image()
    if rgb_image is None:
        return [], None, None

    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    mask = create_hsv_mask(hsv)
    object_centroids = find_centroids(mask, max_area, min_area, axis_tolerance=axis_tolerance)

    annotated = rgb_image.copy()
    for cx, cy in object_centroids:
        center = (int(round(cx)), int(round(cy)))
        cv2.circle(annotated, center, 12, (0, 255, 0), 2)
        cv2.circle(annotated, center, 3, (0, 0, 255), -1)
        cv2.putText(
            annotated,
            'obj ({}, {})'.format(center[0], center[1]),
            (center[0] + 8, max(18, center[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    cv2.putText(
        annotated,
        'Objects: {}'.format(len(object_centroids)),
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return object_centroids, annotated, mask


def show_detection_stream(
    turtle,
    max_area=DEFAULT_MAX_AREA,
    min_area=DEFAULT_MIN_AREA,
    axis_tolerance=DEFAULT_AXIS_RATIO_MIN,
):
    """Open live windows with robot RGB view and detected objects. Press q to quit."""
    print("Opening detection preview. Press 'q' in image window to quit.")

    rate = Rate(15)
    while True:
        object_centroids, annotated, mask = detect_objects_with_debug_frame(
            turtle,
            max_area=max_area,
            min_area=min_area,
            axis_tolerance=axis_tolerance,
        )

        if annotated is not None:
            cv2.imshow('Robot view - detected objects', annotated)
        if mask is not None:
            cv2.imshow('HSV mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        rate.sleep()

    cv2.destroyAllWindows()

def get_average_3d_point(turtle, cx, cy, window_size=5):
    """
    Ziska prumernou 3D souradnici (X, Y, Z) z okoli zadaneho centroidu v Point Cloudu.
    """
    print("jsem ve vision.get_average_3d_point.")

    
    turtle.wait_for_point_cloud()
    pc = turtle.get_point_cloud()

    print("Proběhly funkce get point cloud.")

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
        print("Vidim Nan.")
        return None
        
    avg_point = np.nanmean(valid_points, axis=0)

    print(
        "3D bod míčku: X={:.2f}, Y={:.2f}, Z={:.2f} m".format(
            float(avg_point[0]),
            float(avg_point[1]),
            float(avg_point[2]),
        )
    )
    return avg_point


def main():

    show_detection_stream(turtle)


if __name__ == '__main__':
    main()