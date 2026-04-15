from __future__ import print_function

# Role of this module:
# - Provide vision utilities used by main behavior modules.
# - Detect HSV-based objects.
# - Sample averaged 3D points around image coordinates.

import cv2
import numpy as np

# HSV profile tuned for the ball in current lab lighting.
BALL_MIN_H = 37
BALL_MAX_H = 74
BALL_MIN_S = 98
BALL_MAX_S = 255
BALL_MIN_V = 25
BALL_MAX_V = 255

# HSV profile tuned for gate poles.
GATE_MIN_H = 120
GATE_MAX_H = 159
GATE_MIN_S = 70
GATE_MAX_S = 171
# V is fixed in this lightweight tuner: 0..255
GATE_MIN_V = 0
GATE_MAX_V = 255

# Default geometric filters for ball detections.
BALL_DEFAULT_MIN_AREA = 200
BALL_DEFAULT_MAX_AREA = 8000
BALL_DEFAULT_AXIS_RATIO_MIN = 0.75

# Backward-compatible alias used by legacy call sites/default args.
DEFAULT_AXIS_RATIO_MIN = BALL_DEFAULT_AXIS_RATIO_MIN

# Default geometric filters for gate detections.
GATE_DEFAULT_MIN_AREA = 120
GATE_DEFAULT_MAX_AREA = 20000
GATE_DEFAULT_AXIS_RATIO_MIN = 0.10

# Přidej tyto konstanty nahoru do vision.py k ostatním
GARAGE_MIN_H = 20  # Žlutá barva začíná kolem 20
GARAGE_MAX_H = 40  # a končí kolem 40
GARAGE_MIN_S = 100
GARAGE_MAX_S = 255
GARAGE_MIN_V = 100
GARAGE_MAX_V = 255
GARAGE_DEFAULT_MIN_AREA = 500 # Garáž bude velká, vyfiltrujeme šum
GARAGE_DEFAULT_MAX_AREA = 50000
GARAGE_DEFAULT_AXIS_RATIO_MIN = 0.1

# Přidej tyto konstanty nahoru do vision.py k ostatním
GARAGE_MIN_H = 20  # Žlutá barva začíná kolem 20
GARAGE_MAX_H = 40  # a končí kolem 40
GARAGE_MIN_S = 100
GARAGE_MAX_S = 255
GARAGE_MIN_V = 100
GARAGE_MAX_V = 255
GARAGE_DEFAULT_MIN_AREA = 500
GARAGE_DEFAULT_MAX_AREA = 50000
GARAGE_DEFAULT_AXIS_RATIO_MIN = 0.1
    
    
       

def _resolve_detection_profile(target_type):
    """Return HSV and component-filter defaults for the given target type."""
    if target_type == 'gate':
        # Use the gate-specific HSV and shape constraints.
        return {
            'min_h': GATE_MIN_H,
            'max_h': GATE_MAX_H,
            'min_s': GATE_MIN_S,
            'max_s': GATE_MAX_S,
            'min_v': GATE_MIN_V,
            'max_v': GATE_MAX_V,
            'min_area': GATE_DEFAULT_MIN_AREA,
            'max_area': GATE_DEFAULT_MAX_AREA,
            'axis_tolerance': GATE_DEFAULT_AXIS_RATIO_MIN,
        }
    if target_type == 'garage':
        return {
            'min_h': GARAGE_MIN_H,
            'max_h': GARAGE_MAX_H,
            'min_s': GARAGE_MIN_S,
            'max_s': GARAGE_MAX_S,
            'min_v': GARAGE_MIN_V,
            'max_v': GARAGE_MAX_V,
            'min_area': GARAGE_DEFAULT_MIN_AREA,
            'max_area': GARAGE_DEFAULT_MAX_AREA,
            'axis_tolerance': GARAGE_DEFAULT_AXIS_RATIO_MIN, }
    # Default behavior remains the ball detector.
    if target_type == 'garage':
        return {
            'min_h': GARAGE_MIN_H,
            'max_h': GARAGE_MAX_H,
            'min_s': GARAGE_MIN_S,
            'max_s': GARAGE_MAX_S,
            'min_v': GARAGE_MIN_V,
            'max_v': GARAGE_MAX_V,
            'min_area': GARAGE_DEFAULT_MIN_AREA,
            'max_area': GARAGE_DEFAULT_MAX_AREA,
            'axis_tolerance': GARAGE_DEFAULT_AXIS_RATIO_MIN,
        }
    return {
        'min_h': BALL_MIN_H,
        'max_h': BALL_MAX_H,
        'min_s': BALL_MIN_S,
        'max_s': BALL_MAX_S,
        'min_v': BALL_MIN_V,
        'max_v': BALL_MAX_V,
        'min_area': BALL_DEFAULT_MIN_AREA,
        'max_area': BALL_DEFAULT_MAX_AREA,
        'axis_tolerance': BALL_DEFAULT_AXIS_RATIO_MIN,
    }
    


def get_hsv(turtle):
    """Return an HSV image from the Turtlebot RGB camera."""
    try:
        # Wait for a fresh RGB frame before conversion.
        turtle.wait_for_rgb_image()
        rgb_image = turtle.get_rgb_image()
    except Exception as exc:
        print("Failed to read RGB frame in get_hsv: {}".format(exc))
        return None

    if rgb_image is None:
        print("RGB frame is None in get_hsv.")
        return None

    # Convert the camera frame from BGR to HSV for thresholding.
    return cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)


def create_hsv_mask(hsv_image, min_h, max_h, min_s, max_s, min_v, max_v):
    """Create a binary mask for pixels inside the given HSV interval."""
    lower = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper = np.array([max_h, max_s, max_v], dtype=np.uint8)
    return cv2.inRange(hsv_image, lower, upper).astype(np.uint8)


def find_centroids(mask, max_area, min_area, axis_tolerance=DEFAULT_AXIS_RATIO_MIN):
    """Return centroids (cx, cy) of connected components with area in [min_area, max_area]."""

    # Extract connected components and their statistics from the mask.
    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    chosen = []
    for label in range(1, num_labels):  # label 0 is background
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            width = stats[label, cv2.CC_STAT_WIDTH]
            height = stats[label, cv2.CC_STAT_HEIGHT]
            axis_ratio = min(width, height) / float(max(width, height))

            # Keep only components whose shape is close enough to the target profile.
            if axis_ratio >= axis_tolerance:
                chosen.append(centroids[label])
    return chosen


def detect_objects_by_hsv_and_area(
    turtle,
    max_area=None,
    min_area=None,
    axis_tolerance=None,
    target_type='ball',
):
    """Return a list of object centroids (cx, cy) detected in the configured HSV range."""
    hsv = get_hsv(turtle)
    if hsv is None:
        return []

    # Pick the HSV and shape profile for the requested target type.
    profile = _resolve_detection_profile(target_type)

    if max_area is None:
        max_area = profile['max_area']
    if min_area is None:
        min_area = profile['min_area']
    if axis_tolerance is None:
        axis_tolerance = profile['axis_tolerance']

    # Threshold the image and extract valid connected-component centroids.
    mask = create_hsv_mask(
        hsv,
        profile['min_h'],
        profile['max_h'],
        profile['min_s'],
        profile['max_s'],
        profile['min_v'],
        profile['max_v'],
    )

    object_centroids = find_centroids(mask, max_area, min_area, axis_tolerance=axis_tolerance)
    return object_centroids


def detect_objects_with_debug_frame(
    turtle,
    max_area=None,
    min_area=None,
    axis_tolerance=None,
    target_type='ball',
):
    """Return centroids, annotated RGB frame, and binary mask for debugging."""
    try:
        # Read a fresh RGB frame for debug visualization.
        turtle.wait_for_rgb_image()
        rgb_image = turtle.get_rgb_image()
    except Exception as exc:
        print("Failed to read RGB frame in debug detection: {}".format(exc))
        return [], None, None

    if rgb_image is None:
        print("RGB frame is None in debug detection.")
        return [], None, None

    # Convert the image to HSV and prepare the selected detection profile.
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    profile = _resolve_detection_profile(target_type)

    if max_area is None:
        max_area = profile['max_area']
    if min_area is None:
        min_area = profile['min_area']
    if axis_tolerance is None:
        axis_tolerance = profile['axis_tolerance']

    # Build the mask and extract object centroids.
    mask = create_hsv_mask(
        hsv,
        profile['min_h'],
        profile['max_h'],
        profile['min_s'],
        profile['max_s'],
        profile['min_v'],
        profile['max_v'],
    )
    object_centroids = find_centroids(mask, max_area, min_area, axis_tolerance=axis_tolerance)

    # Draw object markers and text labels into a debug frame.
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

    # Add a summary line with the current number of detections.
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


def get_average_3d_point(turtle, cx, cy, window_size=5):
    """
    Return averaged 3D coordinates (X, Y, Z) around a centroid in the point cloud.
    """
    print("vision.get_average_3d_point started.")

    try:
        # Wait for a fresh point-cloud frame before sampling.
        turtle.wait_for_point_cloud()
        pc = turtle.get_point_cloud()
    except Exception as exc:
        print("Failed to read point cloud: {}".format(exc))
        return None

    print("Point cloud frame received.")

    if pc is None:
        print("Point cloud frame is None.")
        return None

    # Convert centroid coordinates to integer array indices.
    # NumPy uses [row, col], where row = cy and col = cx.
    col = int(round(cx))
    row = int(round(cy))

    # Read the point-cloud dimensions for bounds checking.
    h, w, _ = pc.shape

    # Compute the sampling window around the centroid and clamp it to image limits.
    half_w = window_size // 2
    row_start = max(0, row - half_w)
    row_end = min(h, row + half_w + 1)
    col_start = max(0, col - half_w)
    col_end = min(w, col + half_w + 1)

    # Extract the local 3D neighborhood around the centroid.
    window_pc = pc[row_start:row_end, col_start:col_end, :]

    # Reshape the local window to a flat list of 3D points.
    valid_points = window_pc.reshape(-1, 3)

    # Guard against windows that contain only NaN values.
    if np.all(np.isnan(valid_points)):
        print("All values in the selected window are NaN.")
        return None

    # Compute the average 3D position while ignoring NaN values.
    avg_point = np.nanmean(valid_points, axis=0)

    print(
        "Ball 3D point: X={:.2f}, Y={:.2f}, Z={:.2f} m".format(
            float(avg_point[0]),
            float(avg_point[1]),
            float(avg_point[2]),
        )
    )
    return avg_point

