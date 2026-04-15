from __future__ import print_function

import cv2
import numpy as np

# Ball HSV profile.
BALL_MIN_H = 37
BALL_MAX_H = 74
BALL_MIN_S = 98
BALL_MAX_S = 255
BALL_MIN_V = 25
BALL_MAX_V = 255

# Gate poles HSV profile.
GATE_MIN_H = 120
GATE_MAX_H = 159
GATE_MIN_S = 70
GATE_MAX_S = 171
GATE_MIN_V = 0
GATE_MAX_V = 255

# Yellow garage HSV profile.
# These values are only a starting point and may need tuning in lab lighting.
GARAGE_MIN_H = 15
GARAGE_MAX_H = 45
GARAGE_MIN_S = 70
GARAGE_MAX_S = 255
GARAGE_MIN_V = 80
GARAGE_MAX_V = 255

BALL_DEFAULT_MIN_AREA = 200
BALL_DEFAULT_MAX_AREA = 8000
BALL_DEFAULT_AXIS_RATIO_MIN = 0.75
DEFAULT_AXIS_RATIO_MIN = BALL_DEFAULT_AXIS_RATIO_MIN

GATE_DEFAULT_MIN_AREA = 120
GATE_DEFAULT_MAX_AREA = 20000
GATE_DEFAULT_AXIS_RATIO_MIN = 0.10


def _resolve_detection_profile(target_type):
    if target_type == 'gate':
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


def get_bgr(turtle):
    try:
        image = turtle.get_rgb_image()
    except Exception as exc:
        print("Failed to read RGB frame: {}".format(exc))
        return None

    if image is None:
        print("RGB frame is None.")
        return None

    return image


def get_hsv(turtle):
    rgb_image = get_bgr(turtle)
    if rgb_image is None:
        return None
    return cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)


def create_hsv_mask(hsv_image, min_h, max_h, min_s, max_s, min_v, max_v):
    lower = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper = np.array([max_h, max_s, max_v], dtype=np.uint8)
    return cv2.inRange(hsv_image, lower, upper).astype(np.uint8)


def find_centroids(mask, max_area, min_area, axis_tolerance=DEFAULT_AXIS_RATIO_MIN):
    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    chosen = []
    for label in range(1, num_labels):
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
    max_area=None,
    min_area=None,
    axis_tolerance=None,
    target_type='ball',
):
    hsv = get_hsv(turtle)
    if hsv is None:
        return []

    profile = _resolve_detection_profile(target_type)

    if max_area is None:
        max_area = profile['max_area']
    if min_area is None:
        min_area = profile['min_area']
    if axis_tolerance is None:
        axis_tolerance = profile['axis_tolerance']

    mask = create_hsv_mask(
        hsv,
        profile['min_h'],
        profile['max_h'],
        profile['min_s'],
        profile['max_s'],
        profile['min_v'],
        profile['max_v'],
    )

    return find_centroids(mask, max_area, min_area, axis_tolerance=axis_tolerance)


def get_average_3d_point(turtle, cx, cy, window_size=5):
    print("vision.get_average_3d_point started.")

    try:
        pc = turtle.get_point_cloud()
    except Exception as exc:
        print("Failed to read point cloud: {}".format(exc))
        return None

    print("Point cloud frame received.")

    if pc is None:
        print("Point cloud frame is None.")
        return None

    col = int(round(cx))
    row = int(round(cy))
    h, w, _ = pc.shape

    half_w = window_size // 2
    row_start = max(0, row - half_w)
    row_end = min(h, row + half_w + 1)
    col_start = max(0, col - half_w)
    col_end = min(w, col + half_w + 1)

    window_pc = pc[row_start:row_end, col_start:col_end, :]
    valid_points = window_pc.reshape(-1, 3)

    if np.all(np.isnan(valid_points)):
        print("All values in the selected window are NaN.")
        return None

    avg_point = np.nanmean(valid_points, axis=0)

    print(
        "Ball 3D point: X={:.2f}, Y={:.2f}, Z={:.2f} m".format(
            float(avg_point[0]),
            float(avg_point[1]),
            float(avg_point[2]),
        )
    )
    return avg_point


def get_average_depth(turtle, cx, cy, window_size=7):
    try:
        depth = turtle.get_depth_image()
    except Exception as exc:
        print("Failed to read depth image: {}".format(exc))
        return None

    if depth is None:
        print("Depth image is None.")
        return None

    if len(depth.shape) == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]

    col = int(round(cx))
    row = int(round(cy))
    h, w = depth.shape

    half_w = window_size // 2
    row_start = max(0, row - half_w)
    row_end = min(h, row + half_w + 1)
    col_start = max(0, col - half_w)
    col_end = min(w, col + half_w + 1)

    window = depth[row_start:row_end, col_start:col_end]
    valid = window[~np.isnan(window)]
    if valid.size == 0:
        print("All values in selected depth window are NaN.")
        return None

    return float(np.mean(valid))


def get_yellow_mask(turtle):
    hsv = get_hsv(turtle)
    if hsv is None:
        return None

    mask = create_hsv_mask(
        hsv,
        GARAGE_MIN_H,
        GARAGE_MAX_H,
        GARAGE_MIN_S,
        GARAGE_MAX_S,
        GARAGE_MIN_V,
        GARAGE_MAX_V,
    )

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_garage_opening_center(turtle, row_ratio=0.60, min_gap_width=80):
    """
    Detect the horizontal center of the largest gap without yellow color.
    Returns (gap_center_x, gap_width, scan_row) or None.
    """
    mask = get_yellow_mask(turtle)
    if mask is None:
        return None

    h, w = mask.shape
    row = int(round(h * row_ratio))
    row = max(0, min(h - 1, row))

    line = mask[row, :]
    yellow = line > 0
    non_yellow = ~yellow

    best_start = None
    best_end = None
    current_start = None

    for i, value in enumerate(non_yellow):
        if value and current_start is None:
            current_start = i
        elif (not value) and current_start is not None:
            current_end = i - 1
            if best_start is None or (current_end - current_start) > (best_end - best_start):
                best_start = current_start
                best_end = current_end
            current_start = None

    if current_start is not None:
        current_end = w - 1
        if best_start is None or (current_end - current_start) > (best_end - best_start):
            best_start = current_start
            best_end = current_end

    if best_start is None or best_end is None:
        return None

    gap_width = best_end - best_start + 1
    if gap_width < min_gap_width:
        return None

    gap_center_x = 0.5 * (best_start + best_end)
    return float(gap_center_x), int(gap_width), int(row)

