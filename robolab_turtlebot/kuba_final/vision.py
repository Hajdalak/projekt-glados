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
GARAGE_MIN_H = 20
GARAGE_MAX_H = 40
GARAGE_MIN_S = 100
GARAGE_MAX_S = 255
GARAGE_MIN_V = 70
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
    return cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)


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

    # Pro garaz je dulezite spis zacelit diry ve zlute stene,
    # ne odmazavat male oblasti zlute.
    kernel_close = np.ones((7, 7), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)

    return mask

def find_garage_opening_center(
    turtle,
    row_ratio=0.68,
    band_half_height=18,
    min_gap_width=50,
    min_wall_width=12,
    yellow_ratio_threshold=0.12,
):
    """
    Hleda otvor garaze jako mezeru bez zlute mezi dvema zlutymi stenami.
    Vraci (gap_center_x, gap_width, scan_row) nebo None.
    """
    mask = get_yellow_mask(turtle)
    if mask is None:
        return None

    h, w = mask.shape

    row = int(round(h * row_ratio))
    row = max(0, min(h - 1, row))

    y1 = max(0, row - band_half_height)
    y2 = min(h, row + band_half_height + 1)

    band = mask[y1:y2, :]
    if band.size == 0:
        return None

    col_sum = np.sum(band > 0, axis=0)

    threshold = max(1, int(yellow_ratio_threshold * band.shape[0]))
    yellow_cols = col_sum >= threshold
    free_cols = ~yellow_cols

    gaps = []
    in_gap = False
    start = 0

    for x in range(w):
        if free_cols[x] and not in_gap:
            start = x
            in_gap = True
        elif (not free_cols[x]) and in_gap:
            end = x - 1
            gaps.append((start, end))
            in_gap = False

    if in_gap:
        gaps.append((start, w - 1))

    valid_gaps = []

    for gs, ge in gaps:
        gap_width = ge - gs + 1
        if gap_width < min_gap_width:
            continue

        left_wall_width = 0
        i = gs - 1
        while i >= 0 and yellow_cols[i]:
            left_wall_width += 1
            i -= 1

        right_wall_width = 0
        i = ge + 1
        while i < w and yellow_cols[i]:
            right_wall_width += 1
            i += 1

        # ve fazi hledani bud tolerantnejsi
        if left_wall_width < min_wall_width or right_wall_width < min_wall_width:
            continue

        valid_gaps.append((gs, ge))

    if not valid_gaps:
        return None

    best_start, best_end = max(valid_gaps, key=lambda p: p[1] - p[0])

    gap_width = best_end - best_start + 1
    gap_center_x = 0.5 * (best_start + best_end)

    return float(gap_center_x), int(gap_width), int(row)